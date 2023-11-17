/**
 * \file
 * \brief Skid-steering 2D odometry model.
 * \author Martin Pecka
 * SPDX-License-Identifier: BSD-3-Clause
 * SPDX-FileCopyrightText: Czech Technical University in Prague
 */

#include <utility>

#include <angles/angles.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <cras_cpp_common/functional.hpp>
#include <cras_cpp_common/math_utils.hpp>
#include <cras_cpp_common/optional.hpp>
#include <cras_cpp_common/tf2_utils.hpp>

#include <tracked_robot_tools/skid_steer_odometry.h>

namespace tracked_robot_tools
{

SkidSteerOdometry::SkidSteerOdometry(std::shared_ptr<Kinematics2D> kinematics, const std::string& parentFrame,
  const std::string& childFrame, cras::LogHelperPtr log) : kinematics(std::move(kinematics)), log(std::move(log))
{
  this->odom.header.frame_id = parentFrame;
  this->odom.child_frame_id = childFrame;
  this->odom.pose.pose.orientation.w = 1;
  this->initOdom = this->odom;
  this->integrateMeasurement = cras::bind_front(&SkidSteerOdometry::addMeasurementEuler, this);
}

nav_msgs::Odometry SkidSteerOdometry::addMeasurement(const double vl, const double vr, const ros::Time& stamp)
{
  return this->addMeasurement(vl, vr, stamp, {});
}

nav_msgs::Odometry SkidSteerOdometry::addMeasurement(const double vl, const double vr, const ros::Time& stamp,
  const cras::optional<double>& yawRate)
{
  if (this->lastStamp == ros::Time(0, 0))
  {
    this->lastStamp = this->odom.header.stamp = stamp;
    return this->odom;
  }

  this->integrateMeasurement(this->velocityScaleLeft * vl, this->velocityScaleRight * vr,
    stamp - this->lastStamp, yawRate);

  this->lastStamp = this->odom.header.stamp = stamp;
  return this->odom;
}

void SkidSteerOdometry::setSlip(const double slipLeft, const double slipRight)
{
  this->slipLeft = slipLeft;
  this->slipRight = slipRight;
}

void SkidSteerOdometry::addMeasurementEuler(double vl, double vr, const ros::Duration& dt,
  const cras::optional<double>&)
{
  auto& pos = this->odom.pose.pose.position;
  auto& rot = this->odom.pose.pose.orientation;
  double v, w;
  this->kinematics->tracksToTwist(vl, vr, &v, &w);

  if (!this->lastLinearVel.has_value() || !this->lastAngularVel.has_value())
  {
    this->lastLinearVel = v;
    this->lastAngularVel = w;
  }
  else
  {
    const auto origV = v;
    const auto origW = w;
    v = (v + this->lastLinearVel.value()) / 2;
    w = (w + this->lastAngularVel.value()) / 2;
    this->lastLinearVel = origV;
    this->lastAngularVel = origW;
  }
  
  const auto dtSec = dt.toSec();
  const auto vx = v * std::cos(this->yaw);
  const auto vy = v * std::sin(this->yaw);

  const auto dl = vl * dtSec;
  const auto dr = vr * dtSec;
  const auto dx = vx * dtSec;
  const auto dy = vy * dtSec;
  const auto dv = v * dtSec;
  const auto dw = w * dtSec;

  const auto direction = this->yaw + dw;
  
  pos.x += vx * dtSec;
  pos.y += vy * dtSec;
  this->yaw += w * dtSec;
  this->yaw = angles::normalize_angle(this->yaw);
  rot.z = std::sin(this->yaw / 2.0);
  rot.w = std::cos(this->yaw / 2.0);
  
  this->odom.twist.twist.linear.x = v;
  this->odom.twist.twist.linear.y = 0;
  this->odom.twist.twist.angular.z = w;

  auto& cov_w = this->odom.twist.covariance;
  cov_w.fill(0);
  cov_w[2 * 6 + 2] = 1e-2;
  cov_w[3 * 6 + 3] = cov_w[4 * 6 + 4] = std::pow(M_PI / 10, 2);

  Eigen::Matrix2d tracksCov;
  Eigen::Matrix<double, 3, 2> velocityJacobian;
  tracksCov << (dr * dr), 0, 0, (dl * dl);
  const auto sinDir = std::sin(direction);
  const auto cosDir = std::cos(direction);
  velocityJacobian << cosDir / 2, cosDir / 2,
    sinDir / 2, sinDir / 2,
    1 / this->kinematics->getTracksDistance(), -1 / this->kinematics->getTracksDistance();

  Eigen::Matrix3d velocityCov = velocityJacobian * tracksCov * velocityJacobian.transpose();
  cov_w[0 * 6 + 0] = velocityCov(0, 0);
  cov_w[0 * 6 + 1] = velocityCov(0, 1);
  cov_w[0 * 6 + 5] = velocityCov(0, 2);
  cov_w[1 * 6 + 0] = velocityCov(1, 0);
  cov_w[1 * 6 + 1] = velocityCov(1, 1);
  cov_w[1 * 6 + 5] = velocityCov(1, 2);
  cov_w[5 * 6 + 0] = velocityCov(2, 0);
  cov_w[5 * 6 + 1] = velocityCov(2, 1);
  cov_w[5 * 6 + 5] = velocityCov(2, 2);

  Eigen::Matrix3d poseJacobian;
  poseJacobian << 1, 0, -std::abs(dy), 0, 1, std::abs(dx), 0, 0, 1;  // TODO figure out why abs() is needed
  // poseJacobian << 1, 0, -dy, 0, 1, dx, 0, 0, 1;

  auto& cov_v = this->odom.pose.covariance;
  cov_v[2 * 6 + 2] = std::pow(std::sqrt(cov_v[2 * 6 + 2]) + dtSec * std::sqrt(cov_w[2 * 6 + 2]), 2);
  cov_v[3 * 6 + 3] = cov_v[4 * 6 + 4] = M_PI * M_PI;
  Eigen::Matrix3d poseCov;
  poseCov << cov_v[0 * 6 + 0], cov_v[0 * 6 + 1], cov_v[0 * 6 + 5],
    cov_v[1 * 6 + 0], cov_v[1 * 6 + 1], cov_v[1 * 6 + 5],
    cov_v[5 * 6 + 0], cov_v[5 * 6 + 1], cov_v[5 * 6 + 5];
  poseCov = poseJacobian * poseCov * poseJacobian.transpose() + velocityCov;

  cov_v[0 * 6 + 0] = poseCov(0, 0);
  cov_v[0 * 6 + 1] = poseCov(0, 1);
  cov_v[0 * 6 + 5] = poseCov(0, 2);
  cov_v[1 * 6 + 0] = poseCov(1, 0);
  cov_v[1 * 6 + 1] = poseCov(1, 1);
  cov_v[1 * 6 + 5] = poseCov(1, 2);
  cov_v[5 * 6 + 0] = poseCov(2, 0);
  cov_v[5 * 6 + 1] = poseCov(2, 1);
  cov_v[5 * 6 + 5] = poseCov(2, 2);
}

void SkidSteerOdometry::addMeasurementEulerIMU(double vl, double vr, const ros::Duration& dt,
  const cras::optional<double>& yawRate)
{
  auto& pos = this->odom.pose.pose.position;
  auto& rot = this->odom.pose.pose.orientation;
  double v, w;
  this->kinematics->tracksToTwist(vl, vr, &v, &w);
  if (yawRate.has_value())
  {
    w = yawRate.value();
//    // recompute track velocities to be consistent with the computed v and the IMU w
//    const auto vrO = vr;
//    const auto vlO = vl;
//    const auto vO = v;
//    const auto wO = w;
//    this->kinematics->twistToTracks(&vl, &vr, v, w);
//    auto ar = 1 - vr / vrO;
//    auto al = 1 - vl / vlO;
//    ROS_INFO("al = %.4f, ar = %.4f, al/ar = %.6f, v = %.3f/%.3f, w = %.3f/%.3f", al, ar, al/ar, v, vO, w, wO);
//    const auto num = -::sgn(vlO * vrO) * std::sqrt(std::abs(vrO / vlO));
//    const auto A = -num;
//    const auto alO = al;
//    const auto arO = ar;
//    if (std::abs(vrO) > 0.001 && std::abs(vlO) > 0.001)
//    {
//      al = (w * this->kinematics->getTracksDistance() + vlO - vrO) * A / (vrO + vlO * A);
//      ar = 1 - (w * this->kinematics->getTracksDistance() + vlO * (1 - al)) / vrO;
//      vl = vlO * (1 - al);
//      vr = vrO * (1 - ar);
//      this->kinematics->tracksToTwist(vl, vr, &v, &w);
//    }
//    ROS_INFO("al = %.4f/%.4f, ar = %.4f/%.4f, al/ar = %.6f, num = %.6f, alO/arO = %.6f, v = %.3f/%.3f, w = %.3f/%.3f",
//             al, alO, ar, arO, al/ar, num, alO/arO, v, vO, w, wO);
  }

  if (!this->lastLinearVel.has_value() || !this->lastAngularVel.has_value())
  {
    this->lastLinearVel = v;
    this->lastAngularVel = w;
  }
  else
  {
    const auto origV = v;
    const auto origW = w;
    v = (v + this->lastLinearVel.value()) / 2;
    w = (w + this->lastAngularVel.value()) / 2;
    this->lastLinearVel = origV;
    this->lastAngularVel = origW;
  }
  
  const auto dtSec = dt.toSec();
  const auto vx = v * std::cos(this->yaw);
  const auto vy = v * std::sin(this->yaw);
  
  const auto dl = vl * dtSec;
  const auto dr = vr * dtSec;
  const auto dx = vx * dtSec;
  const auto dy = vy * dtSec;
  const auto dv = v * dtSec;
  const auto dw = w * dtSec;

//  const auto direction = this->yaw + dw;
  const auto direction = this->yaw + dw / 2;

  pos.x += dx;
  pos.y += dy;
  this->yaw += dw;
  this->yaw = angles::normalize_angle(this->yaw);
  rot.z = std::sin(this->yaw / 2.0);
  rot.w = std::cos(this->yaw / 2.0);
  
  this->odom.twist.twist.linear.x = v;
  this->odom.twist.twist.linear.y = 0;
  this->odom.twist.twist.angular.z = w;
  
  Eigen::Matrix2d tracksCov;
  Eigen::Matrix<double, 3, 2> velocityJacobian;
//  tracksCov << (1e-2 * std::abs(dl)), 0, 0, (1e-2 * std::abs(dr));
//  tracksCov << std::abs(dl), 0, 0, std::abs(dr);
  tracksCov << (dr * dr), 0, 0, (dl * dl);
  const auto sinDir = std::sin(direction);
  const auto cosDir = std::cos(direction);
  const auto v2b = dv / (2 * this->kinematics->getTracksDistance());
  velocityJacobian << cosDir / 2 - sinDir * v2b, cosDir / 2 + sinDir * v2b,
                      sinDir / 2 + cosDir * v2b, sinDir / 2 - cosDir * v2b,
                      1 / this->kinematics->getTracksDistance(), -1 / this->kinematics->getTracksDistance();
  
  Eigen::Matrix3d velocityCov = velocityJacobian * tracksCov * velocityJacobian.transpose();

  auto& cov_w = this->odom.twist.covariance;
  cov_w.fill(0);
  cov_w[0 * 6 + 0] = velocityCov(0, 0);
  cov_w[0 * 6 + 5] = velocityCov(0, 2);
  cov_w[1 * 6 + 0] = velocityCov(1, 0);
  cov_w[1 * 6 + 1] = velocityCov(1, 1);
  cov_w[1 * 6 + 5] = velocityCov(1, 2);
  cov_w[5 * 6 + 0] = velocityCov(2, 0);
  cov_w[5 * 6 + 1] = velocityCov(2, 1);
  cov_w[5 * 6 + 5] = 0.32 * 0.32;

  // Rotate the covariance to base_link
  tf2::Transform t(tf2::Quaternion(tf2::Vector3(0, 0, 1), -direction));
  cov_w = tf2::transformCovariance(cov_w, t);

  // Constant terms of the covariance TODO: parametrize
  cov_w[2 * 6 + 2] = 1e-2;
  cov_w[3 * 6 + 3] = cov_w[4 * 6 + 4] = std::pow(M_PI / 10, 2);
  
  Eigen::Matrix3d poseJacobian;
  poseJacobian << 1, 0, -std::abs(dy), 0, 1, std::abs(dx), 0, 0, 1;  // TODO figure out why abs() is needed
  //  poseJacobian << 1, 0, -dy, 0, 1, dx, 0, 0, 1;
  
  auto& cov_v = this->odom.pose.covariance;
  cov_v[2 * 6 + 2] = std::pow(std::sqrt(cov_v[2 * 6 + 2]) + dtSec * std::sqrt(cov_w[2 * 6 + 2]), 2);
  cov_v[3 * 6 + 3] = cov_v[4 * 6 + 4] = M_PI * M_PI;
  Eigen::Matrix3d poseCov;
  poseCov << cov_v[0 * 6 + 0], cov_v[0 * 6 + 1], cov_v[0 * 6 + 5],
             cov_v[1 * 6 + 0], cov_v[1 * 6 + 1], cov_v[1 * 6 + 5],
             cov_v[5 * 6 + 0], cov_v[5 * 6 + 1], cov_v[5 * 6 + 5];
  poseCov = poseJacobian * poseCov * poseJacobian.transpose() + velocityCov;
  
  cov_v[0 * 6 + 0] = poseCov(0, 0);
  cov_v[0 * 6 + 1] = poseCov(0, 1);
  cov_v[0 * 6 + 5] = poseCov(0, 2);
  cov_v[1 * 6 + 0] = poseCov(1, 0);
  cov_v[1 * 6 + 1] = poseCov(1, 1);
  cov_v[1 * 6 + 5] = poseCov(1, 2);
  cov_v[5 * 6 + 0] = poseCov(2, 0);
  cov_v[5 * 6 + 1] = poseCov(2, 1);
  cov_v[5 * 6 + 5] = poseCov(2, 2);
}

void SkidSteerOdometry::addMeasurementChongKleeman(double vl, double vr, const ros::Duration& dt,
  const cras::optional<double>& yawRate)
{
  auto& pos = this->odom.pose.pose.position;
  auto& rot = this->odom.pose.pose.orientation;
  double v, w;
  this->kinematics->tracksToTwist(vl, vr, &v, &w);

  const auto dtSec = dt.toSec();
  const auto vx = v * std::cos(this->yaw);
  const auto vy = v * std::sin(this->yaw);
  
  const auto dl = vl * dtSec;
  const auto dr = vr * dtSec;
  const auto dx = vx * dtSec;
  const auto dy = vy * dtSec;
  const auto dv = v * dtSec;
  const auto dw = w * dtSec;
  const auto b = this->kinematics->getTracksDistance();
  const auto r = -v / w;
  ROS_INFO_STREAM(r << " " << dw);

  if (std::abs(r) > 100)
  {
    pos.x += dx;
    pos.y += dy;
  }
  else
  {
    pos.x += r * (std::sin(this->yaw) - std::sin(this->yaw + dw));
    pos.y += r * (std::cos(this->yaw + dw) - std::cos(this->yaw));
  }

  this->yaw += dw;
  this->yaw = angles::normalize_angle(this->yaw);
  rot.z = std::sin(this->yaw / 2.0);
  rot.w = std::cos(this->yaw / 2.0);
  
  this->odom.twist.twist.linear.x = v;
  this->odom.twist.twist.linear.y = 0;
  this->odom.twist.twist.angular.z = w;
  
  auto& cov_w = this->odom.twist.covariance;
  cov_w.fill(0);
  cov_w[2 * 6 + 2] = 1e-2;
  cov_w[3 * 6 + 3] = cov_w[4 * 6 + 4] = std::pow(M_PI / 10, 2);
  cov_w[5 * 6 + 5] = 0.32 * 0.32;
  
  Eigen::Matrix2d tracksCov;
  Eigen::Matrix<double, 3, 2> velocityJacobian;
//  tracksCov << (1e-2 * std::abs(dl)), 0, 0, (1e-2 * std::abs(dr));
//  tracksCov << std::abs(dl), 0, 0, std::abs(dr);
  tracksCov << (dr * dr), 0, 0, (dl * dl);
  const auto direction = this->yaw + dw;
  const auto sinDir = std::sin(direction);
  const auto cosDir = std::cos(direction);
  const auto v2b = dv / (2 * b);
//  velocityJacobian << cosDir / 2 - sinDir * v2b, cosDir / 2 + sinDir * v2b,
//                      sinDir / 2 + cosDir * v2b, sinDir / 2 - cosDir * v2b,
//                      1 / this->kinematics->getTracksDistance(), -1 / this->kinematics->getTracksDistance();
  velocityJacobian << cosDir / 2, cosDir / 2,
                      sinDir / 2, sinDir / 2,
                      1 / b, -1 / b;
  
  Eigen::Matrix3d velocityCov = velocityJacobian * tracksCov * velocityJacobian.transpose();
  cov_w[0 * 6 + 0] = velocityCov(0, 0);
  cov_w[0 * 6 + 1] = velocityCov(0, 1);
  cov_w[0 * 6 + 5] = velocityCov(0, 2);
  cov_w[1 * 6 + 0] = velocityCov(1, 0);
  cov_w[1 * 6 + 1] = velocityCov(1, 1);
  cov_w[1 * 6 + 5] = velocityCov(1, 2);
  cov_w[5 * 6 + 0] = velocityCov(2, 0);
  cov_w[5 * 6 + 1] = velocityCov(2, 1);
//  cov_w[5 * 6 + 5] = velocityCov(2, 2);
  
  Eigen::Matrix3d poseJacobian;
  poseJacobian << 1, 0, -std::abs(dy), 0, 1, std::abs(dx), 0, 0, 1;
  
  auto& cov_v = this->odom.pose.covariance;
  cov_v[2 * 6 + 2] = std::pow(std::sqrt(cov_v[2 * 6 + 2]) + dtSec * std::sqrt(cov_w[2 * 6 + 2]), 2);
  cov_v[3 * 6 + 3] = cov_v[4 * 6 + 4] = M_PI * M_PI;
  Eigen::Matrix3d poseCov;
  poseCov << cov_v[0 * 6 + 0], cov_v[0 * 6 + 1], cov_v[0 * 6 + 5],
             cov_v[1 * 6 + 0], cov_v[1 * 6 + 1], cov_v[1 * 6 + 5],
             cov_v[5 * 6 + 0], cov_v[5 * 6 + 1], cov_v[5 * 6 + 5];
  poseCov = poseJacobian * poseCov * poseJacobian.transpose() + velocityCov;
  
  cov_v[0 * 6 + 0] = poseCov(0, 0);
  cov_v[0 * 6 + 1] = poseCov(0, 1);
  cov_v[0 * 6 + 5] = poseCov(0, 2);
  cov_v[1 * 6 + 0] = poseCov(1, 0);
  cov_v[1 * 6 + 1] = poseCov(1, 1);
  cov_v[1 * 6 + 5] = poseCov(1, 2);
  cov_v[5 * 6 + 0] = poseCov(2, 0);
  cov_v[5 * 6 + 1] = poseCov(2, 1);
  cov_v[5 * 6 + 5] = poseCov(2, 2);
}

void SkidSteerOdometry::addMeasurementRungeKutta1(double vl, double vr, const ros::Duration& dt,
  const cras::optional<double>&)
{
  auto& pos = this->odom.pose.pose.position;
  auto& rot = this->odom.pose.pose.orientation;
  double v, w;
  this->kinematics->tracksToTwist(vl, vr, &v, &w);
  const auto dtSec = dt.toSec();

  const double direction = this->yaw + (w * 0.5 * dtSec);
  
  const auto vx = v * std::cos(direction);
  const auto vy = v * std::sin(direction);

  const auto dl = vl * dtSec;
  const auto dr = vr * dtSec;
  const auto dx = vx * dtSec;
  const auto dy = vy * dtSec;
  const auto dv = v * dtSec;
  const auto dw = w * dtSec;
  
  pos.x += vx * dtSec;
  pos.y += vy * dtSec;
  this->yaw += w * dtSec;
  this->yaw = angles::normalize_angle(this->yaw);
  rot.z = std::sin(this->yaw / 2.0);
  rot.w = std::cos(this->yaw / 2.0);
  
  this->odom.twist.twist.linear.x = v;
  this->odom.twist.twist.linear.y = 0;
  this->odom.twist.twist.angular.z = w;

  auto& cov_w = this->odom.twist.covariance;
  cov_w.fill(0);
  cov_w[2 * 6 + 2] = 1e-2;
  cov_w[3 * 6 + 3] = cov_w[4 * 6 + 4] = std::pow(M_PI / 10, 2);

  Eigen::Matrix2d tracksCov;
  Eigen::Matrix<double, 3, 2> velocityJacobian;
  tracksCov << (dr * dr), 0, 0, (dl * dl);
  const auto sinDir = std::sin(direction);
  const auto cosDir = std::cos(direction);
  const auto v2b = dv / (2 * this->kinematics->getTracksDistance());
  velocityJacobian << cosDir / 2 - sinDir * v2b, cosDir / 2 + sinDir * v2b,
                      sinDir / 2 + cosDir * v2b, sinDir / 2 - cosDir * v2b,
                      1 / this->kinematics->getTracksDistance(), -1 / this->kinematics->getTracksDistance();

  Eigen::Matrix3d velocityCov = velocityJacobian * tracksCov * velocityJacobian.transpose();
  cov_w[0 * 6 + 0] = velocityCov(0, 0);
  cov_w[0 * 6 + 1] = velocityCov(0, 1);
  cov_w[0 * 6 + 5] = velocityCov(0, 2);
  cov_w[1 * 6 + 0] = velocityCov(1, 0);
  cov_w[1 * 6 + 1] = velocityCov(1, 1);
  cov_w[1 * 6 + 5] = velocityCov(1, 2);
  cov_w[5 * 6 + 0] = velocityCov(2, 0);
  cov_w[5 * 6 + 1] = velocityCov(2, 1);
  cov_w[5 * 6 + 5] = velocityCov(2, 2);

  Eigen::Matrix3d poseJacobian;
  poseJacobian << 1, 0, -std::abs(dy), 0, 1, std::abs(dx), 0, 0, 1;  // TODO figure out why abs() is needed
  //poseJacobian << 1, 0, -dy, 0, 1, dx, 0, 0, 1;

  auto& cov_v = this->odom.pose.covariance;
  cov_v[2 * 6 + 2] = std::pow(std::sqrt(cov_v[2 * 6 + 2]) + dtSec * std::sqrt(cov_w[2 * 6 + 2]), 2);
  cov_v[3 * 6 + 3] = cov_v[4 * 6 + 4] = M_PI * M_PI;
  Eigen::Matrix3d poseCov;
  poseCov << cov_v[0 * 6 + 0], cov_v[0 * 6 + 1], cov_v[0 * 6 + 5],
    cov_v[1 * 6 + 0], cov_v[1 * 6 + 1], cov_v[1 * 6 + 5],
    cov_v[5 * 6 + 0], cov_v[5 * 6 + 1], cov_v[5 * 6 + 5];
  poseCov = poseJacobian * poseCov * poseJacobian.transpose() + velocityCov;

  cov_v[0 * 6 + 0] = poseCov(0, 0);
  cov_v[0 * 6 + 1] = poseCov(0, 1);
  cov_v[0 * 6 + 5] = poseCov(0, 2);
  cov_v[1 * 6 + 0] = poseCov(1, 0);
  cov_v[1 * 6 + 1] = poseCov(1, 1);
  cov_v[1 * 6 + 5] = poseCov(1, 2);
  cov_v[5 * 6 + 0] = poseCov(2, 0);
  cov_v[5 * 6 + 1] = poseCov(2, 1);
  cov_v[5 * 6 + 5] = poseCov(2, 2);
}

void SkidSteerOdometry::addMeasurementRungeKutta4(double vl, double vr, const ros::Duration& dt,
  const cras::optional<double>&)
{
  auto& pos = this->odom.pose.pose.position;
  auto& rot = this->odom.pose.pose.orientation;
  double v, w;
  this->kinematics->tracksToTwist(vl, vr, &v, &w);
  const auto dtSec = dt.toSec();

  const double halfYaw = this->yaw + (w * 0.5 * dtSec);
  const auto endYaw = this->yaw + dtSec * w;

  const auto vx = (v * std::cos(this->yaw) + 4 * v * std::cos(halfYaw) + v * std::cos(endYaw)) / 6;
  const auto vy = (v * std::sin(this->yaw) + 4 * v * std::sin(halfYaw) + v * std::sin(endYaw)) / 6;
  
  pos.x += vx * dtSec;
  pos.y += vy * dtSec;
  this->yaw = angles::normalize_angle(endYaw);
  rot.z = std::sin(this->yaw / 2.0);
  rot.w = std::cos(this->yaw / 2.0);
  
  this->odom.twist.twist.linear.x = v;
  this->odom.twist.twist.linear.y = 0;
  this->odom.twist.twist.angular.z = w;
}

void SkidSteerOdometry::setIntegrationMethod(const std::string& method)
{
  this->integrationMethodName = method;
  if (method == "RK1")
  {
    this->integrateMeasurement = cras::bind_front(&SkidSteerOdometry::addMeasurementRungeKutta1, this);
  }
  else if (method == "RK4")
  {
    this->integrateMeasurement = cras::bind_front(&SkidSteerOdometry::addMeasurementRungeKutta4, this);
  }
  else if (method == "EulerIMU")
  {
    this->integrateMeasurement = cras::bind_front(&SkidSteerOdometry::addMeasurementEulerIMU, this);
  }
  else if (method == "ChongKleeman")
  {
    this->integrateMeasurement = cras::bind_front(&SkidSteerOdometry::addMeasurementChongKleeman, this);
  }
  else
  {
    this->integrateMeasurement = cras::bind_front(&SkidSteerOdometry::addMeasurementEuler, this);
    this->integrationMethodName = "Euler";
    if (method != this->integrationMethodName)
      CRAS_WARN("Unknown integration method %s. Using %s instead.",
        method.c_str(), this->integrationMethodName.c_str());
  }
  CRAS_INFO("Using %s measurement integration method.", this->integrationMethodName.c_str());
}

void SkidSteerOdometry::reset()
{
  this->odom = this->initOdom;
  double roll, pitch;
  cras::getRPY(this->odom.pose.pose.orientation, roll, pitch, this->yaw);
  this->lastStamp = {0, 0};
  this->lastLinearVel.reset();
  this->lastAngularVel.reset();
}

void SkidSteerOdometry::setInitialPose(const geometry_msgs::Pose& pose)
{
  this->initOdom.pose.pose = pose;
  this->reset();
}

void SkidSteerOdometry::setInitialPose(const geometry_msgs::PoseWithCovariance& pose)
{
  this->initOdom.pose = pose;
  this->reset();
}

void SkidSteerOdometry::setVelocityScale(double left, double right)
{
  this->velocityScaleLeft = left;
  this->velocityScaleRight = right;
}

cras::LogHelperPtr SkidSteerOdometry::getCrasLogger()
{
  return this->log;
}

}
