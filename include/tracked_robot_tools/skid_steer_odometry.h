#pragma once

/**
 * \file
 * \brief Skid-steering 2D odometry model.
 * \author Martin Pecka
 * SPDX-License-Identifier: BSD-3-Clause
 * SPDX-FileCopyrightText: Czech Technical University in Prague
 */

#include <memory>

#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseWithCovariance.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <nav_msgs/Odometry.h>
#include <ros/duration.h>
#include <ros/time.h>

#include <cras_cpp_common/log_utils.h>
#include <cras_cpp_common/log_utils/node.h>
#include <cras_cpp_common/optional.hpp>

#include <tracked_robot_tools/kinematics.h>

namespace tracked_robot_tools
{

/** \brief Skid-steering 2D odometry model. */
class SkidSteerOdometry
{
public:
  explicit SkidSteerOdometry(std::shared_ptr<Kinematics2D> kinematics, const std::string& parentFrame,
    const std::string& childFrame, cras::LogHelperPtr log = std::make_shared<cras::NodeLogHelper>());
  
  nav_msgs::Odometry addMeasurement(double vl, double vr, const ros::Time& stamp);
  
  virtual nav_msgs::Odometry addMeasurement(double vl, double vr, const ros::Time& stamp,
    const cras::optional<double>& yawRate);
  
  void setSlip(double slipLeft, double slipRight);
  
  virtual void setIntegrationMethod(const std::string& method);
  void setVelocityScale(double left, double right);
  
  virtual void reset();

  void setInitialPose(const geometry_msgs::Pose& pose);
  void setInitialPose(const geometry_msgs::PoseWithCovariance & pose);
  
protected:
  
  typedef std::function<void(double, double, const ros::Duration&, const cras::optional<double>&)>
    IntegrateMeasurementFn;
  
  void addMeasurementEuler(double vl, double vr, const ros::Duration& dt, const cras::optional<double>& yawRate);
  void addMeasurementEulerIMU(double vl, double vr, const ros::Duration& dt, const cras::optional<double>& yawRate);
  void addMeasurementChongKleeman(double vl, double vr, const ros::Duration& dt, const cras::optional<double>& yawRate);
  void addMeasurementRungeKutta1(double vl, double vr, const ros::Duration& dt, const cras::optional<double>& yawRate);
  void addMeasurementRungeKutta4(double vl, double vr, const ros::Duration& dt, const cras::optional<double>& yawRate);
  
  cras::LogHelperPtr getCrasLogger();
  
  IntegrateMeasurementFn integrateMeasurement;
  std::string integrationMethodName {"Euler"};
  
  std::shared_ptr<Kinematics2D> kinematics;
  nav_msgs::Odometry odom;
  nav_msgs::Odometry initOdom;
  double yaw {0.0};
  ros::Time lastStamp {0, 0};
  double slipLeft {1.0};
  double slipRight {1.0};
  double velocityScaleLeft {1.0};
  double velocityScaleRight {1.0};
  cras::optional<double> lastLinearVel;
  cras::optional<double> lastAngularVel;

  cras::LogHelperPtr log;
};

}