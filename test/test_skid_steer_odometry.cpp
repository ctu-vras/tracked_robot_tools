/**
 * \file
 * \brief 
 * \author Martin Pecka
 * SPDX-License-Identifier: BSD-3-Clause
 * SPDX-FileCopyrightText: Czech Technical University in Prague
 */

#include "gtest/gtest.h"

#include <tracked_robot_tools/skid_steer_odometry.h>

using namespace tracked_robot_tools;

#define EXPECT_ODOM_NEAR(exp, cur) \
EXPECT_EQ((exp).header.stamp, (cur).header.stamp); \
EXPECT_EQ((exp).header.frame_id, (cur).header.frame_id); \
EXPECT_NEAR((exp).pose.pose.position.x, (cur).pose.pose.position.x, 1e-6); \
EXPECT_NEAR((exp).pose.pose.position.y, (cur).pose.pose.position.y, 1e-6); \
EXPECT_NEAR((exp).pose.pose.position.z, (cur).pose.pose.position.z, 1e-6); \
EXPECT_NEAR((exp).pose.pose.orientation.x, (cur).pose.pose.orientation.x, 1e-6); \
EXPECT_NEAR((exp).pose.pose.orientation.y, (cur).pose.pose.orientation.y, 1e-6); \
EXPECT_NEAR((exp).pose.pose.orientation.z, (cur).pose.pose.orientation.z, 1e-6); \
EXPECT_NEAR((exp).pose.pose.orientation.w, (cur).pose.pose.orientation.w, 1e-6); \
EXPECT_NEAR((exp).twist.twist.linear.x, (cur).twist.twist.linear.x, 1e-6); \
EXPECT_NEAR((exp).twist.twist.linear.y, (cur).twist.twist.linear.y, 1e-6); \
EXPECT_NEAR((exp).twist.twist.linear.z, (cur).twist.twist.linear.z, 1e-6); \
EXPECT_NEAR((exp).twist.twist.angular.x, (cur).twist.twist.angular.x, 1e-6); \
EXPECT_NEAR((exp).twist.twist.angular.y, (cur).twist.twist.angular.y, 1e-6); \
EXPECT_NEAR((exp).twist.twist.angular.z, (cur).twist.twist.angular.z, 1e-6);

TEST(SkidSteerOdom, Euler)
{
  auto kinematics = std::make_shared<Kinematics2D>(0.5, 0.9);
  SkidSteerOdometry odom(kinematics, "odom2d", "base_link");
  
  nav_msgs::Odometry expected;
  expected.header.frame_id = "odom2d";
  expected.child_frame_id = "base_link";
  expected.pose.pose.orientation.w = 1;
  
  auto msg = odom.addMeasurement(1, 1, {1, 0});
  expected.header.stamp.sec = 1;
  EXPECT_ODOM_NEAR(expected, msg);
  
  msg = odom.addMeasurement(1, 1, {2, 0});
  expected.header.stamp.sec = 2;
  expected.pose.pose.position.x = 1;
  expected.twist.twist.linear.x = 1;
  EXPECT_ODOM_NEAR(expected, msg);
  
  msg = odom.addMeasurement(-1, -1, {3, 0});
  expected.header.stamp.sec = 3;
  expected.pose.pose.position.x = 0;
  expected.twist.twist.linear.x = -1;
  EXPECT_ODOM_NEAR(expected, msg);
  
  msg = odom.addMeasurement(0.5, 0.5, {4, 0});
  expected.header.stamp.sec = 4;
  expected.pose.pose.position.x = 0.5;
  expected.twist.twist.linear.x = 0.5;
  EXPECT_ODOM_NEAR(expected, msg);
  
  msg = odom.addMeasurement(0.5, -0.5, {5, 0});
  expected.header.stamp.sec = 5;
  expected.pose.pose.orientation.z = std::sin(-0.9);
  expected.pose.pose.orientation.w = std::cos(-0.9);
  expected.twist.twist.linear.x = 0;
  expected.twist.twist.angular.z = -1.8;
  EXPECT_ODOM_NEAR(expected, msg);
  
  msg = odom.addMeasurement(-0.5, 0.5, {6, 0});
  expected.header.stamp.sec = 6;
  expected.pose.pose.orientation.z = 0;
  expected.pose.pose.orientation.w = 1;
  expected.twist.twist.linear.x = 0;
  expected.twist.twist.angular.z = 1.8;
  EXPECT_ODOM_NEAR(expected, msg);
  
  msg = odom.addMeasurement(0, 0.5, {7, 0});
  expected.header.stamp.sec = 7;
  expected.pose.pose.position.x = 0.75;
  // pos.y is 0 in Euler integration in this step
  expected.pose.pose.orientation.z = std::sin(0.45);
  expected.pose.pose.orientation.w = std::cos(0.45);
  expected.twist.twist.linear.x = 0.25;
  expected.twist.twist.angular.z = 0.9;
  EXPECT_ODOM_NEAR(expected, msg);
  
  msg = odom.addMeasurement(0, 0.5, {8, 0});
  expected.header.stamp.sec = 8;
  expected.pose.pose.position.x = 0.905402;
  expected.pose.pose.position.y = 0.195832;
  expected.pose.pose.orientation.z = std::sin(0.9);
  expected.pose.pose.orientation.w = std::cos(0.9);
  expected.twist.twist.linear.x = 0.1554024;
  expected.twist.twist.linear.y = 0.195831;
  expected.twist.twist.angular.z = 0.9;
  EXPECT_ODOM_NEAR(expected, msg);
}

int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}