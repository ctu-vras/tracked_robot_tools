#pragma once

#include <ros/ros.h>
#include <xmlrpcpp/XmlRpcValue.h>

namespace tracked_robot_tools
{

/** \brief Represents a symmetrical flipper pose. */
struct SymmetricalFlipperPose {
  double front; /**< \brief Angle of front flippers [rad]. */
  double rear; /**< \brief Angle of rear flippers [rad]. */

  SymmetricalFlipperPose(double front, double rear);
};

class FlipperPostures
{
public:
  explicit FlipperPostures(ros::NodeHandle nh);
  explicit FlipperPostures(const XmlRpc::XmlRpcValue& value);
  
  template <typename T>
  const SymmetricalFlipperPose& get(const T& index) const
  {
    return this->postures.at(static_cast<size_t>(index));
  }
  
  const std::vector<SymmetricalFlipperPose>& all() const;
  
  size_t size() const;
  
protected:
  void parse(const XmlRpc::XmlRpcValue& value);
  
  std::vector<SymmetricalFlipperPose> postures;
};

}