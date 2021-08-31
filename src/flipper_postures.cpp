#include <tracked_robot_tools/flipper_postures.h>

using namespace tracked_robot_tools;

SymmetricalFlipperPose::SymmetricalFlipperPose(double front, double rear)
  : front(front), rear(rear)
{
}

FlipperPostures::FlipperPostures(ros::NodeHandle nh)
{
  XmlRpc::XmlRpcValue v;
  nh.getParam("flipper_postures", v);
  this->parse(v);
}

FlipperPostures::FlipperPostures(const XmlRpc::XmlRpcValue& value)
{
  this->parse(value);
}

void FlipperPostures::parse(const XmlRpc::XmlRpcValue& value)
{
  if (value.getType() != XmlRpc::XmlRpcValue::TypeArray)
    throw std::runtime_error("Invalid flipper_postures, it must be an array");
  
  for (size_t i = 0; i < value.size(); ++i)
  {
    const auto& item = value[static_cast<int>(i)];
    if (item.getType() != XmlRpc::XmlRpcValue::TypeArray || item.size() != 2 ||
      item[0].getType() != XmlRpc::XmlRpcValue::TypeDouble ||
      item[1].getType() != XmlRpc::XmlRpcValue::TypeDouble)
    {
      ROS_WARN_STREAM("Ignoring invalid flipper posture, it has to be an array of two doubles");
      continue;
    }
    
    this->postures.emplace_back(item[0], item[1]);
  }
  
  ROS_INFO("The following flipper postures are defined:");
  for (size_t i = 0; i < this->postures.size(); ++i)
  {
    ROS_INFO_STREAM(" - " << i << ") front: " << this->postures[i].front << ", rear: " << this->postures[i].rear);
  }
}

const std::vector<SymmetricalFlipperPose>& FlipperPostures::all() const
{
  return this->postures;
}

size_t FlipperPostures::size() const
{
  return this->postures.size();
}
