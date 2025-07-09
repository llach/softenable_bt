#pragma once

#include <string>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <behaviortree_cpp/bt_factory.h>

struct CartesianTarget
{
  geometry_msgs::msg::Pose pose;
  std::string frame_id;
};

namespace BT
{

template <>
inline CartesianTarget convertFromString(BT::StringView str)
{
    // Expected format: x;y;z;qx;qy;qz;qw;frame_id
    auto parts = BT::splitString(str, ';');
    if (parts.size() != 8)
    {
        throw BT::RuntimeError("invalid input for CartesianTarget. Expected 8 values (x;y;z;qx;qy;qz;qw;frame_id)");
    }

    CartesianTarget output;

    output.pose.position.x = BT::convertFromString<double>(parts[0]);
    output.pose.position.y = BT::convertFromString<double>(parts[1]);
    output.pose.position.z = BT::convertFromString<double>(parts[2]);

    output.pose.orientation.x = BT::convertFromString<double>(parts[3]);
    output.pose.orientation.y = BT::convertFromString<double>(parts[4]);
    output.pose.orientation.z = BT::convertFromString<double>(parts[5]);
    output.pose.orientation.w = BT::convertFromString<double>(parts[6]);

    output.frame_id = parts[7];

    return output;
}

} // namespace BT
