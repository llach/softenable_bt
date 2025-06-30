#pragma once
#include <geometry_msgs/msg/pose.hpp>
#include <string>

struct CartesianTarget {
  geometry_msgs::msg::Pose pose;
  std::string frame_id;
};
