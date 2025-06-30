#include "softenable_bt/types/cartesian_target.hpp"
#include <behaviortree_cpp/bt_factory.h>
#include <behaviortree_cpp/utils/serialization.hpp>

namespace BT {
template <>
inline CartesianTarget convertFromString(StringView str)
{
  auto parts = BT::splitString(str, ';');
  if (parts.size() != 8) {
    throw RuntimeError("Invalid format for CartesianTarget. Expected x;y;z;qx;qy;qz;qw;frame_id");
  }

  CartesianTarget target;
  target.pose.position.x = std::stod(parts[0]);
  target.pose.position.y = std::stod(parts[1]);
  target.pose.position.z = std::stod(parts[2]);
  target.pose.orientation.x = std::stod(parts[3]);
  target.pose.orientation.y = std::stod(parts[4]);
  target.pose.orientation.z = std::stod(parts[5]);
  target.pose.orientation.w = std::stod(parts[6]);
  target.frame_id = parts[7];

  return target;
}
}  // namespace BT
