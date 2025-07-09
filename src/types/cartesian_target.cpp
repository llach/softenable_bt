#include "softenable_bt/types/cartesian_target.hpp"
#include <behaviortree_cpp/bt_factory.h>

namespace BT
{
template <>
inline CartesianTarget convertFromString(StringView str)
{
    // Expected format: x;y;z;qx;qy;qz;qw;frame_id
    auto parts = splitString(str, ';');
    if (parts.size() != 8)
    {
        throw RuntimeError("invalid input for CartesianTarget. Expected 8 values (x;y;z;qx;qy;qz;qw;frame_id)");
    }

    CartesianTarget output;

    output.pose.position.x = convertFromString<double>(parts[0]);
    output.pose.position.y = convertFromString<double>(parts[1]);
    output.pose.position.z = convertFromString<double>(parts[2]);

    output.pose.orientation.x = convertFromString<double>(parts[3]);
    output.pose.orientation.y = convertFromString<double>(parts[4]);
    output.pose.orientation.z = convertFromString<double>(parts[5]);
    output.pose.orientation.w = convertFromString<double>(parts[6]);

    output.frame_id = parts[7];

    return output;
}
}  // namespace BT
