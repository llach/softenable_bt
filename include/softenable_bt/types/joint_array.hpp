#pragma once

#include <array>
#include <behaviortree_cpp/bt_factory.h>  // Needed for BT::StringView and splitString

using JointArray = std::array<double, 6>;

namespace BT
{

template <> inline JointArray convertFromString(BT::StringView str)
{
    auto parts = BT::splitString(str, ';');
    if (parts.size() != 6)
    {
        throw BT::RuntimeError("Invalid JointArray input. Expected 6 elements, got ",
                               std::to_string(parts.size()));
    }

    JointArray output;
    for (size_t i = 0; i < 6; ++i)
    {
        output[i] = BT::convertFromString<double>(parts[i]);
    }
    return output;
}

} // namespace BT