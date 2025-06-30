#include "softenable_bt/manipulation/move_joint.hpp"
#include <iostream>

MoveJoint::MoveJoint(const std::string& name, const BT::NodeConfiguration& config)
    : BT::SyncActionNode(name, config) {}

BT::NodeStatus MoveJoint::tick()
{
    std::array<double, 6> joints;
    for (int i = 0; i < 6; ++i) {
        auto val = getInput<double>("j" + std::to_string(i + 1));
        if (!val) {
            throw BT::RuntimeError("Missing joint value for j" + std::to_string(i + 1));
        }
        joints[i] = val.value();
    }

    std::cout << "Moving to joints: ";
    for (auto j : joints) std::cout << j << " ";
    std::cout << "\n";

    return BT::NodeStatus::SUCCESS;
}

BT::PortsList MoveJoint::providedPorts()
{
    return {
        BT::InputPort<double>("j1"),
        BT::InputPort<double>("j2"),
        BT::InputPort<double>("j3"),
        BT::InputPort<double>("j4"),
        BT::InputPort<double>("j5"),
        BT::InputPort<double>("j6")
    };
}
