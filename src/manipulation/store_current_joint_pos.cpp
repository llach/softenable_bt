#include "softenable_bt/manipulation/store_current_joint_pos.hpp"
#include <iostream>

StoreCurrentJointPos::StoreCurrentJointPos(const std::string& name, const BT::NodeConfiguration& config)
    : BT::SyncActionNode(name, config) {}

BT::NodeStatus StoreCurrentJointPos::tick()
{
    JointArray current_joints = {1.0, 1.1, 1.2, 1.3, 1.4, 1.5}; // Simulated joint values

    setOutput("joint_out", current_joints);

    std::cout << "Stored joints: ";
    for (auto j : current_joints) std::cout << j << " ";
    std::cout << "\n";

    return BT::NodeStatus::SUCCESS;
}

BT::PortsList StoreCurrentJointPos::providedPorts()
{
    return { BT::OutputPort<JointArray>("joint_out") };
}
