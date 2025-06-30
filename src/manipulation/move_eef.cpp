#include "softenable_bt/manipulation/move_eef.hpp"
#include <iostream>

MoveEEF::MoveEEF(const std::string& name, const BT::NodeConfiguration& config)
    : BT::SyncActionNode(name, config) {}

BT::NodeStatus MoveEEF::tick()
{
    double x = getInput<double>("x").value_or(0.0);
    double y = getInput<double>("y").value_or(0.0);
    double z = getInput<double>("z").value_or(0.0);

    std::cout << "Moving to EEF pose: x=" << x << ", y=" << y << ", z=" << z << "\n";
    return BT::NodeStatus::SUCCESS;
}

BT::PortsList MoveEEF::providedPorts()
{
    return {
        BT::InputPort<double>("x"),
        BT::InputPort<double>("y"),
        BT::InputPort<double>("z")
    };
}
