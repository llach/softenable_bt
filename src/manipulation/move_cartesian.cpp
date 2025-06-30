#include "softenable_bt/manipulation/move_cartesian.hpp"
#include <iostream>

MoveCartesian::MoveCartesian(const std::string& name, const BT::NodeConfiguration& config)
  : BT::SyncActionNode(name, config) {}

BT::NodeStatus MoveCartesian::tick()
{
  auto target = getInput<CartesianTarget>("target_pose");
  auto ik_frame = getInput<std::string>("ik_frame");

  if (!target || !ik_frame) {
    throw BT::RuntimeError("MoveCartesian: missing input(s)");
  }

  const auto& pose = target->pose;

  std::cout << "MoveCartesian:\n"
            << "  To frame: " << target->frame_id << "\n"
            << "  Using IK frame: " << ik_frame.value() << "\n"
            << "  Pose: [" << pose.position.x << ", "
            << pose.position.y << ", "
            << pose.position.z << "]\n";

  return BT::NodeStatus::SUCCESS;
}

BT::PortsList MoveCartesian::providedPorts()
{
  return {
    BT::InputPort<CartesianTarget>("target_pose"),
    BT::InputPort<std::string>("ik_frame")
  };
}
