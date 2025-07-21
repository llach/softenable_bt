#include <iostream>
#include "softenable_bt/manipulation/move_cartesian.hpp"

#include "stack_msgs/srv/move_arm.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"

MoveCartesian::MoveCartesian(const std::string& name, const BT::NodeConfiguration& config)
  : BT::SyncActionNode(name, config)
{
  config.blackboard->get("ros_node", node_);
  config.blackboard->get("move_arm_client", client_);
}

BT::NodeStatus MoveCartesian::tick()
{
  // auto all_keys = config().blackboard->getKeys();
  // std::cout << "[MoveCartesian] Blackboard keys:\n";
  // for (const auto& key : all_keys) {
  //   std::cout << "  - " << key << std::endl;
  // }

  std::string controller_name;
  if (!getInput<std::string>("controller_name", controller_name))
  {
    // input port not provided, fallback to blackboard directly
    auto blackboard = config().blackboard;
    if (!blackboard)
    {
      throw BT::RuntimeError("Blackboard is nullptr, cannot fallback");
    }

    if (!blackboard->get("default_controller_name", controller_name))
    {
      throw BT::RuntimeError("Missing both input port 'controller_name' and blackboard key 'default_controller_name'");
    }
  }

  auto target = getInput<geometry_msgs::msg::PoseStamped>("target_pose");
  if (!target) {
    throw BT::RuntimeError("MoveCartesian: missing input [target_pose] — ", target.error());
  }

  auto ik_frame = getInput<std::string>("ik_frame");
  if (!ik_frame) {
    throw BT::RuntimeError("MoveCartesian: missing input [ik_frame] — ", ik_frame.error());
  }

  double time = getInput<double>("time").value_or(3.0); 

  // Build request
  auto request = std::make_shared<stack_msgs::srv::MoveArm::Request>();
  request->execute = true;
  request->controller_name = controller_name;
  request->execution_time = time;
  request->ik_link = ik_frame.value();
  request->target_pose = target.value();

  // Call service
  auto future = client_->async_send_request(request);
  auto result = rclcpp::spin_until_future_complete(node_, future, std::chrono::seconds(500));

  if (result == rclcpp::FutureReturnCode::SUCCESS && future.get()->success) {
    return BT::NodeStatus::SUCCESS;
  } else {
    std::cerr << "MoveCartesian: service call failed or returned failure\n";
    return BT::NodeStatus::FAILURE;
  }
}

BT::PortsList MoveCartesian::providedPorts()
{
  return {
    BT::InputPort<std::string>("controller_name"),
    BT::InputPort<double>("time", "Execution time in seconds"),
    BT::InputPort<geometry_msgs::msg::PoseStamped>("target_pose"),
    BT::InputPort<std::string>("ik_frame")
  };
}
