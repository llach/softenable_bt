#include "softenable_bt/helpers/trigger_service.hpp"

TriggerService::TriggerService(const std::string& name, const BT::NodeConfig& config)
: BT::SyncActionNode(name, config)
{
  // Get ROS2 node from blackboard
  node_ = config.blackboard->get<rclcpp::Node::SharedPtr>("ros_node");

  // Get service name from input port
  service_name_ = getInput<std::string>("service_name").value();
  client_ = node_->create_client<std_srvs::srv::Trigger>(service_name_);

  // Wait until service is available
  if (!client_->wait_for_service(std::chrono::seconds(5))) {
    throw std::runtime_error("Service " + service_name_ + " not available");
  }
}

BT::PortsList TriggerService::providedPorts()
{
  return { BT::InputPort<std::string>("service_name") };
}

BT::NodeStatus TriggerService::tick()
{
  RCLCPP_INFO(node_->get_logger(), "Calling service '%s'", service_name_);
  
  auto request = std::make_shared<std_srvs::srv::Trigger::Request>();

  auto future = client_->async_send_request(request);
  auto result = rclcpp::spin_until_future_complete(node_, future, std::chrono::seconds(6000));

  auto response = future.get();
  if (response->success) {
    RCLCPP_INFO(node_->get_logger(), "Service call succeeded: %s", response->message.c_str());
    return BT::NodeStatus::SUCCESS;
  } else {
    RCLCPP_WARN(node_->get_logger(), "Service call failed: %s", response->message.c_str());
    return BT::NodeStatus::FAILURE;
  }
}

// Register node with the factory
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<TriggerService>("TriggerService");
}
