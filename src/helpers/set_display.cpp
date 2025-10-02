#include "softenable_display_bt/set_display.hpp"

#include <chrono>

using namespace std::chrono_literals;
using softenable_display_bt::SetDisplaySkill;

namespace softenable_display_bt
{

SetDisplaySkill::SetDisplaySkill(const std::string& instance_name,
                                 const BT::NodeConfig& config)
: BT::SyncActionNode(instance_name, config)
{
  // Lazily create rclcpp node on first tick (keeps constructor light)
}

BT::PortsList SetDisplaySkill::providedPorts()
{
  return {
    BT::InputPort<std::string>("name", "Preset name to apply"),
    BT::InputPort<std::string>("service", "/set_display", "Service name")
  };
}

void SetDisplaySkill::ensureClient(const std::string& service_name)
{
  if (!node_)
  {
    // Unique-ish node name per plugin instance
    std::string node_name = "set_display_bt_" + this->name();
    // sanitize name (BT instance names may contain invalid chars)
    for (auto& ch : node_name) { if (ch == ':' || ch == '.' || ch == '-') ch = '_'; }

    node_ = rclcpp::Node::make_shared(node_name);
  }

  if (!client_ || service_name != current_service_name_)
  {
    current_service_name_ = service_name;
    client_ = node_->create_client<softenable_display_msgs::srv::SetDisplay>(current_service_name_);
  }
}

BT::NodeStatus SetDisplaySkill::tick()
{
  // Read ports
  auto name_in = getInput<std::string>("name");
  if (!name_in)
  {
    // Missing required input
    if (node_) { RCLCPP_ERROR(node_->get_logger(), "SetDisplaySkill: missing input port 'name': %s", name_in.error().c_str()); }
    return BT::NodeStatus::FAILURE;
  }
  std::string preset_name = *name_in;

  std::string service_name = "/set_display";
  auto srv_in = getInput<std::string>("service");
  if (srv_in) { service_name = *srv_in; }

  // Ensure rclcpp client exists
  ensureClient(service_name);

  // Wait briefly for service (non-blocking long waits are bad in BT tick)
  if (!client_->wait_for_service(500ms))
  {
    RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 2000,
                         "SetDisplaySkill: service [%s] not available", service_name.c_str());
    return BT::NodeStatus::FAILURE;
  }

  // Build request
  auto req = std::make_shared<softenable_display_msgs::srv::SetDisplay::Request>();
  req->name = preset_name;

  // Use a single-threaded executor to wait for the response without spinning the global ROS
  rclcpp::executors::SingleThreadedExecutor exec;
  exec.add_node(node_);

  auto future = client_->async_send_request(req);

  // Wait a short time for the response
  auto ret = exec.spin_until_future_complete(future, 2s);

  if (ret != rclcpp::FutureReturnCode::SUCCESS)
  {
    RCLCPP_ERROR(node_->get_logger(), "SetDisplaySkill: call to [%s] timed out or failed",
                 service_name.c_str());
    return BT::NodeStatus::FAILURE;
  }

  const auto response = future.get();
  if (!response)
  {
    RCLCPP_ERROR(node_->get_logger(), "SetDisplaySkill: null response from [%s]", service_name.c_str());
    return BT::NodeStatus::FAILURE;
  }

  if (response->success)
  {
    RCLCPP_INFO_THROTTLE(node_->get_logger(), *node_->get_clock(), 3000,
                         "SetDisplaySkill: applied preset '%s'", preset_name.c_str());
    return BT::NodeStatus::SUCCESS;
  }
  else
  {
    RCLCPP_WARN(node_->get_logger(), "SetDisplaySkill: preset '%s' not applied (service returned false)",
                preset_name.c_str());
    return BT::NodeStatus::FAILURE;
  }
}

}  // namespace softenable_display_bt

// ---- BehaviorTree.CPP plugin registration ----
BT::NodeBuilder SetDisplaySkillBuilder =
  [](const std::string& name, const BT::NodeConfig& config)
{
  return std::make_unique<SetDisplaySkill>(name, config);
};

void BT_REGISTER_NODES(BT::BehaviorTreeFactory& factory)
{
  factory.registerBuilder<SetDisplaySkill>("SetDisplay", SetDisplaySkillBuilder);
}
