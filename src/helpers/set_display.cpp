#include "softenable_bt/helpers/set_display.hpp"

#include <chrono>

using namespace std::chrono_literals;


SetDisplaySkill::SetDisplaySkill(const std::string& instance_name,
                                 const BT::NodeConfig& config)
: BT::SyncActionNode(instance_name, config)
{
  config.blackboard->get("ros_node", node_);
  config.blackboard->get("set_display_client", client_);
}

BT::PortsList SetDisplaySkill::providedPorts()
{
  return {
    BT::InputPort<std::string>("preset", "Preset name to apply"),
    BT::InputPort<bool>("use_tts", true, "Whether to use TTS when applying the preset"),
    BT::InputPort<bool>("blocking", false, "Whether TTS calls are blocking"),
  };
}


BT::NodeStatus SetDisplaySkill::tick()
{
  // Read ports
  auto name_in = getInput<std::string>("preset");
  if (!name_in)
  {
    // Missing required input
    if (node_) { RCLCPP_ERROR(node_->get_logger(), "SetDisplaySkill: missing input port 'preset': %s", name_in.error().c_str()); }
    return BT::NodeStatus::FAILURE;
  }
  std::string preset_name = *name_in;

  // Read 'use_tts' input (default: true)
  bool use_tts = true;
  auto use_tts_in = getInput<bool>("use_tts");
  if (use_tts_in)
  {
    use_tts = *use_tts_in;
  }

  bool blocking = false;
  auto blocking_in = getInput<bool>("blocking");
  if (blocking_in)
  {
    blocking = *blocking_in;
  }

  // Build request
  auto req = std::make_shared<softenable_display_msgs::srv::SetDisplay::Request>();
  req->name = preset_name;
  req->use_tts = use_tts;
  req->blocking = blocking;

  // Use a single-threaded executor to wait for the response without spinning the global ROS
  rclcpp::executors::SingleThreadedExecutor exec;
  exec.add_node(node_);

  auto future = client_->async_send_request(req);

  // Wait a short time for the response
  auto ret = exec.spin_until_future_complete(future, 20000s);

  if (ret != rclcpp::FutureReturnCode::SUCCESS)
  {
    RCLCPP_ERROR(node_->get_logger(), "Service call timed out or failed");
    return BT::NodeStatus::FAILURE;
  }

  const auto response = future.get();
  if (!response)
  {
    RCLCPP_ERROR(node_->get_logger(), "SetDisplaySkill: null response from service");
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
