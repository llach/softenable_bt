#include "softenable_bt/manipulation/roller_gripper.hpp"

RollerGripper::RollerGripper(const std::string& name,
                             const BT::NodeConfiguration& config)
    : BT::SyncActionNode(name, config)
{
    config.blackboard->get("ros_node", node_);
    config.blackboard->get("roller_gripper_client", client_);
}

BT::PortsList RollerGripper::providedPorts()
{
    return {
        BT::InputPort<std::string>("action"),
        BT::InputPort<int>("roller_vel"),
        BT::InputPort<int>("roller_duration")
    };
}

BT::NodeStatus RollerGripper::tick()
{
    std::string action;
    if (!getInput("action", action)) {
        RCLCPP_ERROR(node_->get_logger(), "Missing required input [action]");
        return BT::NodeStatus::FAILURE;
    }

    auto request = std::make_shared<stack_msgs::srv::RollerGripper::Request>();

    if (action == "open") {
        request->finger_pos = 2500;
    } else if (action == "close") {
        request->finger_pos = 800;
    } else if (action == "roll") {
        int vel, duration;
        if (!getInput("roller_vel", vel) || !getInput("roller_duration", duration)) {
            RCLCPP_ERROR(node_->get_logger(), "Missing roller_vel or roller_duration for 'roll' action");
            return BT::NodeStatus::FAILURE;
        }
        request->roller_vel = vel;
        request->roller_duration = duration;
    } else {
        RCLCPP_ERROR(node_->get_logger(), "Invalid action: %s", action.c_str());
        return BT::NodeStatus::FAILURE;
    }

    auto future = client_->async_send_request(request);
    auto result = rclcpp::spin_until_future_complete(node_, future, std::chrono::seconds(5));

    if (result != rclcpp::FutureReturnCode::SUCCESS || !future.get()->success) {
        RCLCPP_ERROR(node_->get_logger(), "RollerGripper action failed.");
        return BT::NodeStatus::FAILURE;
    }

    return BT::NodeStatus::SUCCESS;
}
