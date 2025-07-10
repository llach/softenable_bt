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
    std::cout << "here\n";
    std::string action;
    if (!getInput("action", action)) {
        RCLCPP_ERROR(node_->get_logger(), "Missing required input [action]");
        return BT::NodeStatus::FAILURE;
    }
    std::cout << "here\n";

    auto request = std::make_shared<stack_msgs::srv::RollerGripper::Request>();
    std::cout << "here\n";

    if (action == "open") {
        request->finger_pos = 0;
    } else if (action == "close") {
        request->finger_pos = 1;
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
    std::cout << "here1\n";

    auto future = client_->async_send_request(request);
    auto result = rclcpp::spin_until_future_complete(node_, future, std::chrono::seconds(5));
    std::cout << "here2\n";

    if (result != rclcpp::FutureReturnCode::SUCCESS || !future.get()->success) {
        RCLCPP_ERROR(node_->get_logger(), "RollerGripper action failed.");
        return BT::NodeStatus::FAILURE;
    }

    return BT::NodeStatus::SUCCESS;
}
