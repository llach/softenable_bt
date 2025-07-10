#include "softenable_bt/perception/stack_detection.hpp"
#include <iostream>

StackDetection::StackDetection(const std::string& name, const BT::NodeConfiguration& config)
    : BT::SyncActionNode(name, config)
{
    config.blackboard->get("ros_node", node_);
    config.blackboard->get("stack_detect_client", client_);
}

BT::NodeStatus StackDetection::tick()
{
    auto request = std::make_shared<stack_msgs::srv::StackDetect::Request>();
    request->store_data = true;

    std::cout << "Calling stack_detect ...\n";
    
    auto future = client_->async_send_request(request);
    auto result = rclcpp::spin_until_future_complete(node_, future, std::chrono::seconds(60));

    auto response = future.get();
    if (result == rclcpp::FutureReturnCode::SUCCESS && response->success) {
        std::cout << "StackDetection succeeded!\n";
    } else {
        std::cerr << "StackDetection failed ....\n";
        return BT::NodeStatus::FAILURE;
    }

    auto pose = response->target_pose;
    RCLCPP_INFO(node_->get_logger(),
        "Stack position: [%.2f, %.2f, %.2f]",
        pose.pose.position.x,
        pose.pose.position.y,
        pose.pose.position.z
    );

    setOutput("stack_pose", pose);

    return BT::NodeStatus::SUCCESS;
}

BT::PortsList StackDetection::providedPorts()
{
    return { BT::OutputPort<geometry_msgs::msg::PoseStamped>("stack_pose") };
}
