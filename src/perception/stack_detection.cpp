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
    if (!future.valid())
    {
        std::cerr << "StackDetection: async_send_request did not return a valid future\n";
        return BT::NodeStatus::FAILURE;
    }

    std::cout << "Waiting for stack_detect response ...\n";
    auto result = rclcpp::spin_until_future_complete(node_, future, std::chrono::seconds(500));
    auto response = future.get();

    if (!response->success) {
        std::cerr << "StackDetection: service returned unsuccessful\n";
        return BT::NodeStatus::FAILURE;
    }

    setOutput("stack_pose", response->target_pose);
    std::cout << "StackDetection: stored stack_pose from service response\n";
    return BT::NodeStatus::SUCCESS;
}

BT::PortsList StackDetection::providedPorts()
{
    return { BT::OutputPort<geometry_msgs::msg::PoseStamped>("stack_pose") };
}
