#include "softenable_bt/manipulation/move_eef.hpp"
#include <iostream>


#include "stack_msgs/srv/move_arm.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"

MoveEEF::MoveEEF(const std::string& name, const BT::NodeConfiguration& config)
    : BT::SyncActionNode(name, config)
{
    config.blackboard->get("ros_node", node_);
    config.blackboard->get("move_arm_client", client_);
}

BT::NodeStatus MoveEEF::tick()
{
    double x = getInput<double>("x").value_or(0.0);
    double y = getInput<double>("y").value_or(0.0);
    double z = getInput<double>("z").value_or(0.0);

    std::cout << "Moving to EEF pose: x=" << x << ", y=" << y << ", z=" << z << "\n";

    geometry_msgs::msg::PoseStamped pose_msg;
    pose_msg.header.frame_id = "wrist_3_link";
    pose_msg.header.stamp = node_->now();
    pose_msg.pose.position.x = x;
    pose_msg.pose.position.y = y;
    pose_msg.pose.position.z = z;
    pose_msg.pose.orientation.w = 1.0;  

    auto request = std::make_shared<stack_msgs::srv::MoveArm::Request>();
    request->execute = true;
    request->execution_time = 3.0;
    request->target_pose = pose_msg;

    // Call service synchronously
    auto future = client_->async_send_request(request);
    auto result = rclcpp::spin_until_future_complete(node_, future, std::chrono::seconds(60));

    if (result == rclcpp::FutureReturnCode::SUCCESS && future.get()->success) {
        return BT::NodeStatus::SUCCESS;
    } else {
        std::cerr << "MoveEEF failed to move arm\n";
        return BT::NodeStatus::FAILURE;
    }
}

BT::PortsList MoveEEF::providedPorts()
{
    return {
        BT::InputPort<double>("x"),
        BT::InputPort<double>("y"),
        BT::InputPort<double>("z")
    };
}
