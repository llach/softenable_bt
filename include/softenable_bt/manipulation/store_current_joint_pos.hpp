#pragma once

#include <behaviortree_cpp/action_node.h>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <array>

#include "softenable_bt/types/joint_array.hpp"


class StoreCurrentJointPos : public BT::SyncActionNode
{
public:
    StoreCurrentJointPos(const std::string& name, const BT::NodeConfiguration& config);

    static BT::PortsList providedPorts();
    BT::NodeStatus tick() override;

private:
    void jointCallback(const sensor_msgs::msg::JointState::SharedPtr msg);

    rclcpp::Node::SharedPtr node_;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_sub_;
    JointArray latest_joints_;
    rclcpp::Time start_time_;
    bool has_received_ = false;
    std::vector<std::string> expected_joint_names_ = {
        "right_arm_shoulder_pan_joint",
        "right_arm_shoulder_lift_joint",
        "right_arm_elbow_joint",
        "right_arm_wrist_1_joint",
        "right_arm_wrist_2_joint",
        "right_arm_wrist_3_joint"
    };
    
};
