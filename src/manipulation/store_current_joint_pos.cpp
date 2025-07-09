#include "softenable_bt/manipulation/store_current_joint_pos.hpp"

StoreCurrentJointPos::StoreCurrentJointPos(const std::string& name, const BT::NodeConfiguration& config)
    : BT::SyncActionNode(name, config)
{
    config.blackboard->get("ros_node", node_);
    start_time_ = node_->now();

    joint_sub_ = node_->create_subscription<sensor_msgs::msg::JointState>(
        "/joint_states", 10,
        std::bind(&StoreCurrentJointPos::jointCallback, this, std::placeholders::_1));
}

void StoreCurrentJointPos::jointCallback(const sensor_msgs::msg::JointState::SharedPtr msg)
{
    if (!has_received_)
    {
        RCLCPP_INFO(node_->get_logger(), "Received first /joint_states message");
        has_received_ = true;
    }

    // Assume the robot has at least 6 joints
    if (msg->position.size() >= 6)
    {
        for (size_t i = 0; i < 6; ++i)
        {
            latest_joints_[i] = msg->position[i];
        }
    }
}

BT::NodeStatus StoreCurrentJointPos::tick()
{
    // Wait up to 3 seconds for the first joint state
    rclcpp::Time start_time = node_->now();
    rclcpp::Duration timeout = rclcpp::Duration::from_seconds(3.0);

    while (!has_received_ && (node_->now() - start_time) < timeout)
    {
        rclcpp::spin_some(node_);
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }

    if (!has_received_)
    {
        RCLCPP_ERROR(node_->get_logger(), "Timeout: no joint states received");
        return BT::NodeStatus::FAILURE;
    }


    setOutput("joint_out", latest_joints_);

    std::cout << "Stored joints: ";
    for (auto j : latest_joints_) std::cout << j << " ";
    std::cout << "\n";

    return BT::NodeStatus::SUCCESS;
}

BT::PortsList StoreCurrentJointPos::providedPorts()
{
    return { BT::OutputPort<JointArray>("joint_out") };
}
