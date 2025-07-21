#include "softenable_bt/manipulation/move_eef.hpp"
#include <iostream>


#include "stack_msgs/srv/move_arm.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"

MoveEEF::MoveEEF(const std::string& name, const BT::NodeConfiguration& config)
    : BT::SyncActionNode(name, config)
{
    config.blackboard->get("ros_node", node_);
    config.blackboard->get("tf_wrapper", tf_wrapper_);
    config.blackboard->get("move_arm_client", client_);
}

BT::NodeStatus MoveEEF::tick()
{
    std::string controller_name;
    if (!getInput<std::string>("controller_name", controller_name))
    {
        // input port not provided, fallback to blackboard directly
        auto blackboard = config().blackboard;
        if (!blackboard)
        {
            throw BT::RuntimeError("Blackboard is nullptr, cannot fallback");
        }

        if (!blackboard->get("default_controller_name", controller_name))
        {
            throw BT::RuntimeError("Missing both input port 'controller_name' and blackboard key 'default_controller_name'");
        }
    }
    std::cout << "Using controller " << controller_name << std::endl;

    geometry_msgs::msg::PoseStamped pose_msg;
    try {
        pose_msg = tf_wrapper_->lookupTransform("map", "right_arm_wrist_3_link", rclcpp::Duration::from_seconds(2.0));
        RCLCPP_INFO(tf_wrapper_->getNode()->get_logger(),
                    "Starting pose: [%.2f, %.2f, %.2f]",
                    pose_msg.pose.position.x,
                    pose_msg.pose.position.y,
                    pose_msg.pose.position.z);
    } catch (const tf2::TransformException& ex) {
        RCLCPP_ERROR(tf_wrapper_->getNode()->get_logger(), "TF lookup failed: %s", ex.what());
        return BT::NodeStatus::FAILURE;
    }

    double time = getInput<double>("time").value_or(3.0); 
    double x = getInput<double>("x").value_or(0.0);
    double y = getInput<double>("y").value_or(0.0);
    double z = getInput<double>("z").value_or(0.0);

    pose_msg.pose.position.x += x;
    pose_msg.pose.position.y += y;
    pose_msg.pose.position.z += z;

    RCLCPP_INFO(tf_wrapper_->getNode()->get_logger(),
        "Goal pose: [%.2f, %.2f, %.2f]",
        pose_msg.pose.position.x,
        pose_msg.pose.position.y,
        pose_msg.pose.position.z
    );

    auto request = std::make_shared<stack_msgs::srv::MoveArm::Request>();
    request->execute = true;
    request->execution_time = time;
    request->target_pose = pose_msg;
    request->controller_name = controller_name;    
    request->name_target = { "right_arm_shoulder_pan_joint", "right_arm_shoulder_lift_joint", "right_arm_elbow_joint", "right_arm_wrist_1_joint", "right_arm_wrist_2_joint", "right_arm_wrist_3_joint" };
    request->ik_link = "right_arm_wrist_3_link";

    // Call service synchronously
    auto future = client_->async_send_request(request);
    auto result = rclcpp::spin_until_future_complete(node_, future, std::chrono::seconds(60));

    if (result == rclcpp::FutureReturnCode::SUCCESS && future.get()->success) {
        std::cout << "MoveEEF succeeded!\n";
        return BT::NodeStatus::SUCCESS;
    } else {
        std::cerr << "MoveEEF failed to move arm\n";
        return BT::NodeStatus::FAILURE;
    }
}

BT::PortsList MoveEEF::providedPorts()
{
    return {
        BT::InputPort<std::string>("controller_name"),
        BT::InputPort<double>("time", "Execution time in seconds"),
        BT::InputPort<double>("x"),
        BT::InputPort<double>("y"),
        BT::InputPort<double>("z")
    };
}
