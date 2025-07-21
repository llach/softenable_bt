#include "softenable_bt/manipulation/move_joint.hpp"
#include "softenable_bt/types/joint_array.hpp"
#include "rclcpp/executors.hpp"
#include <iostream>

MoveJoint::MoveJoint(const std::string& name, const BT::NodeConfiguration& config)
    : BT::SyncActionNode(name, config) {
        config.blackboard->get("move_arm_client", client_);
        config.blackboard->get("ros_node", node_);
    }

BT::NodeStatus MoveJoint::tick()
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
    
    auto joints_opt = getInput<JointArray>("joint_values");
    if (!joints_opt)
        throw BT::RuntimeError("Missing input port [joint_values]");

    const auto& joints = joints_opt.value();

    std::cout << "Moving to joints: ";
    for (auto j : joints) std::cout << j << " ";
    std::cout << "\n";

    double time = getInput<double>("time").value_or(3.0); 

    auto request = std::make_shared<stack_msgs::srv::MoveArm::Request>();

    request->execute = true;
    request->execution_time = time;
    request->controller_name = controller_name;
    request->name_target = { "right_arm_shoulder_pan_joint", "right_arm_shoulder_lift_joint", "right_arm_elbow_joint", "right_arm_wrist_1_joint", "right_arm_wrist_2_joint", "right_arm_wrist_3_joint" };
    request->q_target.reserve(6);

    for (double j : joints) {
        request->q_target.push_back(static_cast<float>(j));
    }


    auto future = client_->async_send_request(request);
    auto result = rclcpp::spin_until_future_complete(node_, future, std::chrono::seconds(60));

    if (result == rclcpp::FutureReturnCode::SUCCESS && future.get()->success)
    {
        std::cout << "MoveJoint: /move_arm call succeeded\n";
        return BT::NodeStatus::SUCCESS;
    }
    else
    {
        std::cerr << "MoveJoint: /move_arm call failed or timed out\n";
        return BT::NodeStatus::FAILURE;
    }

    return BT::NodeStatus::SUCCESS;
}

BT::PortsList MoveJoint::providedPorts()
{
    return {
        BT::InputPort<double>("time", "Execution time in seconds"),
        BT::InputPort<std::string>("controller_name"),
        BT::InputPort<JointArray>("joint_values"),
    };
}