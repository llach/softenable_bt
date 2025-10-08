#include <behaviortree_cpp/bt_factory.h>
#include "rclcpp/rclcpp.hpp"

#include <filesystem>
#include "ament_index_cpp/get_package_share_directory.hpp"

#include "softenable_bt/helpers/wait.hpp"
#include "softenable_bt/helpers/set_display.hpp"
#include "softenable_bt/helpers/trigger_service.hpp"
#include "softenable_bt/helpers/tf_listener_wrapper.hpp"
#include "softenable_bt/perception/stack_detection.hpp"
#include "softenable_bt/perception/sam2_segmentation.hpp"
#include "softenable_bt/perception/dino_detection.hpp"
#include "softenable_bt/manipulation/move_eef.hpp"
#include "softenable_bt/manipulation/roller_gripper.hpp"
#include "softenable_bt/manipulation/store_current_joint_pos.hpp"
#include "softenable_bt/manipulation/move_joint.hpp"
#include "softenable_bt/manipulation/move_cartesian.hpp"

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto ros_node = rclcpp::Node::make_shared("bt_move_arm_node");

    auto tf_wrapper = std::make_shared<softenable_bt::TFListenerWrapper>(ros_node);
    auto move_arm_client = ros_node->create_client<stack_msgs::srv::MoveArm>("/move_arm");
    auto stack_detect_client = ros_node->create_client<stack_msgs::srv::StackDetect>("/stack_detect");
    auto set_display_client = ros_node->create_client<softenable_display_msgs::srv::SetDisplay>("/set_display");
    auto left_roller_gripper_client = ros_node->create_client<stack_msgs::srv::RollerGripper>("/left_roller_gripper");
    auto right_roller_gripper_client = ros_node->create_client<stack_msgs::srv::RollerGripper>("/right_roller_gripper");

    while (!move_arm_client->wait_for_service(std::chrono::seconds(5))) {
        RCLCPP_INFO(ros_node->get_logger(), "Waiting for /move_arm service...");
    }

    while (!stack_detect_client->wait_for_service(std::chrono::seconds(5))) {
        RCLCPP_INFO(ros_node->get_logger(), "Waiting for /stack_detect service...");
    }

    while (!left_roller_gripper_client->wait_for_service(std::chrono::seconds(5))) {
        RCLCPP_INFO(ros_node->get_logger(), "Waiting for /left_roller_gripper service...");
    }

    while (!right_roller_gripper_client->wait_for_service(std::chrono::seconds(5))) {
        RCLCPP_INFO(ros_node->get_logger(), "Waiting for /right_roller_gripper service...");
    }

    while (!set_display_client->wait_for_service(std::chrono::seconds(5))) {
        RCLCPP_INFO(ros_node->get_logger(), "Waiting for /set_display service...");
    }


    std::cout << "services are connected!\n";

    BT::BehaviorTreeFactory factory;

    factory.registerNodeType<StoreCurrentJointPos>("StoreCurrentJointPos");
    factory.registerNodeType<SAM2Segmentation>("SAM2Segmentation");
    factory.registerNodeType<SetDisplaySkill>("SetDisplaySkill");
    factory.registerNodeType<TriggerService>("TriggerService");
    factory.registerNodeType<StackDetection>("StackDetection");
    factory.registerNodeType<DINODetection>("DINODetection");
    factory.registerNodeType<MoveCartesian>("MoveCartesian");
    factory.registerNodeType<RollerGripper>("RollerGripper");
    factory.registerNodeType<MoveJoint>("MoveJoint");
    factory.registerNodeType<MoveEEF>("MoveEEF");
    factory.registerNodeType<Wait>("Wait");

    std::string package_path = ament_index_cpp::get_package_share_directory("softenable_bt");
    std::string tree_path = package_path + "/behavior_trees/grasp_first_layer.xml";

    auto blackboard = BT::Blackboard::create();
    blackboard->set("ros_node", ros_node);
    blackboard->set("tf_wrapper", tf_wrapper);
    blackboard->set("move_arm_client", move_arm_client);
    blackboard->set("set_display_client", set_display_client);
    blackboard->set("stack_detect_client", stack_detect_client);
    blackboard->set("left_roller_gripper_client", left_roller_gripper_client);
    blackboard->set("right_roller_gripper_client", right_roller_gripper_client);

    try {
        auto pose = tf_wrapper->lookupTransform("map", "right_arm_wrist_3_link", rclcpp::Duration::from_seconds(2.0));
        RCLCPP_INFO(tf_wrapper->getNode()->get_logger(),
                    "Got pose: [%.2f, %.2f, %.2f]",
                    pose.pose.position.x,
                    pose.pose.position.y,
                    pose.pose.position.z);
    } catch (const tf2::TransformException& ex) {
        RCLCPP_ERROR(tf_wrapper->getNode()->get_logger(), "TF lookup failed: %s", ex.what());
        return -1;
    }

    auto tree = factory.createTreeFromFile(tree_path, blackboard);
    tree.tickWhileRunning();

    return 0;
}
