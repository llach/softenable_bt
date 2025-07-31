#include <behaviortree_cpp/bt_factory.h>
#include "rclcpp/rclcpp.hpp"

#include <filesystem>
#include "ament_index_cpp/get_package_share_directory.hpp"

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
    auto roller_gripper_client = ros_node->create_client<stack_msgs::srv::RollerGripper>("/roller_gripper");


    while (!move_arm_client->wait_for_service(std::chrono::seconds(5))) {
        RCLCPP_INFO(ros_node->get_logger(), "Waiting for /move_arm service...");
    }

    // while (!stack_detect_client->wait_for_service(std::chrono::seconds(5))) {
    //     RCLCPP_INFO(ros_node->get_logger(), "Waiting for /stack_detect service...");
    // }

    // while (!roller_gripper_client->wait_for_service(std::chrono::seconds(5))) {
    //     RCLCPP_INFO(ros_node->get_logger(), "Waiting for /roller_gripper service...");
    // }


    std::cout << "services are connected!\n";

    BT::BehaviorTreeFactory factory;

    factory.registerNodeType<StoreCurrentJointPos>("StoreCurrentJointPos");
    factory.registerNodeType<SAM2Segmentation>("SAM2Segmentation");
    factory.registerNodeType<StackDetection>("StackDetection");
    factory.registerNodeType<DINODetection>("DINODetection");
    factory.registerNodeType<MoveCartesian>("MoveCartesian");
    factory.registerNodeType<RollerGripper>("RollerGripper");
    factory.registerNodeType<MoveJoint>("MoveJoint");
    factory.registerNodeType<MoveEEF>("MoveEEF");

    std::string package_path = ament_index_cpp::get_package_share_directory("softenable_bt");
    std::string tree_path = package_path + "/behavior_trees/grasp_first_layer.xml";
    // std::string tree_path = package_path + "/behavior_trees/test_nodes.xml";

    auto blackboard = BT::Blackboard::create();
    blackboard->set("ros_node", ros_node);
    blackboard->set("tf_wrapper", tf_wrapper);
    blackboard->set("move_arm_client", move_arm_client);
    blackboard->set("stack_detect_client", stack_detect_client);
    blackboard->set("roller_gripper_client", roller_gripper_client);

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
