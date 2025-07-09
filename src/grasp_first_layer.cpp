#include <behaviortree_cpp/bt_factory.h>
#include "rclcpp/rclcpp.hpp"

#include <filesystem>
#include "ament_index_cpp/get_package_share_directory.hpp"

#include "softenable_bt/perception/stack_detection.hpp"
#include "softenable_bt/perception/sam2_segmentation.hpp"
#include "softenable_bt/perception/dino_detection.hpp"
#include "softenable_bt/manipulation/move_eef.hpp"
#include "softenable_bt/manipulation/store_current_joint_pos.hpp"
#include "softenable_bt/manipulation/move_joint.hpp"
#include "softenable_bt/manipulation/move_cartesian.hpp"

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto ros_node = rclcpp::Node::make_shared("bt_move_arm_node");

    auto move_arm_client = ros_node->create_client<stack_msgs::srv::MoveArm>("/move_arm");
    auto stack_detect_client = ros_node->create_client<stack_msgs::srv::StackDetect>("/stack_detect");

    // Wait for service (optional)
    while (!move_arm_client->wait_for_service(std::chrono::seconds(1))) {
        RCLCPP_INFO(ros_node->get_logger(), "Waiting for /move_arm service...");
    }

    BT::BehaviorTreeFactory factory;

    factory.registerNodeType<StoreCurrentJointPos>("StoreCurrentJointPos");
    factory.registerNodeType<SAM2Segmentation>("SAM2Segmentation");
    factory.registerNodeType<StackDetection>("StackDetection");
    factory.registerNodeType<DINODetection>("DINODetection");
    factory.registerNodeType<MoveCartesian>("MoveCartesian");
    factory.registerNodeType<MoveJoint>("MoveJoint");
    factory.registerNodeType<MoveEEF>("MoveEEF");

    std::string package_path = ament_index_cpp::get_package_share_directory("softenable_bt");
    std::string tree_path = package_path + "/behavior_trees/test_nodes.xml";

    auto blackboard = BT::Blackboard::create();
    blackboard->set("ros_node", ros_node);
    blackboard->set("move_arm_client", move_arm_client);
    blackboard->set("stack_detect_client", stack_detect_client);

    auto tree = factory.createTreeFromFile(tree_path, blackboard);

    tree.tickWhileRunning();

    return 0;
}
