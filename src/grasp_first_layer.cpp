#include <behaviortree_cpp/bt_factory.h>
#include "softenable_bt/perception/sam2_segmentation.hpp"
#include "softenable_bt/perception/dino_detection.hpp"
#include "softenable_bt/manipulation/move_eef.hpp"
#include "softenable_bt/manipulation/store_current_joint_pos.hpp"
#include "softenable_bt/manipulation/move_joint.hpp"

int main(int argc, char** argv)
{
    BT::BehaviorTreeFactory factory;

    factory.registerNodeType<StoreCurrentJointPos>("StoreCurrentJointPos");
    factory.registerNodeType<SAM2Segmentation>("SAM2Segmentation");
    factory.registerNodeType<DINODetection>("DINODetection");
    factory.registerNodeType<MoveJoint>("MoveJoint");
    factory.registerNodeType<MoveEEF>("MoveEEF");

    auto tree = factory.createTreeFromFile("behavior_trees/grasp_first_layer.xml");

    tree.tickWhileRunning();

    return 0;
}
