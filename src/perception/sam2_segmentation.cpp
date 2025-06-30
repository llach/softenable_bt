#include "softenable_bt/perception/sam2_segmentation.hpp"
#include <iostream>

SAM2Segmentation::SAM2Segmentation(const std::string& name, const BT::NodeConfiguration& config)
  : BT::SyncActionNode(name, config) {}

BT::NodeStatus SAM2Segmentation::tick()
{
  CartesianTarget target;
  target.pose.position.x = 0.1;
  target.pose.position.y = 0.0;
  target.pose.position.z = 0.3;
  target.pose.orientation.x = 0.0;
  target.pose.orientation.y = 0.0;
  target.pose.orientation.z = 0.0;
  target.pose.orientation.w = 1.0;
  target.frame_id = "camera_link";

  setOutput("segmentation_pose", target);

  std::cout << "SAM2Segmentation: produced CartesianTarget in frame " << target.frame_id << "\n";
  return BT::NodeStatus::SUCCESS;
}

BT::PortsList SAM2Segmentation::providedPorts()
{
  return { BT::OutputPort<CartesianTarget>("segmentation_pose") };
}
