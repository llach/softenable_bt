#include "softenable_bt/perception/sam2_segmentation.hpp"
#include <iostream>

SAM2Segmentation::SAM2Segmentation(const std::string& name, const BT::NodeConfiguration& config)
  : BT::SyncActionNode(name, config) {}

BT::NodeStatus SAM2Segmentation::tick()
{
  auto all_keys = config().blackboard->getKeys();
  
  std::cout << "[SAM2Segmentation] Blackboard keys:\n";
  for (const auto& key : all_keys) {
      std::cout << "  - " << key << std::endl;
  }

  return BT::NodeStatus::SUCCESS;
}

BT::PortsList SAM2Segmentation::providedPorts()
{
  return { };
}
