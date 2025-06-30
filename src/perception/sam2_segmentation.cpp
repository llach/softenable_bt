#include "softenable_bt/perception/sam2_segmentation.hpp"
#include <iostream>

SAM2Segmentation::SAM2Segmentation(const std::string& name, const BT::NodeConfiguration& config)
    : BT::SyncActionNode(name, config) {}

BT::NodeStatus SAM2Segmentation::tick()
{
    std::cout << "Ticking SAM2 Segmentation\n";
    return BT::NodeStatus::SUCCESS;
}

BT::PortsList SAM2Segmentation::providedPorts() { return {}; }
