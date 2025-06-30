#include "softenable_bt/perception/dino_detection.hpp"
#include <iostream>

DINODetection::DINODetection(const std::string& name, const BT::NodeConfiguration& config)
    : BT::SyncActionNode(name, config) {}

BT::NodeStatus DINODetection::tick()
{
    std::cout << "Ticking DINO Detection\n";
    return BT::NodeStatus::SUCCESS;
}

BT::PortsList DINODetection::providedPorts() { return {}; }
