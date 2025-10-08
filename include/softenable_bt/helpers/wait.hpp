#pragma once

#include <behaviortree_cpp/bt_factory.h>
#include <chrono>
#include <thread>
#include <iostream>


class Wait : public BT::SyncActionNode
{
public:
  Wait(const std::string& name, const BT::NodeConfig& config)
    : BT::SyncActionNode(name, config)
  {}

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<double>("seconds", "Time to wait in seconds")
    };
  }

  BT::NodeStatus tick() override
  {
    double seconds = 0.0;
    if (!getInput<double>("seconds", seconds))
    {
      throw BT::RuntimeError("Missing required input [seconds]");
    }

    std::cout << "[Wait] Waiting for " << seconds << " seconds..." << std::endl;

    std::this_thread::sleep_for(std::chrono::duration<double>(seconds));

    std::cout << "[Wait] Done waiting." << std::endl;
    return BT::NodeStatus::SUCCESS;
  }
};
