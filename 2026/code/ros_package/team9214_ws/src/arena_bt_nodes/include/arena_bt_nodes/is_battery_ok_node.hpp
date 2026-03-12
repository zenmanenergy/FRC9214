#pragma once

#include <behaviortree_cpp/condition_node.h>

namespace arena_bt_nodes
{

class IsBatteryOk : public BT::ConditionNode
{
public:
  IsBatteryOk(const std::string & name, const BT::NodeConfig & config);

  static BT::PortsList providedPorts();

  BT::NodeStatus tick() override;
};

}  // namespace arena_bt_nodes
