#include "arena_bt_nodes/is_battery_ok_node.hpp"

namespace arena_bt_nodes
{

IsBatteryOk::IsBatteryOk(const std::string & name, const BT::NodeConfig & config)
: BT::ConditionNode(name, config)
{
}

BT::PortsList IsBatteryOk::providedPorts()
{
  return {
    BT::InputPort<double>("min_voltage", 11.0, "Minimum battery voltage required"),
    BT::InputPort<double>("battery_voltage", 12.0, "Current battery voltage")
  };
}

BT::NodeStatus IsBatteryOk::tick()
{
  const auto min_v = getInput<double>("min_voltage");
  if (!min_v) {
    throw BT::RuntimeError("missing input [min_voltage]: ", min_v.error());
  }

  const auto batt_v = getInput<double>("battery_voltage");
  if (!batt_v) {
    throw BT::RuntimeError("missing input [battery_voltage]: ", batt_v.error());
  }

  return (*batt_v >= *min_v) ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
}

}  // namespace arena_bt_nodes
