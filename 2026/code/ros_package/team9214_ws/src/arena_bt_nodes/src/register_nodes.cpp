#include "arena_bt_nodes/is_battery_ok_node.hpp"
#include "arena_bt_nodes/register_nodes.hpp"

namespace arena_bt_nodes
{

void RegisterNodes(BT::BehaviorTreeFactory & factory)
{
  factory.registerNodeType<IsBatteryOk>("IsBatteryOk");

  factory.addMetadataToManifest(
    "IsBatteryOk",
    {{"category", "custom"}, {"doc", "Returns SUCCESS when battery_voltage >= min_voltage."}});
}

}  // namespace arena_bt_nodes
