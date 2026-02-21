#include <behaviortree_cpp/bt_factory.h>

#include "arena_bt_nodes/register_nodes.hpp"

BT_REGISTER_NODES(factory)
{
  arena_bt_nodes::RegisterNodes(factory);
}
