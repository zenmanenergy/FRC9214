#include <fstream>
#include <iostream>
#include <string>

#include <behaviortree_cpp/bt_factory.h>
#include <behaviortree_cpp/xml_parsing.h>

#include "arena_bt_nodes/register_nodes.hpp"

int main(int argc, char ** argv)
{
  if (argc != 2) {
    std::cerr << "Usage: generate_bt_model_xml <output_xml_path>\n";
    return 1;
  }

  const std::string output_path = argv[1];

  BT::BehaviorTreeFactory factory;
  arena_bt_nodes::RegisterNodes(factory);

  const std::string xml = BT::writeTreeNodesModelXML(factory, true);

  std::ofstream out(output_path);
  if (!out.is_open()) {
    std::cerr << "Failed to open output file: " << output_path << "\n";
    return 2;
  }
  out << xml;
  out.close();

  std::cout << "Wrote Groot2 node model XML: " << output_path << "\n";
  return 0;
}
