# arena_bringup docs index

This is the package-level entry point for architecture artifacts.

## Diagram Sets

### Flow diagrams (`flow__*`)
- Bringup (combined): [`diagrams/flow__bringup__combined__overview.puml`](diagrams/flow__bringup__combined__overview.puml) | [`svg`](diagrams/flow__bringup__combined__overview.svg)
- Bringup (single cam): [`diagrams/flow__bringup__single_cam__overview.puml`](diagrams/flow__bringup__single_cam__overview.puml) | [`svg`](diagrams/flow__bringup__single_cam__overview.svg)
- Bringup (multi cam): [`diagrams/flow__bringup__multi_cam__overview.puml`](diagrams/flow__bringup__multi_cam__overview.puml) | [`svg`](diagrams/flow__bringup__multi_cam__overview.svg)
- Navigation stack: [`diagrams/flow__navigation__stack__overview.puml`](diagrams/flow__navigation__stack__overview.puml) | [`svg`](diagrams/flow__navigation__stack__overview.svg)
- Localization (`ekf_map`): [`diagrams/flow__localization__ekf_map__overview.puml`](diagrams/flow__localization__ekf_map__overview.puml) | [`svg`](diagrams/flow__localization__ekf_map__overview.svg)
- Sim harness: [`diagrams/flow__sim__harness__overview.puml`](diagrams/flow__sim__harness__overview.puml) | [`svg`](diagrams/flow__sim__harness__overview.svg)

### Per-node topic diagrams (`topics__*`)
- Bringup (multi cam): [`diagrams/topics__bringup__multi_cam__per_node.puml`](diagrams/topics__bringup__multi_cam__per_node.puml) | [`svg`](diagrams/topics__bringup__multi_cam__per_node.svg)
- Navigation stack: [`diagrams/topics__navigation__stack__per_node.puml`](diagrams/topics__navigation__stack__per_node.puml) | [`svg`](diagrams/topics__navigation__stack__per_node.svg)
- Localization (`ekf_map`): [`diagrams/topics__localization__ekf_map__per_node.puml`](diagrams/topics__localization__ekf_map__per_node.puml) | [`svg`](diagrams/topics__localization__ekf_map__per_node.svg)
- Sim harness: [`diagrams/topics__sim__harness__per_node.puml`](diagrams/topics__sim__harness__per_node.puml) | [`svg`](diagrams/topics__sim__harness__per_node.svg)
- Sim inputs generated topics: [`diagrams/topics__sim__inputs__generated.puml`](diagrams/topics__sim__inputs__generated.puml) | [`svg`](diagrams/topics__sim__inputs__generated.svg)

### Bridge and mapping diagrams
- NT4/ROS bridge overview: [`diagrams/bridge__roborio__nt4_ros__overview.puml`](diagrams/bridge__roborio__nt4_ros__overview.puml) | [`svg`](diagrams/bridge__roborio__nt4_ros__overview.svg)
- ROS->NT4 key map table: [`diagrams/table__roborio__ros_to_nt4__map.puml`](diagrams/table__roborio__ros_to_nt4__map.puml) | [`svg`](diagrams/table__roborio__ros_to_nt4__map.svg)

## Supporting Assets
- Diagram generation notes: [`diagrams/README.md`](diagrams/README.md)
- Source spreadsheet: [`excel/node_topics_sim_stack.xlsx`](excel/node_topics_sim_stack.xlsx)
