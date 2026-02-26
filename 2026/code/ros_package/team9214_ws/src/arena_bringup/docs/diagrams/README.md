# arena_bringup/docs

## Generate PlantUML diagrams

`plantuml team9214_ws/src/arena_bringup/docs/flow__bringup__combined__overview.puml`
`plantuml team9214_ws/src/arena_bringup/docs/flow__bringup__multi_cam__overview.puml`
`plantuml team9214_ws/src/arena_bringup/docs/flow__bringup__single_cam__overview.puml`
`plantuml team9214_ws/src/arena_bringup/docs/flow__localization__ekf_map__overview.puml`
`plantuml team9214_ws/src/arena_bringup/docs/flow__navigation__stack__overview.puml`
`plantuml team9214_ws/src/arena_bringup/docs/flow__sim__harness__overview.puml`
`plantuml team9214_ws/src/arena_bringup/docs/topics__bringup__multi_cam__per_node.puml`
`plantuml team9214_ws/src/arena_bringup/docs/topics__localization__ekf_map__per_node.puml`
`plantuml team9214_ws/src/arena_bringup/docs/topics__navigation__stack__per_node.puml`
`plantuml team9214_ws/src/arena_bringup/docs/topics__sim__harness__per_node.puml`
`plantuml team9214_ws/src/arena_bringup/docs/topics__sim__inputs__generated.puml`
`plantuml team9214_ws/src/arena_bringup/docs/bridge__roborio__nt4_ros__overview.puml`
`plantuml team9214_ws/src/arena_bringup/docs/table__roborio__ros_to_nt4__map.puml`

## Generate SVGs with Docker Compose

- Run the compose file from `doc/` directory:

`docker compose run --rm plantuml`

## Host plantuml server to autogen images with api

`docker run -d -p 8080:8080 plantuml/plantuml-server:jetty`
