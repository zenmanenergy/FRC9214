# arena_bringup/docs

## Generate PlantUML diagrams

`plantuml team9214_ws/src/arena_bringup/docs/bringup_nodes_topics.puml`
`plantuml team9214_ws/src/arena_bringup/docs/bringup_single_cam_nodes_topics.puml`
`plantuml team9214_ws/src/arena_bringup/docs/bringup_multi_cam_nodes_topics.puml`
`plantuml team9214_ws/src/arena_bringup/docs/roborio_to_team9214_blackbox.puml`
`plantuml team9214_ws/src/arena_bringup/docs/ros_topics_to_roborio_nt4_table.puml`
`plantuml team9214_ws/src/arena_bringup/docs/sim_inputs_generated_topics.puml`

## Generate SVGs with Docker Compose

- Run the compose file from `doc/` directory:

`docker compose run --rm plantuml`

## Host plantuml server to autogen images with api

`docker run -d -p 8080:8080 plantuml/plantuml-server:jetty`
