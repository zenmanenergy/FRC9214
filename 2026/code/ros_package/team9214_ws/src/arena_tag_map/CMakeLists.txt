cmake_minimum_required(VERSION 3.8)
project(arena_tag_map)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)

install(
  PROGRAMS
    scripts/load_map_in_nav2.sh
    scripts/generate_world_and_tag_maps.py
  DESTINATION lib/${PROJECT_NAME}
)

install(
  FILES README.md
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
