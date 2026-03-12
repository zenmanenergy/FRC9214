cmake_minimum_required(VERSION 3.8)
project(arena_bt_nodes)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(behaviortree_cpp REQUIRED)

add_library(${PROJECT_NAME} SHARED
  src/is_battery_ok_node.cpp
  src/register_nodes.cpp
  src/plugin_entrypoint.cpp
)
target_compile_definitions(${PROJECT_NAME}
  PRIVATE
    BT_PLUGIN_EXPORT
)
target_include_directories(${PROJECT_NAME}
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)
target_compile_features(${PROJECT_NAME} PUBLIC cxx_std_17)
target_link_libraries(${PROJECT_NAME} PUBLIC behaviortree_cpp::behaviortree_cpp)

add_executable(generate_bt_model_xml src/generate_bt_model_xml.cpp)
target_compile_features(generate_bt_model_xml PUBLIC cxx_std_17)
target_include_directories(generate_bt_model_xml
  PRIVATE
    include
)
target_link_libraries(generate_bt_model_xml
  PRIVATE
    ${PROJECT_NAME}
    behaviortree_cpp::behaviortree_cpp
)

install(
  TARGETS ${PROJECT_NAME} generate_bt_model_xml
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY include/
  DESTINATION include
)

install(
  DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
