cmake_minimum_required(VERSION 3.8)
project(tag_pose_observer)

# ament_python packages don't build code via CMake, but CMakeLists.txt is still expected.
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

# Install the python package (matches setup.py package name)
ament_python_install_package(${PROJECT_NAME})

# Install launch + config files
install(DIRECTORY
  launch
  config
  DESTINATION share/${PROJECT_NAME}
)

# Install the ament resource index marker
install(FILES
  resource/${PROJECT_NAME}
  DESTINATION share/ament_index/resource_index/packages
)

# Install package.xml
install(FILES
  package.xml
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
