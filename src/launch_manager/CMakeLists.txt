cmake_minimum_required(VERSION 3.8)
project(launch_manager)

find_package(ament_cmake REQUIRED)

# Install launch files.
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()