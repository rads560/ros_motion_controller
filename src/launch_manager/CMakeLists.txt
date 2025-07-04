cmake_minimum_required(VERSION 3.8)
project(launch_manager)

# Install launch files.
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)
