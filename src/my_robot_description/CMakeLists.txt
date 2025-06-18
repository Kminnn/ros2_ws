cmake_minimum_required(VERSION 3.8)
project(my_robot_description)

find_package(ament_cmake REQUIRED)

# Install URDF and launch files
install(DIRECTORY urdf launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()

