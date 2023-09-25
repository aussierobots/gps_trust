cmake_minimum_required(VERSION 3.8)
project(gps_trust)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(gps_trust_msgs REQUIRED)
find_package(gps_trust_node REQUIRED)

install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()