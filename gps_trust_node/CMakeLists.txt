cmake_minimum_required(VERSION 3.8)
project(gps_trust_node)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(std_msgs REQUIRED)
find_package(ublox_ubx_msgs REQUIRED)
find_package(rtcm_msgs REQUIRED)
find_package(gps_trust_msgs REQUIRED)

# Find jsoncpp
find_package(PkgConfig REQUIRED)
pkg_check_modules(JSONCPP jsoncpp)
find_package(jsoncpp REQUIRED)
# link_libraries(${JSONCPP_LIBRARIES})
# include_directories(${JSONCPP_INCLUDE_DIRS})

find_package(libcurl_vendor REQUIRED)

pkg_check_modules(ZLIB zlib)
find_package(ZLIB REQUIRED)

include_directories(include SYSTEM ${JSONCPP_INCLUDE_DIRS} ${ZLIB_INCLUDE_DIRS})

add_library(gps_trust_components SHARED
  src/gps_trust_node.cpp
)

target_compile_definitions(gps_trust_components
  PRIVATE "GPS_TRUST_NODE_BUILDING_DLL"
)

ament_target_dependencies(gps_trust_components
  rclcpp
  rclcpp_components
  std_msgs
  ublox_ubx_msgs
  rtcm_msgs
  gps_trust_msgs
  libcurl_vendor
)

# Link against system libraries
target_link_libraries(gps_trust_components
  ${JSONCPP_LIBRARIES}
  ${ZLIB_LIBRARIES}
)

rclcpp_components_register_node(gps_trust_components
  PLUGIN "gps_trust::GPSTrustNode"
  EXECUTABLE gps_trust_node
)

install(TARGETS
  gps_trust_components
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
