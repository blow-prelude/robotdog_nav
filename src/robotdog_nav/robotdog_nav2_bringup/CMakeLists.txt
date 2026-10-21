cmake_minimum_required(VERSION 3.8)
project(robotdog_nav2_bringup)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geographic_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(msg_set_msgs REQUIRED)
find_package(nav2_msgs REQUIRED)

find_package(PkgConfig REQUIRED)
pkg_check_modules(GeoLib REQUIRED geographiclib)

add_library(multi_goal_bridge_core
  src/geo_origin.cpp
  src/multi_goal_bridge.cpp
)
target_include_directories(multi_goal_bridge_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(multi_goal_bridge_core PUBLIC c_std_99 cxx_std_17)
ament_target_dependencies(multi_goal_bridge_core
  rclcpp
  std_msgs
  geometry_msgs
  nav_msgs
  sensor_msgs
  geographic_msgs
  tf2
  tf2_geometry_msgs
  msg_set_msgs
)
target_link_libraries(multi_goal_bridge_core
  ${GeoLib_LIBRARIES}
  tf2_geometry_msgs::tf2_geometry_msgs
)
target_include_directories(multi_goal_bridge_core PRIVATE ${GeoLib_INCLUDE_DIRS})

add_executable(multi_goal_bridge_node
  src/multi_goal_bridge_node.cpp
)
target_compile_features(multi_goal_bridge_node PUBLIC c_std_99 cxx_std_17)
ament_target_dependencies(multi_goal_bridge_node
  rclcpp
  rclcpp_action
  std_msgs
  std_srvs
  geometry_msgs
  nav_msgs
  sensor_msgs
  geographic_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  msg_set_msgs
  nav2_msgs
)
target_link_libraries(multi_goal_bridge_node
  multi_goal_bridge_core
  ${GeoLib_LIBRARIES}
  tf2_geometry_msgs::tf2_geometry_msgs
)
target_include_directories(multi_goal_bridge_node PRIVATE ${GeoLib_INCLUDE_DIRS})

install(TARGETS
  multi_goal_bridge_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

install(DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config
)

install(FILES README.md
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
