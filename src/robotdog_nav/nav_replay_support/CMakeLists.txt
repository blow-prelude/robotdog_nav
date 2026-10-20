cmake_minimum_required(VERSION 3.8)
project(nav_replay_support)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)

# 里程计 → TF 适配节点
add_executable(odom_to_tf_node
  src/odom_to_tf_node.cpp
)
target_compile_features(odom_to_tf_node PUBLIC cxx_std_14)
ament_target_dependencies(odom_to_tf_node
  rclcpp
  nav_msgs
  geometry_msgs
  tf2_ros
)

install(TARGETS odom_to_tf_node
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
