cmake_minimum_required(VERSION 3.8)
project(execute_task_server)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# C++17 requirement
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(msg_set_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# Include nlohmann/json for JSON parsing
find_package(nlohmann_json REQUIRED)

# Generate ROS2 interfaces (srv files)
rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/ExecuteTask.srv"
  DEPENDENCIES std_msgs
)

# Create executable
add_executable(execute_task_node src/execute_task_node.cpp)
ament_target_dependencies(execute_task_node
  rclcpp
  std_msgs
  geometry_msgs
  msg_set_msgs
  nlohmann_json
)

# Link to rosidl generated code
rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")
target_link_libraries(execute_task_node "${cpp_typesupport_target}")

# Install executable
install(TARGETS
  execute_task_node
  DESTINATION lib/${PROJECT_NAME}
)

# Install launch and config files
install(DIRECTORY
  launch
  config
  DESTINATION share/${PROJECT_NAME}/
)

# Testing
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
