#!/bin/bash
# Full system test for execute_task_server → waypoint_nav

source /media/lenovo/1/project/gazebo/0713/robotdog_nav/install/setup.bash

echo "=========================================="
echo "Full System Test: execute_task_server"
echo "=========================================="
echo ""

# Test 1: Local coordinates
echo "Test 1: Local waypoints (3 points)"
ros2 service call /agent/uav1/execute_task execute_task_server/srv/ExecuteTask "{
  request_json: '{\"mission_id\":\"local_test\",\"agent_id\":\"uav1\",\"capability_id\":\"goto_waypoints\",\"params\":{\"waypoint_frame\":\"local\",\"waypoints\":[{\"x\":1.0,\"y\":0.0,\"z\":0.0},{\"x\":2.0,\"y\":1.0,\"z\":0.0},{\"x\":0.0,\"y\":2.0,\"z\":0.0}]}'
}"
echo ""

# Test 2: GPS coordinates
echo "Test 2: GPS waypoints (2 points)"
ros2 service call /agent/uav1/execute_task execute_task_server/srv/ExecuteTask "{
  request_json: '{\"mission_id\":\"gps_test\",\"agent_id\":\"uav1\",\"capability_id\":\"goto_waypoints\",\"params\":{\"waypoint_frame\":\"gps\",\"waypoints\":[{\"lat\":30.06888,\"lon\":120.04646,\"alt\":5},{\"lat\":30.06890,\"lon\":120.04650,\"alt\":5}]}'
}"
echo ""

# Test 3: Wrong agent (should fail)
echo "Test 3: Wrong agent ID (should return error)"
ros2 service call /agent/uav1/execute_task execute_task_server/srv/ExecuteTask "{
  request_json: '{\"mission_id\":\"wrong_agent\",\"agent_id\":\"dog2\",\"capability_id\":\"goto_waypoints\",\"params\":{\"waypoint_frame\":\"gps\",\"waypoints\":[{\"lat\":30.0,\"lon\":120.0,\"alt\":5}]}'
}"
echo ""

echo "=========================================="
echo "Tests completed!"
echo "=========================================="
echo ""
echo "Check node logs for details:"
echo "  - execute_task_node: ros2 topic echo /rosout --filter 'node_name==\"execute_task_node\"'"
echo "  - nav_waypoint_node: ros2 topic echo /rosout --filter 'node_name==\"nav_waypoint_node\"'"
echo ""
