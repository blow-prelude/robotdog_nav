#!/bin/bash
# Test script for execute_task_server

SCRIPT_DIR=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)
cd "$SCRIPT_DIR" || exit

source /opt/ros/humble/setup.bash
source install/setup.bash

echo "=========================================="
echo "ExecuteTask Server Test"
echo "=========================================="
echo ""

# Test 1: GPS waypoints
echo "Test 1: Sending GPS waypoints..."
ros2 service call /agent/uav1/execute_task execute_task_server/srv/ExecuteTask '{
  request_json: "{\"mission_id\":\"test_gps_01\",\"agent_id\":\"uav1\",\"capability_id\":\"goto_waypoints\",\"params\":{\"waypoint_frame\":\"gps\",\"waypoints\":[{\"lat\":30.06888,\"lon\":120.04646,\"alt\":5,\"vel\":1.0},{\"lat\":30.06890,\"lon\":120.04650,\"alt\":5,\"vel\":0.8}]}}"
}'

echo ""
echo "Test 1 completed. Check /waypoint_goals topic for published MultiGoal message."
echo ""

# Test 2: Local waypoints
echo "Test 2: Sending local waypoints..."
ros2 service call /agent/uav1/execute_task execute_task_server/srv/ExecuteTask '{
  request_json: "{\"mission_id\":\"test_local_01\",\"agent_id\":\"uav1\",\"capability_id\":\"goto_waypoints\",\"params\":{\"waypoint_frame\":\"local\",\"waypoints\":[{\"x\":1.0,\"y\":0.0,\"z\":0.0,\"vel\":1.0},{\"x\":2.0,\"y\":1.0,\"z\":0.0,\"vel\":0.8}]}}"
}'

echo ""
echo "Test 2 completed."
echo ""

# Test 3: Wrong agent ID (should fail)
echo "Test 3: Sending to wrong agent (should return error)..."
ros2 service call /agent/uav1/execute_task execute_task_server/srv/ExecuteTask '{
  request_json: "{\"mission_id\":\"test_wrong_01\",\"agent_id\":\"uav2\",\"capability_id\":\"goto_waypoints\",\"params\":{\"waypoint_frame\":\"gps\",\"waypoints\":[{\"lat\":30.06888,\"lon\":120.04646,\"alt\":5}]}}"
}'

echo ""
echo "Test 3 completed. Should have received error response."
echo ""

# Test 4: Invalid JSON (should fail)
echo "Test 4: Sending invalid JSON (should fail)..."
ros2 service call /agent/uav1/execute_task execute_task_server/srv/ExecuteTask '{
  request_json: "invalid json string"
}'

echo ""
echo "All tests completed!"
echo ""
echo "To monitor waypoints in real-time, run:"
echo "  ros2 topic echo /waypoint_goals"
echo ""
