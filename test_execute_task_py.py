#!/usr/bin/env python3
"""Full system test for execute_task_server → waypoint_nav"""

import json
import rclpy
from execute_task_server.srv import ExecuteTask

def main():
    rclpy.init()
    node = rclpy.create_node('test_client')
    client = node.create_client(ExecuteTask, '/agent/uav1/execute_task')
    
    if not client.wait_for_service(timeout_sec=5.0):
        node.get_logger().error("Service not available!")
        return
    
    print("=" * 50)
    print("Full System Test: execute_task_server → waypoint_nav")
    print("=" * 50)
    print()
    
    # Test 1: Local waypoints
    print("Test 1: Local waypoints (3 points)")
    request1 = ExecuteTask.Request()
    request1.request_json = json.dumps({
        "mission_id": "local_test_01",
        "agent_id": "uav1",
        "capability_id": "goto_waypoints",
        "params": {
            "waypoint_frame": "local",
            "waypoints": [
                {"x": 1.0, "y": 0.0, "z": 0.0, "vel": 0.5},
                {"x": 2.0, "y": 1.0, "z": 0.0, "vel": 0.5},
                {"x": 0.0, "y": 2.0, "z": 0.0, "vel": 0.5}
            ]
        }
    })
    future1 = client.call_async(request1)
    rclpy.spin_until_future_complete(node, future1)
    response1 = future1.result()
    print(f"Response: {response1.response_json}")
    print()
    
    # Test 2: GPS waypoints (small offsets for simulation)
    # 使用小 GPS 坐标值，与 ros2 topic pub 命令效果一致
    print("Test 2: GPS waypoints (4 points with small offsets)")
    request2 = ExecuteTask.Request()
    request2.request_json = json.dumps({
        "mission_id": "gps_test_01",
        "agent_id": "uav1",
        "capability_id": "goto_waypoints",
        "params": {
            "waypoint_frame": "gps",
            "waypoints": [
                {"lat": 0.00012, "lon": 0.00002, "alt": 0.0, "vel": 1.0},
                {"lat": -0.00015, "lon": 0.00013, "alt": 0.0, "vel": 0.8},
                {"lat": -0.00005, "lon": -0.00010, "alt": 0.0, "vel": 0.8},
                {"lat": 0.00008, "lon": -0.00008, "alt": 0.0, "vel": 1.0}
            ]
        }
    })
    future2 = client.call_async(request2)
    rclpy.spin_until_future_complete(node, future2)
    response2 = future2.result()
    print(f"Response: {response2.response_json}")
    print()
    
    # Test 3: Wrong agent ID (should fail)
    print("Test 3: Wrong agent ID (should return error)")
    request3 = ExecuteTask.Request()
    request3.request_json = json.dumps({
        "mission_id": "wrong_agent_test",
        "agent_id": "dog2",
        "capability_id": "goto_waypoints",
        "params": {
            "waypoint_frame": "gps",
            "waypoints": [{"lat": 30.0, "lon": 120.0, "alt": 5.0}]
        }
    })
    future3 = client.call_async(request3)
    rclpy.spin_until_future_complete(node, future3)
    response3 = future3.result()
    print(f"Response: {response3.response_json}")
    print()
    
    # Test 4: Invalid capability (should fail)
    print("Test 4: Invalid capability (should return error)")
    request4 = ExecuteTask.Request()
    request4.request_json = json.dumps({
        "mission_id": "invalid_capability",
        "agent_id": "uav1",
        "capability_id": "invalid_capability",
        "params": {
            "waypoint_frame": "gps",
            "waypoints": [{"lat": 30.0, "lon": 120.0, "alt": 5.0}]
        }
    })
    future4 = client.call_async(request4)
    rclpy.spin_until_future_complete(node, future4)
    response4 = future4.result()
    print(f"Response: {response4.response_json}")
    print()
    
    print("=" * 50)
    print("All tests completed!")
    print("=" * 50)
    
    node.destroy_node()

if __name__ == '__main__':
    main()
