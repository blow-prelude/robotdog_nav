# ExecuteTask Server

ROS2 service server for receiving multi-goal tasks from the command center via `multi_task_bridge`.

## Overview

This package provides a bridge between the command center's task management system and the robot's waypoint navigation system. It receives task requests in JSON format, parses them, and publishes `MultiGoal` messages that the `waypoint_nav` package can execute.

## Architecture

```
Command Center
    ↓ (MultiTask message)
/uav1/swarms_backend/global/multitask
    ↓
multi_task_bridge (parses & forwards)
    ↓ (ExecuteTask service)
/agent/uav1/execute_task  ← This package
    ↓ (MultiGoal message)
/waypoint_goals
    ↓
waypoint_nav → executes the path
```

## Features

- **JSON-based task interface**: Receives tasks in flexible JSON format
- **Multi-frame support**: Supports both GPS (lat/lon/alt) and local (x/y/z) coordinates
- **Service response**: Returns execution status to command center
- **Configurable agent ID**: Supports multiple agents with different IDs

## JSON Request Format

```json
{
  "mission_id": "patrol_01",
  "agent_id": "uav1",
  "capability_id": "goto_waypoints",
  "params": {
    "waypoint_frame": "gps",
    "waypoints": [
      {"lat": 30.06888, "lon": 120.04646, "alt": 5, "yaw": 0.0, "vel": 1.0},
      {"lat": 30.06890, "lon": 120.04650, "alt": 5, "yaw": 1.57, "vel": 0.8}
    ]
  }
}
```

### Fields

| Field | Type | Description |
|-------|------|-------------|
| `mission_id` | string | Unique mission identifier |
| `agent_id` | string | Target agent (must match configured agent_id) |
| `capability_id` | string | Task type: `goto_waypoints`, `takeoff`, `return` |
| `params.waypoint_frame` | string | Coordinate frame: `gps` or `local` |
| `params.waypoints` | array | List of waypoint objects |
| `waypoint.lat/x_or_lat` | float | Latitude (GPS) or X (local) |
| `waypoint.lon/y_or_lon` | float | Longitude (GPS) or Y (local) |
| `waypoint.alt/z_or_alt` | float | Altitude |
| `waypoint.yaw` | float | Optional: target yaw angle (rad) |
| `waypoint.vel` | float | Optional: target velocity (m/s) |

## JSON Response Format

### Success
```json
{
  "status": "success",
  "mission_id": "patrol_01",
  "waypoint_count": 4,
  "message": "Task accepted and executing"
}
```

### Error
```json
{
  "status": "error",
  "error": "Task not for this agent"
}
```

## Usage

### Build

```bash
cd /path/to/robotdog_nav
colcon build --packages-select execute_task_server
source install/setup.bash
```

### Launch

```bash
# With default parameters (agent_id=uav1)
ros2 launch execute_task_server execute_task.launch.py

# With custom agent ID
ros2 launch execute_task_server execute_task.launch.py agent_id:=dog1
```

### Test the Service

```bash
# Test with a sample request
ros2 service call /agent/uav1/execute_task multi_task_bridge/srv/ExecuteTask '{
  request_json: "{\"mission_id\":\"test_01\",\"agent_id\":\"uav1\",\"capability_id\":\"goto_waypoints\",\"params\":{\"waypoint_frame\":\"gps\",\"waypoints\":[{\"lat\":30.06888,\"lon\":120.04646,\"alt\":5},{\"lat\":30.06890,\"lon\":120.04650,\"alt\":5}]}}"
}'
```

### Parameters

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `agent_id` | string | `uav1` | Agent identifier |
| `default_waypoint_frame` | string | `gps` | Default coordinate frame |
| `default_velocity` | double | `1.0` | Default waypoint velocity (m/s) |
| `default_yaw` | double | `0.0` | Default yaw angle (rad) |

## Topics

### Published

| Topic | Type | Description |
|-------|------|-------------|
| `/waypoint_goals` | `msg_set_msgs/msg/MultiGoal` | Waypoint goals for navigation |

### Subscribed

None (service-based interface)

## Services

| Service | Type | Description |
|---------|------|-------------|
| `/agent/uav1/execute_task` | `multi_task_bridge/srv/ExecuteTask` | Task execution interface |

## Integration

### With waypoint_nav

The `execute_task_server` publishes to `/waypoint_goals`, which is directly subscribed by `waypoint_nav`. No additional configuration needed.

```bash
# Terminal 1: Start GPS/IMU fusion
ros2 launch gps_imu_fusion gps_imu_fusion.launch.py

# Terminal 2: Start waypoint navigation
ros2 launch waypoint_nav waypoint_nav.launch.py

# Terminal 3: Start execute_task server
ros2 launch execute_task_server execute_task.launch.py

# Terminal 4: Send task (via multi_task_bridge or direct service call)
ros2 service call /agent/uav1/execute_task ...
```

### With multi_task_bridge

The `multi_task_bridge` automatically forwards tasks to this service. Ensure the `agent_id` matches in both packages.

## Dependencies

- `rclcpp`
- `std_msgs`
- `geometry_msgs`
- `msg_set_msgs`
- `multi_task_bridge`
- `nlohmann_json`

## License

Apache-2.0
