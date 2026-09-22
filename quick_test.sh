#!/bin/bash
# Quick test for execute_task_server

SCRIPT_DIR=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)
cd "$SCRIPT_DIR" || exit

echo "=========================================="
echo "ExecuteTask Server Quick Test"
echo "=========================================="
echo ""

source /opt/ros/humble/setup.bash
source install/setup.bash

# Check if execute_task_server is already running
if ros2 service list 2>/dev/null | grep -q "/agent/uav1/execute_task"; then
    echo "✅ execute_task_server is already running"
    echo ""
    echo "Running tests..."
    echo ""
    python3 test_execute_task_py.py
else
    echo "⚠️  execute_task_server is not running"
    echo ""
    echo "Starting execute_task_server..."
    echo ""
    
    # Start in background
    ros2 launch execute_task_server execute_task.launch.py agent_id:=uav1 &
    SERVER_PID=$!
    
    # Wait for it to start
    echo "Waiting for service to be available..."
    for i in {1..10}; do
        if ros2 service list 2>/dev/null | grep -q "/agent/uav1/execute_task"; then
            echo "✅ Service available!"
            echo ""
            echo "Running tests..."
            echo ""
            python3 test_execute_task_py.py
            kill $SERVER_PID 2>/dev/null
            exit 0
        fi
        sleep 1
    done
    
    echo "❌ Service did not become available"
    kill $SERVER_PID 2>/dev/null
    exit 1
fi
