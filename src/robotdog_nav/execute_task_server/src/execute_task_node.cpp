#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <msg_set_msgs/msg/multi_goal.hpp>
#include <msg_set_msgs/msg/multi_goal_point.hpp>
#include <execute_task_server/srv/execute_task.hpp>
#include <nlohmann/json.hpp>

#include <memory>
#include <string>
#include <vector>

using json = nlohmann::json;

/**
 * @brief ExecuteTask Service Server for receiving multi-goal tasks from command center
 * 
 * This node receives task requests from multi_task_bridge, parses JSON payloads,
 * converts them to MultiGoal messages, and publishes to /waypoint_goals topic
 * for the waypoint_nav package to execute.
 */
class ExecuteTaskNode : public rclcpp::Node
{
public:
  explicit ExecuteTaskNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
  : Node("execute_task_node", options),
    task_count_(0)
  {
    // Declare parameters
    this->declare_parameter<std::string>("agent_id", "uav1");
    this->declare_parameter<std::string>("default_waypoint_frame", "gps");
    this->declare_parameter<double>("default_velocity", 1.0);
    this->declare_parameter<double>("default_yaw", 0.0);

    // Get parameters
    this->get_parameter("agent_id", agent_id_);
    this->get_parameter("default_waypoint_frame", default_waypoint_frame_);
    this->get_parameter("default_velocity", default_velocity_);
    this->get_parameter("default_yaw", default_yaw_);

    // Create service server
    std::string service_name = "/agent/" + agent_id_ + "/execute_task";
    execute_task_service_ = this->create_service<execute_task_server::srv::ExecuteTask>(
      service_name,
      std::bind(&ExecuteTaskNode::handleExecuteTask, this, std::placeholders::_1, std::placeholders::_2)
    );

    // Create publisher for MultiGoal messages
    waypoint_goal_pub_ = this->create_publisher<msg_set_msgs::msg::MultiGoal>(
      "/waypoint_goals", 10
    );

    RCLCPP_INFO(this->get_logger(), "ExecuteTask Node initialized");
    RCLCPP_INFO(this->get_logger(), "  - Service: %s", service_name.c_str());
    RCLCPP_INFO(this->get_logger(), "  - Publishing to: /waypoint_goals");
    RCLCPP_INFO(this->get_logger(), "  - Agent ID: %s", agent_id_.c_str());
    RCLCPP_INFO(this->get_logger(), "  - Default waypoint frame: %s", default_waypoint_frame_.c_str());
  }

private:
  /**
   * @brief Handle ExecuteTask service requests
   * 
   * @param request_json JSON string containing task information
   * @param response_json JSON string containing execution result
   */
  void handleExecuteTask(
    const std::shared_ptr<execute_task_server::srv::ExecuteTask::Request> request,
    std::shared_ptr<execute_task_server::srv::ExecuteTask::Response> response)
  {
    RCLCPP_INFO(this->get_logger(), "Received ExecuteTask request: %s", request->request_json.c_str());

    try {
      // Parse JSON request
      json request_data = json::parse(request->request_json);
      
      // Extract mission information
      std::string mission_id = request_data.value("mission_id", "unknown_mission");
      std::string agent_id = request_data.value("agent_id", "");
      std::string capability_id = request_data.value("capability_id", "");

      RCLCPP_INFO(this->get_logger(), "Mission ID: %s, Agent: %s, Capability: %s",
                  mission_id.c_str(), agent_id.c_str(), capability_id.c_str());

      // Check if this agent should handle the task
      if (!agent_id.empty() && agent_id != agent_id_) {
        RCLCPP_WARN(this->get_logger(), "Task not for this agent (expected: %s, got: %s)",
                    agent_id_.c_str(), agent_id.c_str());
        response->response_json = createErrorResponse("Task not for this agent");
        return;
      }

      // Handle different capability types
      if (capability_id == "goto_waypoints" || capability_id == "takeoff" || capability_id == "return") {
        // Parse waypoints and publish MultiGoal
        auto multi_goal = parseWaypoints(request_data);
        
        if (multi_goal) {
          // Publish the MultiGoal message
          waypoint_goal_pub_->publish(*multi_goal);
          
          task_count_++;
          RCLCPP_INFO(this->get_logger(), "Published MultiGoal with %zu waypoints (task #%zu)",
                      multi_goal->multi_goal_points.size(), task_count_);
          
          // Create success response
          response->response_json = createSuccessResponse(mission_id, multi_goal->multi_goal_points.size());
        } else {
          response->response_json = createErrorResponse("Failed to parse waypoints");
        }
      } else {
        RCLCPP_WARN(this->get_logger(), "Unsupported capability_id: %s", capability_id.c_str());
        response->response_json = createErrorResponse("Unsupported capability: " + capability_id);
      }

    } catch (const json::parse_error & e) {
      RCLCPP_ERROR(this->get_logger(), "JSON parse error: %s", e.what());
      response->response_json = createErrorResponse("JSON parse error: " + std::string(e.what()));
    } catch (const std::exception & e) {
      RCLCPP_ERROR(this->get_logger(), "Exception: %s", e.what());
      response->response_json = createErrorResponse("Exception: " + std::string(e.what()));
    }
  }

  /**
   * @brief Parse waypoints from JSON request and create MultiGoal message
   * 
   * @param request_data JSON object containing task parameters
   * @return Shared pointer to MultiGoal message, or nullptr if parsing failed
   */
  msg_set_msgs::msg::MultiGoal::SharedPtr parseWaypoints(const json & request_data)
  {
    auto multi_goal = std::make_shared<msg_set_msgs::msg::MultiGoal>();
    
    // Get params object
    if (!request_data.contains("params")) {
      RCLCPP_ERROR(this->get_logger(), "Missing 'params' in request");
      return nullptr;
    }

    const auto & params = request_data["params"];

    // Determine waypoint frame (gps or local)
    // Support direct is_gps_aid override from JSON params
    bool is_gps_aid_override = false;
    bool has_is_gps_aid = params.contains("is_gps_aid");
    if (has_is_gps_aid) {
      is_gps_aid_override = params.value("is_gps_aid", false);
    }
    
    std::string waypoint_frame = params.value("waypoint_frame", default_waypoint_frame_);
    // Use is_gps_aid from JSON if provided, otherwise derive from waypoint_frame
    multi_goal->is_gps_aid = has_is_gps_aid ? is_gps_aid_override : (waypoint_frame == "gps");
    
    // Determine altitude frame
    multi_goal->is_gps_hgt = params.value("is_gps_hgt", true);
    
    // Get control mode (default: 0 = vel & yawrate)
    multi_goal->ctl_mode = static_cast<int8_t>(params.value("ctl_mode", 0));

    // Parse waypoints array
    if (!params.contains("waypoints")) {
      RCLCPP_ERROR(this->get_logger(), "Missing 'waypoints' in params");
      return nullptr;
    }

    const auto & waypoints = params["waypoints"];
    
    for (const auto & wp : waypoints) {
      msg_set_msgs::msg::MultiGoalPoint point;
      
      // Extract coordinates based on frame
      if (multi_goal->is_gps_aid) {
        // GPS coordinates (lat/lon/alt)
        point.x_or_lat = wp.value("lat", 0.0);
        point.y_or_lon = wp.value("lon", 0.0);
        point.z_or_alt = wp.value("alt", 0.0);
      } else {
        // Local coordinates (x/y/z)
        point.x_or_lat = wp.value("x", 0.0);
        point.y_or_lon = wp.value("y", 0.0);
        point.z_or_alt = wp.value("z", 0.0);
      }
      
      // Extract optional parameters
      point.yaw = static_cast<float>(wp.value("yaw", default_yaw_));
      point.vel = static_cast<float>(wp.value("vel", default_velocity_));
      
      multi_goal->multi_goal_points.push_back(point);
    }

    if (multi_goal->multi_goal_points.empty()) {
      RCLCPP_ERROR(this->get_logger(), "No waypoints parsed from request");
      return nullptr;
    }

    return multi_goal;
  }

  /**
   * @brief Create success response JSON
   */
  std::string createSuccessResponse(const std::string & mission_id, size_t waypoint_count)
  {
    json response_data;
    response_data["status"] = "success";
    response_data["mission_id"] = mission_id;
    response_data["waypoint_count"] = waypoint_count;
    response_data["message"] = "Task accepted and executing";
    return response_data.dump();
  }

  /**
   * @brief Create error response JSON
   */
  std::string createErrorResponse(const std::string & error_message)
  {
    json response_data;
    response_data["status"] = "error";
    response_data["error"] = error_message;
    return response_data.dump();
  }

  // ROS2 interfaces
  rclcpp::Service<execute_task_server::srv::ExecuteTask>::SharedPtr execute_task_service_;
  rclcpp::Publisher<msg_set_msgs::msg::MultiGoal>::SharedPtr waypoint_goal_pub_;

  // Configuration
  std::string agent_id_;
  std::string default_waypoint_frame_;
  double default_velocity_;
  double default_yaw_;

  // State
  size_t task_count_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<ExecuteTaskNode>();
  RCLCPP_INFO(node->get_logger(), "ExecuteTask Node started");
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
