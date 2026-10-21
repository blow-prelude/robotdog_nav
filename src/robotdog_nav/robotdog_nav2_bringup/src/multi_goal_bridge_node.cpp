#include <algorithm>
#include <chrono>
#include <cmath>
#include <memory>
#include <optional>
#include <string>
#include <utility>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

#include "std_msgs/msg/string.hpp"
#include "std_srvs/srv/trigger.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "sensor_msgs/msg/nav_sat_fix.hpp"
#include "geographic_msgs/msg/geo_point_stamped.hpp"
#include "nav2_msgs/action/follow_waypoints.hpp"
#include "nav2_msgs/msg/speed_limit.hpp"
#include "msg_set_msgs/msg/multi_goal.hpp"

#include "tf2/LinearMath/Quaternion.h"
#include "tf2/utils.h"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

#include "robotdog_nav2_bringup/geo_origin.hpp"
#include "robotdog_nav2_bringup/multi_goal_bridge.hpp"

namespace robotdog_nav2_bringup
{

class MultiGoalBridgeNode : public rclcpp::Node
{
public:
  using FollowWaypoints = nav2_msgs::action::FollowWaypoints;
  using GoalHandle = rclcpp_action::ClientGoalHandle<FollowWaypoints>;

  MultiGoalBridgeNode()
  : Node("multi_goal_bridge_node")
  {
    declareParameters();
    readParameters();

    origin_.setOrigin(origin_lat_, origin_lon_, origin_alt_);

    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    status_pub_ = this->create_publisher<std_msgs::msg::String>(nav_status_topic_, 10);
    speed_limit_pub_ =
      this->create_publisher<nav2_msgs::msg::SpeedLimit>(speed_limit_topic_, 10);

    goal_sub_ = this->create_subscription<msg_set_msgs::msg::MultiGoal>(
      multi_goal_topic_, 10,
      std::bind(&MultiGoalBridgeNode::goalCallback, this, std::placeholders::_1));
    odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
      odom_topic_, 10,
      std::bind(&MultiGoalBridgeNode::odomCallback, this, std::placeholders::_1));
    gps_sub_ = this->create_subscription<sensor_msgs::msg::NavSatFix>(
      gps_topic_, 10,
      std::bind(&MultiGoalBridgeNode::gpsCallback, this, std::placeholders::_1));

    if (origin_source_ == "topic") {
      origin_sub_ = this->create_subscription<geographic_msgs::msg::GeoPointStamped>(
        origin_topic_, 10,
        std::bind(&MultiGoalBridgeNode::originCallback, this, std::placeholders::_1));
    }

    cancel_srv_ = this->create_service<std_srvs::srv::Trigger>(
      cancel_service_,
      std::bind(
        &MultiGoalBridgeNode::cancelCallback, this,
        std::placeholders::_1, std::placeholders::_2));

    follow_waypoints_client_ =
      rclcpp_action::create_client<FollowWaypoints>(this, follow_waypoints_action_);

    const auto status_period = std::chrono::duration_cast<std::chrono::milliseconds>(
      std::chrono::duration<double>(1.0 / status_rate_hz_));
    status_timer_ = this->create_wall_timer(
      status_period, std::bind(&MultiGoalBridgeNode::statusTimer, this));
    work_timer_ = this->create_wall_timer(
      std::chrono::milliseconds(100), std::bind(&MultiGoalBridgeNode::workTimer, this));

    setStatus(Status::WAITING_FOR_GOALS);

    RCLCPP_INFO(
      this->get_logger(),
      "MultiGoal bridge initialized: origin=(%.6f, %.6f, %.2f), source=%s, "
      "yaw_mode=%s, action=%s",
      origin_lat_, origin_lon_, origin_alt_, origin_source_.c_str(),
      yaw_mode_.c_str(), follow_waypoints_action_.c_str());
  }

private:
  enum class Status
  {
    WAITING_FOR_GOALS,
    INITIALIZING,
    EXECUTING_PATH,
    GOAL_REACHED,
    COMPLETED,
    FAILED,
  };

  enum class CancelReason
  {
    NONE,
    PREEMPT,
    USER,
    TIMEOUT,
    STALL,
  };

  void declareParameters()
  {
    this->declare_parameter("origin_lat", 0.0);
    this->declare_parameter("origin_lon", 0.0);
    this->declare_parameter("origin_alt", 0.0);
    this->declare_parameter("origin_source", std::string("params"));
    this->declare_parameter(
      "origin_topic", std::string("/mavros/global_position/gp_origin"));

    this->declare_parameter("world_frame", std::string("world"));
    this->declare_parameter("robot_base_frame", std::string("base_link"));
    this->declare_parameter("odom_topic", std::string("/world_odom"));
    this->declare_parameter("gps_topic", std::string("/gps/data"));
    this->declare_parameter("multi_goal_topic", std::string("/waypoint_goals"));
    this->declare_parameter("follow_waypoints_action", std::string("follow_waypoints"));
    this->declare_parameter("nav_status_topic", std::string("/nav_status"));
    this->declare_parameter("speed_limit_topic", std::string("speed_limit"));
    this->declare_parameter("cancel_service", std::string("nav/cancel"));

    this->declare_parameter("yaw_mode", std::string("path"));
    this->declare_parameter("respect_waypoint_velocity", true);
    this->declare_parameter("min_goal_velocity", 0.1);
    this->declare_parameter("max_goal_velocity", 0.6);

    this->declare_parameter("action_server_wait_timeout_sec", 5.0);
    this->declare_parameter("goal_retry_interval_sec", 0.5);
    this->declare_parameter("cancel_timeout_sec", 3.0);
    this->declare_parameter("stall_timeout_sec", 30.0);
    this->declare_parameter("task_timeout_sec", 0.0);
    this->declare_parameter("require_tf_before_goal", true);
    this->declare_parameter("tf_wait_timeout_sec", 5.0);

    this->declare_parameter("origin_check_enabled", true);
    this->declare_parameter("origin_check_tolerance", 5.0);
    this->declare_parameter("origin_check_max_age_sec", 1.0);
    this->declare_parameter("max_goal_distance_m", 5000.0);

    this->declare_parameter("status_rate_hz", 10.0);
    this->declare_parameter("status_completed_hold_sec", 2.0);
    this->declare_parameter("status_goal_reached_hold_sec", 0.3);
  }

  void readParameters()
  {
    this->get_parameter("origin_lat", origin_lat_);
    this->get_parameter("origin_lon", origin_lon_);
    this->get_parameter("origin_alt", origin_alt_);
    this->get_parameter("origin_source", origin_source_);
    this->get_parameter("origin_topic", origin_topic_);

    this->get_parameter("world_frame", world_frame_);
    this->get_parameter("robot_base_frame", robot_base_frame_);
    this->get_parameter("odom_topic", odom_topic_);
    this->get_parameter("gps_topic", gps_topic_);
    this->get_parameter("multi_goal_topic", multi_goal_topic_);
    this->get_parameter("follow_waypoints_action", follow_waypoints_action_);
    this->get_parameter("nav_status_topic", nav_status_topic_);
    this->get_parameter("speed_limit_topic", speed_limit_topic_);
    this->get_parameter("cancel_service", cancel_service_);

    this->get_parameter("yaw_mode", yaw_mode_);
    this->get_parameter("respect_waypoint_velocity", respect_waypoint_velocity_);
    this->get_parameter("min_goal_velocity", min_goal_velocity_);
    this->get_parameter("max_goal_velocity", max_goal_velocity_);

    this->get_parameter("action_server_wait_timeout_sec", action_server_wait_timeout_sec_);
    this->get_parameter("goal_retry_interval_sec", goal_retry_interval_sec_);
    this->get_parameter("cancel_timeout_sec", cancel_timeout_sec_);
    this->get_parameter("stall_timeout_sec", stall_timeout_sec_);
    this->get_parameter("task_timeout_sec", task_timeout_sec_);
    this->get_parameter("require_tf_before_goal", require_tf_before_goal_);
    this->get_parameter("tf_wait_timeout_sec", tf_wait_timeout_sec_);

    this->get_parameter("origin_check_enabled", origin_check_enabled_);
    this->get_parameter("origin_check_tolerance", origin_check_tolerance_);
    this->get_parameter("origin_check_max_age_sec", origin_check_max_age_sec_);
    this->get_parameter("max_goal_distance_m", max_goal_distance_m_);

    this->get_parameter("status_rate_hz", status_rate_hz_);
    this->get_parameter("status_completed_hold_sec", status_completed_hold_sec_);
    this->get_parameter("status_goal_reached_hold_sec", status_goal_reached_hold_sec_);

    if (origin_source_ != "params" && origin_source_ != "topic") {
      RCLCPP_WARN(
        this->get_logger(),
        "Unknown origin_source '%s', falling back to 'params'",
        origin_source_.c_str());
      origin_source_ = "params";
    }
    if (yaw_mode_ != "path" && yaw_mode_ != "msg") {
      RCLCPP_WARN(
        this->get_logger(),
        "Unknown yaw_mode '%s', falling back to 'path'",
        yaw_mode_.c_str());
      yaw_mode_ = "path";
    }
    if (min_goal_velocity_ < 0.0) {
      min_goal_velocity_ = 0.0;
    }
    if (max_goal_velocity_ < min_goal_velocity_) {
      max_goal_velocity_ = min_goal_velocity_;
    }
    if (status_rate_hz_ < 1.0) {
      status_rate_hz_ = 1.0;
    }
  }

  void goalCallback(const msg_set_msgs::msg::MultiGoal::SharedPtr msg)
  {
    if (!msg) {
      return;
    }

    if (msg->ctl_mode != 0 && !ctl_mode_warned_) {
      RCLCPP_WARN(
        this->get_logger(),
        "MultiGoal.ctl_mode=%d is not directly supported by Nav2; "
        "treating it as velocity + yaw-rate mode",
        static_cast<int>(msg->ctl_mode));
      ctl_mode_warned_ = true;
    }

    if (msg->multi_goal_points.empty()) {
      rejectNewGoal("MultiGoal contains no waypoints");
      return;
    }

    if (msg->is_gps_aid && origin_check_enabled_ && !originConsistent()) {
      rejectNewGoal("GPS origin consistency check failed");
      return;
    }

    WaypointPlan plan;
    std::string error;
    if (!convertMultiGoal(
        *msg, origin_, current_pose_, world_frame_, yaw_mode_, plan, error))
    {
      rejectNewGoal(error);
      return;
    }

    if (max_goal_distance_m_ > 0.0 && current_pose_.valid) {
      for (size_t i = 0; i < plan.poses.size(); ++i) {
        const double dx = plan.poses[i].pose.position.x - current_pose_.x;
        const double dy = plan.poses[i].pose.position.y - current_pose_.y;
        if (std::hypot(dx, dy) > max_goal_distance_m_) {
          rejectNewGoal(
            "Waypoint " + std::to_string(i) +
            " is farther than max_goal_distance_m; check GPS origin");
          return;
        }
      }
    }

    ++generation_;
    pending_plan_ = std::move(plan);
    pending_since_ = this->now();
    next_goal_send_time_ = 0.0;
    first_feedback_ = true;
    current_waypoint_ = 0;

    if (active_goal_ || canceling_) {
      RCLCPP_INFO(
        this->get_logger(),
        "Received new MultiGoal while a task is active; preempting current task");
      setStatus(Status::INITIALIZING);
      if (canceling_ && cancel_reason_ == CancelReason::USER) {
        // 新目标优先于尚未完成的用户取消
        cancel_reason_ = CancelReason::PREEMPT;
      }
      if (active_goal_ && !canceling_) {
        requestCancel(CancelReason::PREEMPT);
      }
    } else {
      setStatus(Status::INITIALIZING);
    }

    RCLCPP_INFO(
      this->get_logger(), "Received %zu waypoints (GPS aid: %s)",
      msg->multi_goal_points.size(), msg->is_gps_aid ? "true" : "false");
  }

  void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    if (!msg) {
      return;
    }
    current_pose_.x = msg->pose.pose.position.x;
    current_pose_.y = msg->pose.pose.position.y;
    tf2::Quaternion orientation(
      msg->pose.pose.orientation.x,
      msg->pose.pose.orientation.y,
      msg->pose.pose.orientation.z,
      msg->pose.pose.orientation.w);
    current_pose_.yaw = tf2::getYaw(orientation);
    current_pose_.valid = true;
    last_odom_time_ = this->now();
    odom_received_ = true;
  }

  void gpsCallback(const sensor_msgs::msg::NavSatFix::SharedPtr msg)
  {
    if (!msg) {
      return;
    }
    latest_gps_lat_ = msg->latitude;
    latest_gps_lon_ = msg->longitude;
    latest_gps_alt_ = msg->altitude;
    last_gps_time_ = this->now();
    gps_received_ = true;
  }

  void originCallback(const geographic_msgs::msg::GeoPointStamped::SharedPtr msg)
  {
    if (!msg || origin_source_ != "topic") {
      return;
    }
    origin_lat_ = msg->position.latitude;
    origin_lon_ = msg->position.longitude;
    origin_alt_ = msg->position.altitude;
    origin_.setOrigin(origin_lat_, origin_lon_, origin_alt_);
    RCLCPP_INFO(
      this->get_logger(),
      "GPS origin updated from topic: lat=%.6f, lon=%.6f, alt=%.2f",
      origin_lat_, origin_lon_, origin_alt_);
  }

  void cancelCallback(
    const std::shared_ptr<std_srvs::srv::Trigger::Request> /*request*/,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response)
  {
    pending_plan_.reset();
    ++generation_;

    if (active_goal_) {
      if (canceling_) {
        cancel_reason_ = CancelReason::USER;
      } else {
        requestCancel(CancelReason::USER);
      }
    } else {
      setStatus(Status::WAITING_FOR_GOALS);
    }

    response->success = true;
    response->message = "Navigation cancellation requested";
    RCLCPP_INFO(this->get_logger(), "Cancel service requested");
  }

  void statusTimer()
  {
    const auto now = this->now();
    if (status_ == Status::GOAL_REACHED &&
      (now - status_changed_time_).seconds() > status_goal_reached_hold_sec_)
    {
      setStatus(Status::EXECUTING_PATH);
    } else if (status_ == Status::COMPLETED &&
      (now - status_changed_time_).seconds() > status_completed_hold_sec_)
    {
      setStatus(Status::WAITING_FOR_GOALS);
    }

    std_msgs::msg::String message;
    message.data = statusString(status_);
    status_pub_->publish(message);
  }

  void workTimer()
  {
    const auto now = this->now();

    if (canceling_ && (now - cancel_started_).seconds() > cancel_timeout_sec_) {
      RCLCPP_WARN(
        this->get_logger(),
        "FollowWaypoints cancel did not complete within %.1f s; clearing active goal",
        cancel_timeout_sec_);
      finishCancel();
    }

    if (active_goal_) {
      if (task_timeout_sec_ > 0.0 &&
        (now - goal_accepted_time_).seconds() > task_timeout_sec_)
      {
        RCLCPP_WARN(
          this->get_logger(), "Navigation task timed out after %.1f s",
          task_timeout_sec_);
        requestCancel(CancelReason::TIMEOUT);
      } else if (stall_timeout_sec_ > 0.0 &&
        (now - last_feedback_time_).seconds() > stall_timeout_sec_)
      {
        RCLCPP_WARN(
          this->get_logger(), "No FollowWaypoints feedback for %.1f s",
          stall_timeout_sec_);
        requestCancel(CancelReason::STALL);
      }
    }

    if (goal_response_pending_ &&
      (now - pending_since_).seconds() > action_server_wait_timeout_sec_)
    {
      RCLCPP_WARN(
        this->get_logger(),
        "Timed out waiting for FollowWaypoints goal response; sending latest goal");
      goal_response_pending_ = false;
    }

    if (pending_plan_ && !active_goal_ && !canceling_ && !goal_response_pending_) {
      sendPendingGoal();
    }
  }

  void sendPendingGoal()
  {
    if (!pending_plan_ || this->now().seconds() < next_goal_send_time_) {
      return;
    }

    if (!follow_waypoints_client_->action_server_is_ready()) {
      if ((this->now() - pending_since_).seconds() > action_server_wait_timeout_sec_) {
        fail("FollowWaypoints action server is not available");
      }
      return;
    }

    if (require_tf_before_goal_ && !tfReady()) {
      if ((this->now() - pending_since_).seconds() > tf_wait_timeout_sec_) {
        fail(
          "TF " + world_frame_ + " -> " + robot_base_frame_ +
          " is not available; refusing to send goals");
      }
      return;
    }

    FollowWaypoints::Goal goal;
    goal.poses = pending_plan_->poses;
    const auto stamp = this->now();
    for (auto & pose : goal.poses) {
      pose.header.stamp = stamp;
    }

    const auto generation = generation_;
    rclcpp_action::Client<FollowWaypoints>::SendGoalOptions options;
    options.goal_response_callback =
      [this, generation](const GoalHandle::SharedPtr & goal_handle) {
        onGoalResponse(generation, goal_handle);
      };
    options.feedback_callback =
      [this, generation](
      GoalHandle::SharedPtr,
      const std::shared_ptr<const FollowWaypoints::Feedback> feedback) {
        onFeedback(generation, feedback);
      };
    options.result_callback =
      [this, generation](const GoalHandle::WrappedResult & result) {
        onResult(generation, result);
      };

    goal_response_pending_ = true;
    try {
      follow_waypoints_client_->async_send_goal(goal, options);
    } catch (const std::exception & exception) {
      goal_response_pending_ = false;
      fail(std::string("Failed to send FollowWaypoints goal: ") + exception.what());
      return;
    }

    RCLCPP_INFO(
      this->get_logger(), "Sent FollowWaypoints goal with %zu poses",
      goal.poses.size());
  }

  void onGoalResponse(
    uint64_t generation, const GoalHandle::SharedPtr & goal_handle)
  {
    goal_response_pending_ = false;

    if (!goal_handle) {
      if (generation == generation_) {
        if ((this->now() - pending_since_).seconds() >= action_server_wait_timeout_sec_) {
          fail("FollowWaypoints goal was rejected by the action server");
        } else {
          // waypoint_follower 的 action server 在 configure 阶段就存在，但未
          // activate 时会拒绝目标；等待 lifecycle 完成后再重试。
          RCLCPP_WARN_THROTTLE(
            this->get_logger(), *this->get_clock(), 1000,
            "FollowWaypoints goal rejected while server is still activating; retrying");
          next_goal_send_time_ = this->now().seconds() + goal_retry_interval_sec_;
        }
      }
      return;
    }

    if (generation != generation_) {
      // 旧目标在收到响应前已被新目标替代；直接取消它，不覆盖当前任务状态。
      RCLCPP_INFO(
        this->get_logger(),
        "Canceling stale FollowWaypoints goal accepted after preemption");
      try {
        follow_waypoints_client_->async_cancel_goal(
          goal_handle,
          [this](auto /*response*/) {
            RCLCPP_INFO(this->get_logger(), "Stale FollowWaypoints goal canceled");
          });
      } catch (const std::exception & exception) {
        RCLCPP_WARN(
          this->get_logger(), "Failed to cancel stale goal: %s", exception.what());
      }
      return;
    }

    active_goal_ = goal_handle;
    if (pending_plan_) {
      active_plan_ = std::move(*pending_plan_);
      pending_plan_.reset();
    }
    current_waypoint_ = 0;
    first_feedback_ = true;
    goal_accepted_time_ = this->now();
    last_feedback_time_ = goal_accepted_time_;
    setStatus(Status::INITIALIZING);

    if (respect_waypoint_velocity_) {
      publishSpeedLimitForIndex(0);
    }

    RCLCPP_INFO(this->get_logger(), "FollowWaypoints goal accepted");
  }

  void onFeedback(
    uint64_t generation,
    const std::shared_ptr<const FollowWaypoints::Feedback> feedback)
  {
    if (generation != generation_ || !feedback || !active_goal_ || canceling_ ||
      status_ == Status::COMPLETED || status_ == Status::FAILED)
    {
      return;
    }

    last_feedback_time_ = this->now();

    if (first_feedback_) {
      first_feedback_ = false;
      current_waypoint_ = feedback->current_waypoint;
      setStatus(Status::EXECUTING_PATH);
      publishSpeedLimitForIndex(current_waypoint_);
      return;
    }

    if (feedback->current_waypoint != current_waypoint_) {
      current_waypoint_ = feedback->current_waypoint;
      setStatus(Status::GOAL_REACHED);
      publishSpeedLimitForIndex(current_waypoint_);
    } else if (status_ != Status::GOAL_REACHED) {
      setStatus(Status::EXECUTING_PATH);
    }
  }

  void onResult(
    uint64_t generation, const GoalHandle::WrappedResult & result)
  {
    if (generation != generation_) {
      return;
    }

    active_goal_.reset();
    active_plan_.reset();
    goal_response_pending_ = false;

    switch (result.code) {
      case rclcpp_action::ResultCode::SUCCEEDED:
        if (result.result && !result.result->missed_waypoints.empty()) {
          RCLCPP_WARN(
            this->get_logger(), "FollowWaypoints succeeded with %zu missed waypoints",
            result.result->missed_waypoints.size());
          setStatus(Status::FAILED);
        } else {
          RCLCPP_INFO(this->get_logger(), "All waypoints completed");
          setStatus(Status::COMPLETED);
        }
        break;
      case rclcpp_action::ResultCode::ABORTED:
        if (result.result) {
          RCLCPP_ERROR(
            this->get_logger(), "FollowWaypoints aborted with %zu missed waypoints",
            result.result->missed_waypoints.size());
        } else {
          RCLCPP_ERROR(this->get_logger(), "FollowWaypoints aborted");
        }
        setStatus(Status::FAILED);
        break;
      case rclcpp_action::ResultCode::CANCELED:
        if (!canceling_) {
          setStatus(Status::WAITING_FOR_GOALS);
        }
        break;
      default:
        RCLCPP_ERROR(this->get_logger(), "FollowWaypoints returned unknown result code");
        setStatus(Status::FAILED);
        break;
    }

    if (respect_waypoint_velocity_) {
      publishSpeedLimit(0.0);
    }

    if (pending_plan_ && !canceling_) {
      setStatus(Status::INITIALIZING);
    }
  }

  void requestCancel(CancelReason reason)
  {
    if (!active_goal_ || canceling_) {
      return;
    }

    if (reason == CancelReason::TIMEOUT || reason == CancelReason::STALL) {
      ++generation_;
    }

    canceling_ = true;
    cancel_reason_ = reason;
    cancel_started_ = this->now();

    const char * reason_text = "unknown";
    switch (reason) {
      case CancelReason::PREEMPT:
        reason_text = "preempted by new goal";
        break;
      case CancelReason::USER:
        reason_text = "user cancel";
        break;
      case CancelReason::TIMEOUT:
        reason_text = "task timeout";
        break;
      case CancelReason::STALL:
        reason_text = "feedback stall";
        break;
      default:
        break;
    }
    RCLCPP_INFO(this->get_logger(), "Canceling FollowWaypoints goal: %s", reason_text);

    try {
      follow_waypoints_client_->async_cancel_goal(
        active_goal_,
        [this](auto /*response*/) {
          finishCancel();
        });
    } catch (const std::exception & exception) {
      RCLCPP_WARN(
        this->get_logger(), "Failed to request goal cancellation: %s", exception.what());
      finishCancel();
    }
  }

  void finishCancel()
  {
    if (!canceling_) {
      return;
    }

    canceling_ = false;
    active_goal_.reset();
    active_plan_.reset();

    const auto reason = cancel_reason_;
    cancel_reason_ = CancelReason::NONE;

    if (respect_waypoint_velocity_) {
      publishSpeedLimit(0.0);
    }

    switch (reason) {
      case CancelReason::PREEMPT:
        if (pending_plan_) {
          setStatus(Status::INITIALIZING);
        } else {
          setStatus(Status::WAITING_FOR_GOALS);
        }
        break;
      case CancelReason::USER:
        pending_plan_.reset();
        setStatus(Status::WAITING_FOR_GOALS);
        break;
      case CancelReason::TIMEOUT:
      case CancelReason::STALL:
        pending_plan_.reset();
        setStatus(Status::FAILED);
        break;
      default:
        setStatus(Status::WAITING_FOR_GOALS);
        break;
    }
  }

  void rejectNewGoal(const std::string & error)
  {
    if (active_goal_ || canceling_) {
      RCLCPP_ERROR(
        this->get_logger(),
        "Rejecting new MultiGoal (%s); keeping current task", error.c_str());
      return;
    }

    fail(error);
  }

  void fail(const std::string & error)
  {
    RCLCPP_ERROR(this->get_logger(), "Navigation failed: %s", error.c_str());
    pending_plan_.reset();
    setStatus(Status::FAILED);
    if (respect_waypoint_velocity_) {
      publishSpeedLimit(0.0);
    }
  }

  bool originConsistent()
  {
    if (!origin_check_enabled_) {
      return true;
    }
    if (!gps_received_ || !odom_received_) {
      RCLCPP_WARN_THROTTLE(
        this->get_logger(), *this->get_clock(), 5000,
        "GPS origin check skipped: waiting for /gps/data and /world_odom");
      return true;
    }

    const auto now = this->now();
    if ((now - last_gps_time_).seconds() > origin_check_max_age_sec_ ||
      (now - last_odom_time_).seconds() > origin_check_max_age_sec_)
    {
      RCLCPP_WARN_THROTTLE(
        this->get_logger(), *this->get_clock(), 5000,
        "GPS origin check skipped: GPS or odometry data is stale");
      return true;
    }

    double x = 0.0;
    double y = 0.0;
    double z = 0.0;
    origin_.forward(latest_gps_lat_, latest_gps_lon_, latest_gps_alt_, x, y, z);
    const double distance = std::hypot(x - current_pose_.x, y - current_pose_.y);
    if (distance > origin_check_tolerance_) {
      RCLCPP_ERROR(
        this->get_logger(),
        "GPS origin mismatch: converted GPS position is %.2f m from /world_odom; "
        "check origin_lat/lon/alt against gps_imu_fusion",
        distance);
      return false;
    }
    return true;
  }

  bool tfReady() const
  {
    if (!require_tf_before_goal_) {
      return true;
    }
    return tf_buffer_->canTransform(
      world_frame_, robot_base_frame_, tf2::TimePointZero);
  }

  void publishSpeedLimitForIndex(size_t index)
  {
    if (!respect_waypoint_velocity_ || !active_plan_ ||
      index >= active_plan_->velocities.size())
    {
      return;
    }

    double velocity = active_plan_->velocities[index];
    if (!std::isfinite(velocity)) {
      return;
    }
    velocity = std::clamp(velocity, min_goal_velocity_, max_goal_velocity_);
    publishSpeedLimit(velocity);
    RCLCPP_INFO(
      this->get_logger(), "Waypoint %zu speed limit: %.2f m/s", index, velocity);
  }

  void publishSpeedLimit(double velocity)
  {
    nav2_msgs::msg::SpeedLimit message;
    message.percentage = false;
    message.speed_limit = velocity;
    speed_limit_pub_->publish(message);
  }

  void setStatus(Status status)
  {
    if (status_ != status) {
      status_ = status;
      status_changed_time_ = this->now();
      RCLCPP_INFO(
        this->get_logger(), "Navigation status: %s", statusString(status_).c_str());
    }
  }

  static std::string statusString(Status status)
  {
    switch (status) {
      case Status::WAITING_FOR_GOALS:
        return "WAITING_FOR_GOALS";
      case Status::INITIALIZING:
        return "INITIALIZING";
      case Status::EXECUTING_PATH:
        return "EXECUTING_PATH";
      case Status::GOAL_REACHED:
        return "GOAL_REACHED";
      case Status::COMPLETED:
        return "COMPLETED";
      case Status::FAILED:
        return "FAILED";
      default:
        return "UNKNOWN";
    }
  }

  // 参数
  double origin_lat_{0.0};
  double origin_lon_{0.0};
  double origin_alt_{0.0};
  std::string origin_source_;
  std::string origin_topic_;

  std::string world_frame_;
  std::string robot_base_frame_;
  std::string odom_topic_;
  std::string gps_topic_;
  std::string multi_goal_topic_;
  std::string follow_waypoints_action_;
  std::string nav_status_topic_;
  std::string speed_limit_topic_;
  std::string cancel_service_;

  std::string yaw_mode_;
  bool respect_waypoint_velocity_{true};
  double min_goal_velocity_{0.1};
  double max_goal_velocity_{0.6};

  double action_server_wait_timeout_sec_{5.0};
  double goal_retry_interval_sec_{0.5};
  double cancel_timeout_sec_{3.0};
  double stall_timeout_sec_{30.0};
  double task_timeout_sec_{0.0};
  bool require_tf_before_goal_{true};
  double tf_wait_timeout_sec_{5.0};

  bool origin_check_enabled_{true};
  double origin_check_tolerance_{5.0};
  double origin_check_max_age_sec_{1.0};
  double max_goal_distance_m_{5000.0};

  double status_rate_hz_{10.0};
  double status_completed_hold_sec_{2.0};
  double status_goal_reached_hold_sec_{0.3};

  // 坐标与数据
  GeoOrigin origin_;
  Pose2D current_pose_;
  double latest_gps_lat_{0.0};
  double latest_gps_lon_{0.0};
  double latest_gps_alt_{0.0};
  bool gps_received_{false};
  bool odom_received_{false};
  rclcpp::Time last_gps_time_{0, 0, RCL_ROS_TIME};
  rclcpp::Time last_odom_time_{0, 0, RCL_ROS_TIME};

  // 任务状态
  Status status_{Status::WAITING_FOR_GOALS};
  rclcpp::Time status_changed_time_{0, 0, RCL_ROS_TIME};
  uint64_t generation_{0};
  std::optional<WaypointPlan> active_plan_;
  std::optional<WaypointPlan> pending_plan_;
  rclcpp::Time pending_since_{0, 0, RCL_ROS_TIME};
  double next_goal_send_time_{0.0};
  GoalHandle::SharedPtr active_goal_;
  bool goal_response_pending_{false};
  bool canceling_{false};
  CancelReason cancel_reason_{CancelReason::NONE};
  rclcpp::Time cancel_started_{0, 0, RCL_ROS_TIME};
  rclcpp::Time goal_accepted_time_{0, 0, RCL_ROS_TIME};
  rclcpp::Time last_feedback_time_{0, 0, RCL_ROS_TIME};
  uint32_t current_waypoint_{0};
  bool first_feedback_{true};
  bool ctl_mode_warned_{false};

  // ROS 接口
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  rclcpp::Subscription<msg_set_msgs::msg::MultiGoal>::SharedPtr goal_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr gps_sub_;
  rclcpp::Subscription<geographic_msgs::msg::GeoPointStamped>::SharedPtr origin_sub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_pub_;
  rclcpp::Publisher<nav2_msgs::msg::SpeedLimit>::SharedPtr speed_limit_pub_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr cancel_srv_;
  rclcpp_action::Client<FollowWaypoints>::SharedPtr follow_waypoints_client_;
  rclcpp::TimerBase::SharedPtr status_timer_;
  rclcpp::TimerBase::SharedPtr work_timer_;
};

}  // namespace robotdog_nav2_bringup

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<robotdog_nav2_bringup::MultiGoalBridgeNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
