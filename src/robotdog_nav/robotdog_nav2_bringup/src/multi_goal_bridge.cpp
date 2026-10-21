#include "robotdog_nav2_bringup/multi_goal_bridge.hpp"

#include <cmath>
#include <utility>
#include <vector>

#include <tf2/LinearMath/Quaternion.h>
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

namespace robotdog_nav2_bringup
{

bool convertMultiGoal(
  const msg_set_msgs::msg::MultiGoal & msg,
  const GeoOrigin & origin,
  const Pose2D & current_pose,
  const std::string & world_frame,
  const std::string & yaw_mode,
  WaypointPlan & plan,
  std::string & error)
{
  plan = WaypointPlan();
  plan.is_gps_aid = msg.is_gps_aid;

  const auto & points = msg.multi_goal_points;
  if (points.empty()) {
    error = "MultiGoal contains no waypoints";
    return false;
  }

  const auto origin_alt = origin.getOrigin().alt;
  std::vector<double> xs;
  std::vector<double> ys;
  std::vector<double> zs;
  xs.reserve(points.size());
  ys.reserve(points.size());
  zs.reserve(points.size());

  for (size_t i = 0; i < points.size(); ++i) {
    const auto & wp = points[i];
    double x = 0.0;
    double y = 0.0;
    double z = 0.0;

    if (msg.is_gps_aid) {
      if (!std::isfinite(wp.x_or_lat) || !std::isfinite(wp.y_or_lon) ||
        !std::isfinite(wp.z_or_alt))
      {
        error = "Waypoint " + std::to_string(i) + " contains non-finite GPS coordinates";
        return false;
      }

      if (msg.is_gps_hgt) {
        origin.forward(wp.x_or_lat, wp.y_or_lon, wp.z_or_alt, x, y, z);
      } else {
        double unused_z = 0.0;
        origin.forward(wp.x_or_lat, wp.y_or_lon, origin_alt, x, y, unused_z);
        z = wp.z_or_alt;
      }
    } else {
      x = wp.x_or_lat;
      y = wp.y_or_lon;
      z = wp.z_or_alt;
    }

    if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(z)) {
      error = "Waypoint " + std::to_string(i) + " converted to non-finite world coordinates";
      return false;
    }

    xs.push_back(x);
    ys.push_back(y);
    zs.push_back(z);
  }

  plan.poses.reserve(points.size());
  plan.velocities.reserve(points.size());

  for (size_t i = 0; i < points.size(); ++i) {
    const auto & wp = points[i];
    double yaw = 0.0;

    if (yaw_mode == "msg") {
      yaw = wp.yaw;
    } else {
      double dx = 0.0;
      double dy = 0.0;
      if (i + 1 < points.size()) {
        dx = xs[i + 1] - xs[i];
        dy = ys[i + 1] - ys[i];
      } else if (points.size() > 1) {
        dx = xs[i] - xs[i - 1];
        dy = ys[i] - ys[i - 1];
      } else if (current_pose.valid) {
        dx = xs[i] - current_pose.x;
        dy = ys[i] - current_pose.y;
      }

      if (std::hypot(dx, dy) > 1e-9) {
        yaw = std::atan2(dy, dx);
      } else if (current_pose.valid) {
        yaw = current_pose.yaw;
      }
    }

    if (!std::isfinite(yaw)) {
      error = "Waypoint " + std::to_string(i) + " has non-finite yaw";
      return false;
    }

    geometry_msgs::msg::PoseStamped pose;
    pose.header.frame_id = world_frame;
    pose.pose.position.x = xs[i];
    pose.pose.position.y = ys[i];
    pose.pose.position.z = zs[i];

    tf2::Quaternion orientation;
    orientation.setRPY(0.0, 0.0, yaw);
    pose.pose.orientation = tf2::toMsg(orientation);

    plan.poses.push_back(std::move(pose));
    plan.velocities.push_back(wp.vel);
  }

  return true;
}

}  // namespace robotdog_nav2_bringup
