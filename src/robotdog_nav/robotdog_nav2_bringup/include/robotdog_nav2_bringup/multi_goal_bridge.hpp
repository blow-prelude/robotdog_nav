#ifndef ROBOTDOG_NAV2_BRINGUP__MULTI_GOAL_BRIDGE_HPP_
#define ROBOTDOG_NAV2_BRINGUP__MULTI_GOAL_BRIDGE_HPP_

#include <string>
#include <vector>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "msg_set_msgs/msg/multi_goal.hpp"
#include "robotdog_nav2_bringup/geo_origin.hpp"

namespace robotdog_nav2_bringup
{

struct Pose2D
{
  double x{0.0};
  double y{0.0};
  double yaw{0.0};
  bool valid{false};
};

struct WaypointPlan
{
  std::vector<geometry_msgs::msg::PoseStamped> poses;
  std::vector<double> velocities;
  bool is_gps_aid{false};
};

/// 将 MultiGoal 转换为 Nav2 FollowWaypoints 所需的 world 位姿序列。
/// 返回 false 时 error 中包含可直接打印的原因。
bool convertMultiGoal(
  const msg_set_msgs::msg::MultiGoal & msg,
  const GeoOrigin & origin,
  const Pose2D & current_pose,
  const std::string & world_frame,
  const std::string & yaw_mode,
  WaypointPlan & plan,
  std::string & error);

}  // namespace robotdog_nav2_bringup

#endif  // ROBOTDOG_NAV2_BRINGUP__MULTI_GOAL_BRIDGE_HPP_
