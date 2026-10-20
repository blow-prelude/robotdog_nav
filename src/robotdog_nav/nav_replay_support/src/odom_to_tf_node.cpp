// 里程计 → TF 适配节点：把 nav_msgs/Odometry 按消息自身时间戳广播为 TF。
// 回放 rosbag（仅雷达 + 里程计）时代替 gps_imu_fusion 提供 world→base_link TF，
// 供下游导航栈（Nav2 costmap）查询机器人位姿；仿真/实机时不启动本节点。
#include <cmath>
#include <memory>
#include <string>

#include "geometry_msgs/msg/transform_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2_ros/transform_broadcaster.h"

class OdomToTfNode : public rclcpp::Node
{
public:
  OdomToTfNode() : Node("odom_to_tf_node")
  {
    // 参数：话题名与坐标系覆盖（空串表示使用消息自带的 frame）
    odom_topic_ = this->declare_parameter<std::string>("odom_topic", "/world_odom");
    parent_frame_ = this->declare_parameter<std::string>("parent_frame", "");
    child_frame_ = this->declare_parameter<std::string>("child_frame", "");

    sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
      odom_topic_, 10,
      std::bind(&OdomToTfNode::odomCallback, this, std::placeholders::_1));

    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

    RCLCPP_INFO(this->get_logger(),
                "odom_to_tf_node started: topic=%s, parent_frame='%s', child_frame='%s' (empty means use message frames)",
                odom_topic_.c_str(), parent_frame_.c_str(), child_frame_.c_str());
  }

private:
  void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    const std::string parent = parent_frame_.empty() ? msg->header.frame_id : parent_frame_;
    const std::string child = child_frame_.empty() ? msg->child_frame_id : child_frame_;

    // 边界校验：空 frame、父子同名、零时间戳、非法位姿一律丢弃
    if (parent.empty() || child.empty()) {
      warnDrop("empty frame id (parent='" + parent + "', child='" + child + "')");
      return;
    }
    if (parent == child) {
      warnDrop("parent frame equals child frame '" + parent + "'");
      return;
    }
    if (msg->header.stamp.sec == 0 && msg->header.stamp.nanosec == 0) {
      warnDrop("zero header stamp");
      return;
    }

    const auto & position = msg->pose.pose.position;
    const auto & quat = msg->pose.pose.orientation;
    if (!std::isfinite(position.x) || !std::isfinite(position.y) || !std::isfinite(position.z) ||
        !std::isfinite(quat.x) || !std::isfinite(quat.y) || !std::isfinite(quat.z) || !std::isfinite(quat.w))
    {
      warnDrop("non-finite pose");
      return;
    }

    const double norm = std::sqrt(quat.x * quat.x + quat.y * quat.y + quat.z * quat.z + quat.w * quat.w);
    if (norm < 1e-6) {
      warnDrop("degenerate quaternion (norm < 1e-6)");
      return;
    }

    // 精确使用消息时间戳（回放时为仿真时间），保证与点云时间对齐；不做前移
    geometry_msgs::msg::TransformStamped t;
    t.header.stamp = msg->header.stamp;
    t.header.frame_id = parent;
    t.child_frame_id = child;
    t.transform.translation.x = position.x;
    t.transform.translation.y = position.y;
    t.transform.translation.z = position.z;
    t.transform.rotation.x = quat.x / norm;
    t.transform.rotation.y = quat.y / norm;
    t.transform.rotation.z = quat.z / norm;
    t.transform.rotation.w = quat.w / norm;

    tf_broadcaster_->sendTransform(t);

    if (published_count_++ == 0) {
      RCLCPP_INFO(this->get_logger(), "First TF broadcast: %s -> %s", parent.c_str(), child.c_str());
    }
    RCLCPP_DEBUG_THROTTLE(this->get_logger(), *(this->get_clock()), 2000,
                          "TF %s -> %s published (%lu total)", parent.c_str(), child.c_str(), published_count_);
  }

  // 限流告警：同类异常只周期性输出，避免刷屏
  void warnDrop(const std::string & detail)
  {
    dropped_count_++;
    RCLCPP_WARN_THROTTLE(this->get_logger(), *(this->get_clock()), 5000,
                         "Odometry message dropped (%lu total): %s", dropped_count_, detail.c_str());
  }

  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

  std::string odom_topic_;
  std::string parent_frame_;
  std::string child_frame_;

  uint64_t published_count_{0};
  uint64_t dropped_count_{0};
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<OdomToTfNode>());
  rclcpp::shutdown();
  return 0;
}
