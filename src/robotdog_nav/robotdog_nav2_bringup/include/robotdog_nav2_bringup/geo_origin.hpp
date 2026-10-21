#ifndef ROBOTDOG_NAV2_BRINGUP__GEO_ORIGIN_HPP_
#define ROBOTDOG_NAV2_BRINGUP__GEO_ORIGIN_HPP_

#include <memory>
#include <mutex>

#include <GeographicLib/LocalCartesian.hpp>

namespace robotdog_nav2_bringup
{

/// GPS 原点与 ENU 局部坐标转换封装。
/// 与 gps_imu_fusion / waypoint_nav 使用相同的 GeographicLib 语义：
/// x = East, y = North, z = Up。
class GeoOrigin
{
public:
  struct Origin
  {
    double lat{0.0};
    double lon{0.0};
    double alt{0.0};
  };

  explicit GeoOrigin(double lat = 0.0, double lon = 0.0, double alt = 0.0);

  void setOrigin(double lat, double lon, double alt);

  Origin getOrigin() const;

  void forward(
    double lat, double lon, double alt,
    double & x, double & y, double & z) const;

  void reverse(
    double x, double y, double z,
    double & lat, double & lon, double & alt) const;

private:
  mutable std::mutex mutex_;
  std::unique_ptr<GeographicLib::LocalCartesian> converter_;
  double origin_lat_{0.0};
  double origin_lon_{0.0};
  double origin_alt_{0.0};
};

}  // namespace robotdog_nav2_bringup

#endif  // ROBOTDOG_NAV2_BRINGUP__GEO_ORIGIN_HPP_
