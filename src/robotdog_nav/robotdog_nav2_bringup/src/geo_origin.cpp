#include "robotdog_nav2_bringup/geo_origin.hpp"

namespace robotdog_nav2_bringup
{

GeoOrigin::GeoOrigin(double lat, double lon, double alt)
: converter_(std::make_unique<GeographicLib::LocalCartesian>(lat, lon, alt)),
  origin_lat_(lat),
  origin_lon_(lon),
  origin_alt_(alt)
{
}

void GeoOrigin::setOrigin(double lat, double lon, double alt)
{
  std::lock_guard<std::mutex> lock(mutex_);
  origin_lat_ = lat;
  origin_lon_ = lon;
  origin_alt_ = alt;
  converter_ = std::make_unique<GeographicLib::LocalCartesian>(lat, lon, alt);
}

GeoOrigin::Origin GeoOrigin::getOrigin() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return {origin_lat_, origin_lon_, origin_alt_};
}

void GeoOrigin::forward(
  double lat, double lon, double alt,
  double & x, double & y, double & z) const
{
  std::lock_guard<std::mutex> lock(mutex_);
  converter_->Forward(lat, lon, alt, x, y, z);
}

void GeoOrigin::reverse(
  double x, double y, double z,
  double & lat, double & lon, double & alt) const
{
  std::lock_guard<std::mutex> lock(mutex_);
  converter_->Reverse(x, y, z, lat, lon, alt);
}

}  // namespace robotdog_nav2_bringup
