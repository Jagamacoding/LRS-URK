// Copyright (c) 2026 STU FEI URK
// SPDX-License-Identifier: MIT
#pragma once
#include <cmath>
#include <stdexcept>

namespace mission {
constexpr double pi = 3.14159265358979323846;
struct Vec { double x{}, y{}, z{}; };
inline double distance(Vec a, Vec b) {
  return std::hypot(std::hypot(a.x-b.x, a.y-b.y), a.z-b.z);
}
// ENU angles use (-pi, pi]; unwrap chooses the nearest equivalent angle.
inline double wrap(double a) {
  const double result=std::remainder(a,2*pi);
  return result<=-pi?result+2*pi:result;
}
inline double unwrap(double previous, double next) { return previous + wrap(next-previous); }
inline double compassYaw(double degrees) { return wrap(pi/2-degrees*pi/180); }
// WGS84 curvature radii at the captured origin; adequate for this small field.
class LocalFrame {
  double lat_, lon_, alt_, north_, east_;
public:
  LocalFrame(double lat, double lon, double alt):lat_(lat),lon_(lon),alt_(alt) {
    if (!std::isfinite(lat) || !std::isfinite(lon) || !std::isfinite(alt) ||
        std::abs(lat)>85 || std::abs(lon)>180) throw std::invalid_argument("Invalid GPS origin");
    const double s=std::sin(lat*pi/180), d=1-0.00669437999014*s*s;
    north_=6378137*(1-0.00669437999014)/std::pow(d,1.5)*pi/180;
    east_=6378137/std::sqrt(d)*std::cos(lat*pi/180)*pi/180;
  }
  Vec local(Vec geo) const {
    if(!std::isfinite(geo.x) || !std::isfinite(geo.y) || !std::isfinite(geo.z) ||
       std::abs(geo.x)>90 || std::abs(geo.y)>180)
      throw std::invalid_argument("Invalid geodetic coordinate");
    return {std::remainder(geo.y-lon_,360.0)*east_,(geo.x-lat_)*north_,geo.z-alt_};
  }
  Vec geodetic(Vec p) const {
    if(!std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(p.z))
      throw std::invalid_argument("Invalid local coordinate");
    const Vec result{lat_+p.y/north_,std::remainder(lon_+p.x/east_,360.0),alt_+p.z};
    if(std::abs(result.x)>90) throw std::invalid_argument("Local coordinate exceeds tangent-plane range");
    return result;
  }
};
inline Vec figure(double phase, double amplitude, Vec centre) {
  return {centre.x+amplitude*std::sin(phase),centre.y+amplitude*std::sin(phase)*std::cos(phase),centre.z};
}
inline double tangent(double phase) { return std::atan2(std::cos(2*phase),std::cos(phase)); }
}
