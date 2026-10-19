// Copyright (c) 2026 STU FEI URK
// SPDX-License-Identifier: MIT
#pragma once

#include "lrs_assignment1/astar_planner.hpp"
#include <algorithm>
#include <array>
#include <cmath>
#include <stdexcept>
#include <vector>

namespace mission {

// A1.2 validates routes at the flight boundary without modifying the A1.1
// planner. Partition each segment at ALL voxel faces, then check each open
// interval and each face crossing. Face/edge/corner contact checks every
// adjacent cell (a conservative supercover), so a short intersection cannot
// disappear between fixed-distance samples. The referenced map must stay
// loaded and unchanged for this object's lifetime.
class FlightPathGuard {
public:
  explicit FlightPathGuard(const VoxelMap &map):map_(map),size_(map.getVoxelSize()) {
    float x{},y{},z{};
    if(!std::isfinite(size_) || size_<=0 || !map.gridToWorld(0,0,0,x,y,z))
      throw std::invalid_argument("Flight path guard requires a loaded voxel map");
    origin_={x-size_/2,y-size_/2,z-size_/2};
  }

  bool segmentFree(const Point3D &a,const Point3D &b) const {
    if(!finite(a) || !finite(b) || !map_.isInside(a.x,a.y,a.z) ||
       !map_.isInside(b.x,b.y,b.z)) return false;
    const std::array<double,3> start{a.x,a.y,a.z},end{b.x,b.y,b.z};
    std::vector<double> crossings{0,1};
    for(std::size_t axis=0;axis<3;++axis) {
      const double delta=end[axis]-start[axis];
      if(delta==0) continue;
      const int first=static_cast<int>(std::floor((std::min(start[axis],end[axis])-origin_[axis])/size_));
      const int last=static_cast<int>(std::ceil((std::max(start[axis],end[axis])-origin_[axis])/size_));
      for(int boundary=first;boundary<=last;++boundary) {
        const double t=(origin_[axis]+boundary*size_-start[axis])/delta;
        if(t>0 && t<1) crossings.push_back(t);
      }
    }
    std::sort(crossings.begin(),crossings.end());
    const auto check=[&](double t) {
      return pointFree({start[0]+t*(end[0]-start[0]),
                        start[1]+t*(end[1]-start[1]),
                        start[2]+t*(end[2]-start[2])});
    };
    for(std::size_t i=0;i<crossings.size();++i) {
      if(!check(crossings[i])) return false;
      if(i && crossings[i]>crossings[i-1] && !check((crossings[i]+crossings[i-1])/2)) return false;
    }
    return true;
  }

  bool pathFree(const std::vector<Point3D> &path) const {
    if(path.empty()) return false;
    if(path.size()==1) return segmentFree(path.front(),path.front());
    for(std::size_t i=1;i<path.size();++i)
      if(!segmentFree(path[i-1],path[i])) return false;
    return true;
  }

private:
  static bool finite(const Point3D &p) {
    return std::isfinite(p.x) && std::isfinite(p.y) && std::isfinite(p.z);
  }

  bool pointFree(const std::array<double,3> &point) const {
    std::array<std::array<int,2>,3> cells{};
    std::array<int,3> counts{1,1,1};
    for(std::size_t axis=0;axis<3;++axis) {
      const double coordinate=(point[axis]-origin_[axis])/size_;
      const double boundary=std::round(coordinate);
      // The A1.1 map uses float coordinates. Account for their rounding at
      // exact voxel boundaries (2.5 micrometres with the 0.25 m grid).
      if(std::abs(coordinate-boundary)<=1e-5) {
        cells[axis]={static_cast<int>(boundary)-1,static_cast<int>(boundary)};
        counts[axis]=2;
      } else {
        cells[axis][0]=static_cast<int>(std::floor(coordinate));
      }
    }
    for(int x=0;x<counts[0];++x)
      for(int y=0;y<counts[1];++y)
        for(int z=0;z<counts[2];++z)
          if(map_.isBlockedGrid(cells[0][x],cells[1][y],cells[2][z])) return false;
    return true;
  }

  const VoxelMap &map_;
  double size_;
  std::array<double,3> origin_;
};
} // namespace mission
