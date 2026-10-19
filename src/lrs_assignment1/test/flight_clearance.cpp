// Copyright (c) 2026 STU FEI URK
// SPDX-License-Identifier: MIT
// Audit an indoor CSV exported by analyze_bag.py. This is a map-clearance test,
// not a claim of ground-truth collision detection from Gazebo contacts.
#include "lrs_assignment1/flight_path_guard.hpp"
#include "lrs_assignment1/mission_file.hpp"
#include <iostream>
#include <sstream>
int main(int argc,char **argv) {
  try {
    if(argc!=4) throw std::runtime_error("Usage: flight_clearance map.pcd exported.csv radius");
    VoxelMap map(0.25f,float(mission::number(argv[3])));
    if(!map.load(argv[1])) return 1;
    mission::FlightPathGuard guard(map);
    std::ifstream file(argv[2]); if(!file) throw std::runtime_error("Cannot open track CSV");
    std::string line; std::getline(file,line);
    size_t samples=0,blocked=0,segments=0; bool previous_valid=false; Point3D previous{};
    while(std::getline(file,line)) {
      std::vector<std::string> f; std::stringstream input(line);std::string field;
      while(std::getline(input,field,',')) f.push_back(field);
      if(f.size()<5) continue;
      const bool flight=f[1]=="NAVIGATE" || f[1]=="ALIGN" || f[1]=="TASK";
      if(!flight) {previous_valid=false;continue;}
      // Default hangar map->ENU yaw is +pi/2 and launch map XY is (13,7).
      Point3D p{float(13+mission::number(f[3])),float(7-mission::number(f[2])),float(mission::number(f[4]))};
      ++samples;
      if(map.isBlocked(p.x,p.y,p.z)) ++blocked;
      if(previous_valid && !guard.segmentFree(previous,p)) ++segments;
      previous=p; previous_valid=true;
    }
    std::cout<<"Airborne navigation/hold samples="<<samples<<", blocked samples="<<blocked
             <<", blocked segments="<<segments<<", inflation="<<argv[3]<<" m\n";
    return samples>0 && blocked==0 && segments==0?0:2;
  } catch(const std::exception &e) {std::cerr<<e.what()<<'\n';return 1;}
}
