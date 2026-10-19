// Copyright (c) 2026 STU FEI URK
// SPDX-License-Identifier: MIT
#include "lrs_assignment1/flight_path_guard.hpp"
#include <filesystem>
#include <fstream>
#include <iostream>
#include <limits>
#include <unistd.h>

namespace {
void check(bool condition,const char *message) {
  if(!condition) throw std::runtime_error(message);
}
struct Scratch {
  std::filesystem::path path=std::filesystem::temp_directory_path()/("lrs-path-guard-tests-"+std::to_string(getpid()));
  Scratch() {std::filesystem::create_directories(path);}
  ~Scratch() {std::error_code ignored;std::filesystem::remove_all(path,ignored);}
};
void cloud(const std::filesystem::path &file) {
  std::ofstream out(file);
  out<<"# .PCD v0.7\nVERSION 0.7\nFIELDS x y z\nSIZE 4 4 4\nTYPE F F F\n"
       "COUNT 1 1 1\nWIDTH 3\nHEIGHT 1\nVIEWPOINT 0 0 0 1 0 0 0\nPOINTS 3\nDATA ascii\n"
       "0 0 0\n4 4 4\n1.1 1.1 1.1\n";
}
}

int main(int argc,char **argv) {
  try {
    Scratch scratch; const auto pcd=scratch.path/"obstacle.pcd";cloud(pcd);
    VoxelMap map(1,0);check(map.load(pcd.string()),"Load isolated obstacle fixture");
    mission::FlightPathGuard guard(map);
    check(guard.segmentFree({0.5,0.5,1.5},{3.5,0.5,1.5}),"Clear segment");
    check(!guard.segmentFree({0.5,1.5,1.5},{2.5,1.5,1.5}),"Obstacle crossing");
    check(!guard.segmentFree({0.5,1.5,1.5},{1.5,0.5,1.5}),"Corner contact must cover both adjacent cells");
    check(!guard.segmentFree({0.5,1,1.5},{2.5,1,1.5}),"Segment on an obstacle face");
    check(!guard.segmentFree({0.5,0.5,0.5},{2.5,2.5,2.5}),"Three dimensional diagonal crossing");
    check(!guard.segmentFree({0.5,1.5,1.5},{1.5,0.50001f,1.5}),"Very short obstacle intersection");
    check(guard.segmentFree({0.5,1.5,1.5},{1.5,0.4999f,1.5}),"Nearby clear diagonal retains clearance");
    check(guard.segmentFree({2.2,2.2,2.2},{2.3,2.3,2.3}),"Two different points in the same voxel");
    check(guard.segmentFree({2.2,2.2,2.2},{2.2,2.2,2.2}),"Zero-length free segment");
    check(!guard.segmentFree({1.2,1.2,1.2},{1.2,1.2,1.2}),"Zero-length blocked segment");
    check(!guard.segmentFree({-1,2,2},{2,2,2}),"Out of bounds endpoint");
    check(!guard.segmentFree({std::numeric_limits<float>::quiet_NaN(),2,2},{2,2,2}),"Reject NaN");
    check(!guard.segmentFree({2,2,2},{std::numeric_limits<float>::infinity(),2,2}),"Reject infinity");
    check(!guard.pathFree({}),"Empty route rejected");
    check(guard.pathFree({{2.2,2.2,2.2}}),"Single free waypoint accepted");
    check(!guard.pathFree({{1.2,1.2,1.2}}),"Single blocked waypoint rejected");
    check(guard.pathFree({{0.5,0.5,1.5},{2.5,0.5,1.5},{2.5,2.5,1.5}}),"Clear detour");
    check(!guard.pathFree({{0.5,0.5,1.5},{2.5,2.5,1.5}}),"Unsafe shortcut rejected");

    // Optional regression against the course map. CTest remains portable when
    // the university repository is elsewhere; run this executable with its PCD.
    if(argc==2) {
      VoxelMap course(0.25f,0.5f);check(course.load(argv[1]),"Load course PCD");
      AStarPlanner planner(course);mission::FlightPathGuard flight(course);
      const auto raw=planner.plan({8.84f,6.9f,2},{2.81f,8.15f,1.5f});
      check(flight.pathFree(raw),"Original raw course route is safe");
      check(!flight.pathFree(planner.simplifyPath(raw)),"Reject original shortcut crossing an inflated voxel");
      check(!flight.segmentFree({7.81226349f,10.1525593f,3.27662396f},
                                {6.88460636f,9.80269909f,3.84544706f}),"Reject independently found short intersection");
    }
    std::cout<<"Flight path guard tests passed\n";
  } catch(const std::exception &e) {std::cerr<<e.what()<<'\n';return 1;}
}
