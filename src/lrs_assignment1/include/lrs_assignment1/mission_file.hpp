// Copyright (c) 2026 STU FEI URK
// SPDX-License-Identifier: MIT
#pragma once
#include "lrs_assignment1/mission_math.hpp"
#include <fstream>
#include <regex>
#include <string>
#include <vector>

namespace mission {
struct Waypoint { Vec position; bool hard; std::string task; double yaw{}; };
inline std::string trim(std::string s) {
  auto a=s.find_first_not_of(" \t\r\n");
  return a==std::string::npos ? "" : s.substr(a,s.find_last_not_of(" \t\r\n")-a+1);
}
inline double number(const std::string &s) {
  // CSV fields use decimal/scientific notation, never NaN, infinity or hex floats.
  static const std::regex decimal(R"([+-]?(?:[0-9]+(?:\.[0-9]*)?|\.[0-9]+)(?:[eE][+-]?[0-9]+)?)");
  if(!std::regex_match(s,decimal)) throw std::runtime_error("Invalid number: "+s);
  size_t n; double v=std::stod(s,&n);
  if(n!=s.size() || !std::isfinite(v)) throw std::runtime_error("Invalid number: "+s);
  return v;
}
inline std::vector<Waypoint> loadMission(const std::string &path) {
  std::ifstream file(path);
  if(!file) throw std::runtime_error("Cannot open mission: "+path);
  std::vector<Waypoint> result; std::string line; size_t row=0;
  while(std::getline(file,line)) {
    ++row; line=trim(line);
    if(line.empty() || line[0]=='#') continue;
    try {
      std::vector<std::string> f;
      size_t begin=0;
      for(;;) {
        const auto end=line.find(',',begin);
        f.push_back(trim(line.substr(begin,end==std::string::npos?end:end-begin)));
        if(end==std::string::npos) break;
        begin=end+1;
      }
      if(f.size()!=5) throw std::runtime_error("Expected x,y,z,precision,task");
      if(f==std::vector<std::string>{"x","y","z","precision","task"}) {
        if(!result.empty()) throw std::runtime_error("Header must precede mission rows");
        continue;
      }
      if(f[3]!="hard" && f[3]!="soft") throw std::runtime_error("Unknown precision");
      Waypoint w{{number(f[0]),number(f[1]),number(f[2])},f[3]=="hard",f[4]};
      if(w.position.z<=0) throw std::runtime_error("Waypoint altitude must be positive");
      if(w.task.rfind("yaw",0)==0) w.yaw=number(trim(w.task.substr(3)))*pi/180;
      else if(w.task!="-" && w.task!="takeoff" && w.task!="land" && w.task!="landtakeoff")
        throw std::runtime_error("Unknown task");
      result.push_back(w);
    } catch(const std::exception &e) { throw std::runtime_error("Mission row "+std::to_string(row)+": "+e.what()); }
  }
  if(result.size()<2 || result.front().task!="takeoff" || result.back().task!="land")
    throw std::runtime_error("Mission must begin with takeoff and end with land");
  for(size_t i=1;i<result.size();++i)
    if(result[i].task=="takeoff" || (result[i].task=="land" && i+1<result.size()))
      throw std::runtime_error("Use landtakeoff for intermediate landing");
  return result;
}
}
