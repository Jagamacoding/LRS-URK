// Copyright (c) 2026 STU FEI URK
// SPDX-License-Identifier: MIT
// A1.2/A1.3 helpers only: deliberately independent of the unchanged A1.1 planner.
#include "lrs_assignment1/mission_file.hpp"
#include <filesystem>
#include <functional>
#include <iostream>
#include <limits>
#include <unistd.h>

namespace {
void check(bool condition,const std::string &message) {
  if(!condition) throw std::runtime_error(message);
}
void rejects(const std::function<void()> &operation,const std::string &message) {
  bool rejected=false;
  try { operation(); } catch(const std::exception &) { rejected=true; }
  check(rejected,message);
}
struct Scratch {
  std::filesystem::path path=std::filesystem::temp_directory_path()/("lrs-helper-tests-"+std::to_string(getpid()));
  Scratch() { std::filesystem::create_directories(path); }
  ~Scratch() { std::error_code ignored; std::filesystem::remove_all(path,ignored); }
};
mission::Vec ecef(mission::Vec geo) {
  using namespace mission;
  const double lat=geo.x*pi/180,lon=geo.y*pi/180,e2=0.00669437999014;
  const double n=6378137/std::sqrt(1-e2*std::sin(lat)*std::sin(lat));
  return {(n+geo.z)*std::cos(lat)*std::cos(lon),(n+geo.z)*std::cos(lat)*std::sin(lon),
          (n*(1-e2)+geo.z)*std::sin(lat)};
}
mission::Vec referenceEnu(mission::Vec origin,mission::Vec geo) {
  using namespace mission;
  const Vec a=ecef(origin),b=ecef(geo),d{b.x-a.x,b.y-a.y,b.z-a.z};
  const double lat=origin.x*pi/180,lon=origin.y*pi/180;
  return {-std::sin(lon)*d.x+std::cos(lon)*d.y,
          -std::sin(lat)*std::cos(lon)*d.x-std::sin(lat)*std::sin(lon)*d.y+std::cos(lat)*d.z,
          std::cos(lat)*std::cos(lon)*d.x+std::cos(lat)*std::sin(lon)*d.y+std::sin(lat)*d.z};
}
}
int main() {
  using namespace mission;
  try {
    for(const auto &pair:std::vector<std::pair<double,double>>{{0,pi/2},{90,0},{180,-pi/2},{270,pi},{360,pi/2}})
      check(std::abs(wrap(compassYaw(pair.first)-pair.second))<1e-10,"Compass cardinal convention");
    check(std::abs(wrap(-pi)-pi)<1e-10,"Angle interval excludes -pi");
    check(std::abs(unwrap(179*pi/180,-179*pi/180)-181*pi/180)<1e-10,"Yaw unwrap counter-clockwise");
    check(std::abs(unwrap(-179*pi/180,179*pi/180)+181*pi/180)<1e-10,"Yaw unwrap clockwise");
    check(std::abs(unwrap(8*pi+0.1,0.2)-(8*pi+0.2))<1e-10,"Yaw unwrap multiple turns");

    const double nan=std::numeric_limits<double>::quiet_NaN();
    rejects([&]{LocalFrame invalid(nan,0,0);},"Reject nonfinite origin");
    rejects([]{LocalFrame invalid(86,0,0);},"Reject origin outside documented latitude range");
    rejects([]{LocalFrame invalid(0,181,0);},"Reject invalid origin longitude");
    LocalFrame frame(48.15,17.07,150);
    rejects([&]{frame.local({91,17,150});},"Reject invalid latitude");
    rejects([&]{frame.local({48,181,150});},"Reject invalid longitude");
    rejects([&]{frame.geodetic({nan,0,0});},"Reject nonfinite local coordinate");
    check(distance(frame.local({48.15,17.07,150}),{})<1e-9,"Geodetic origin");
    for(Vec p:std::vector<Vec>{{25,-30,5},{-25,30,-5},{0,0,0}})
      check(distance(frame.local(frame.geodetic(p)),p)<1e-6,"Geodetic round trip");
    check(frame.local({48.15,17.0701,155}).x>0,"Increasing longitude is East");
    check(frame.local({48.1501,17.07,155}).y>0,"Increasing latitude is North");
    check(std::abs(frame.local({48.15,17.07,155}).z-5)<1e-9,"Altitude is relative to captured fix");
    LocalFrame dateline(0,179.99999,0);
    check(distance(dateline.local(dateline.geodetic({50,0,0})),{50,0,0})<1e-6,"Antimeridian round trip");
    double max_error=0;
    // Independent ECEF rotation checks the approximation over a 50 m radius field.
    for(double latitude:std::vector<double>{0,48.15,80}) {
      const Vec origin{latitude,17.07,150}; LocalFrame local(origin.x,origin.y,origin.z);
      for(int i=0;i<360;++i) {
        const double angle=i*pi/180; const Vec p{50*std::cos(angle),50*std::sin(angle),5};
        max_error=std::max(max_error,distance(p,referenceEnu(origin,local.geodetic(p))));
      }
    }
    check(max_error<0.01,"Flat field approximation error below 1 cm in tested 50 m cases");

    const Vec centre{4,-2,5};
    check(distance(figure(0,3,centre),centre)<1e-9,"Figure begins at POI");
    check(distance(figure(0,3,centre),figure(2*pi,3,centre))<1e-9,"Figure closes");
    check(std::abs(std::abs(wrap(tangent(0)-tangent(pi)))-pi/2)<1e-10,"Crossing directions differ");
    double previous=tangent(0);
    for(double phase=0;phase<4*pi;phase+=0.001) {
      const Vec a=figure(phase,3,centre),b=figure(phase+1e-6,3,centre);
      check(std::abs(wrap(tangent(phase)-std::atan2(b.y-a.y,b.x-a.x)))<1e-5,"Analytic tangent matches finite difference");
      check(std::abs(a.x-centre.x)<=3.000001 && std::abs(a.y-centre.y)<=1.500001 && a.z==5,"Fixed figure geometry");
      const double continuous=unwrap(previous,tangent(phase));
      check(std::abs(continuous-previous)<0.01,"Continuous tangent across wrap");
      previous=continuous;
      check(distance(a,b)/(1e-6/0.2)<=3*0.2*std::sqrt(2.0)+1e-5,"Figure peak speed bound");
    }

    Scratch scratch; const auto csv=scratch.path/"mission.csv";
    auto read=[&](const std::string &contents) {
      {std::ofstream f(csv); f<<contents;}
      return loadMission(csv.string());
    };
    auto parsed=read("# comment\r\n x, y, z, precision, task \r\n0,0,1,soft,takeoff\n1e0,-.5,1.2,hard,yaw 90\n1,-.5,1.2,hard,landtakeoff\n0,0,1,soft,yaw-180\n0,0,1,hard,land\n");
    check(parsed.size()==5 && !parsed[0].hard && parsed[1].hard,"Mission ordering and precision");
    check(std::abs(parsed[1].yaw-pi/2)<1e-10 && std::abs(parsed[3].yaw+pi)<1e-10,"Compact and spaced yaw tasks");
    check(parsed[2].task=="landtakeoff" && parsed[1].position.y==-0.5,"Intermediate landing and signed decimal parsing");
    check(read("0,0,1,soft,takeoff\n0,0,1,hard,land\n").size()==2,"Header is optional");
    for(const auto &bad:std::vector<std::string>{
        "", "0,0,1,hard,land\n", "0,0,1,hard,takeoff\n",
        "0,0,nan,hard,takeoff\n0,0,1,hard,land\n",
        "0,0,inf,hard,takeoff\n0,0,1,hard,land\n",
        "0,0,1e999,hard,takeoff\n0,0,1,hard,land\n",
        "0,0,0x1p0,hard,takeoff\n0,0,1,hard,land\n",
        "0,0,1m,hard,takeoff\n0,0,1,hard,land\n",
        "0,0,0,hard,takeoff\n0,0,1,hard,land\n",
        "0,0,-1,hard,takeoff\n0,0,1,hard,land\n",
        "0,0,1,medium,takeoff\n0,0,1,hard,land\n",
        "0,0,1,hard,takeoff,\n0,0,1,hard,land\n",
        "0,0,1,hard,takeoff\n0,0,1,hard,\n0,0,1,hard,land\n",
        "0,0,1,hard,takeoff\n0,0,1,hard,yaw\n0,0,1,hard,land\n",
        "0,0,1,hard,takeoff\n0,0,1,hard,yaw90junk\n0,0,1,hard,land\n",
        "0,0,1,hard,takeoff\n0,0,1,hard,land\n0,0,1,hard,land\n",
        "0,0,1,hard,takeoff\n0,0,1,hard,takeoff\n0,0,1,hard,land\n",
        "0,0,1,hard,takeoff\nx,y,z,precision,task\n0,0,1,hard,land\n"})
      rejects([&]{read(bad);},"Reject malformed mission: "+bad);
    rejects([&]{loadMission((scratch.path/"missing.csv").string());},"Reject missing mission file");
    try {read("# comment\n0,0,nan,hard,takeoff\n");}
    catch(const std::exception &e) {check(std::string(e.what()).find("Mission row 2:")!=std::string::npos,"Errors identify source row");}
    std::cout<<"All A1.2/A1.3 helper tests passed; maximum tested 50 m ENU approximation error "<<max_error<<" m\n";
  } catch(const std::exception &e) {std::cerr<<e.what()<<'\n';return 1;}
}
