// Copyright (c) 2026 STU FEI URK
// SPDX-License-Identifier: MIT
#include "lrs_assignment1/astar_planner.hpp"
#include "lrs_assignment1/flight_path_guard.hpp"
#include "lrs_assignment1/mission_file.hpp"
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <std_msgs/msg/float64.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/u_int32.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <mavros_msgs/msg/state.hpp>
#include <mavros_msgs/msg/extended_state.hpp>
#include <mavros_msgs/msg/gpsraw.hpp>
#include <mavros_msgs/msg/position_target.hpp>
#include <mavros_msgs/srv/set_mode.hpp>
#include <mavros_msgs/srv/command_bool.hpp>
#include <mavros_msgs/srv/command_tol.hpp>
#include <mavros_msgs/srv/message_interval.hpp>
#include <nav_msgs/msg/path.hpp>
#include <chrono>
#include <mutex>
#include <memory>
#include <iostream>
#include <limits>
#include <algorithm>

using mission::Vec;
using namespace std::chrono_literals;
class MissionNode : public rclcpp::Node {
  using Target=mavros_msgs::msg::PositionTarget;
  using Mode=mavros_msgs::srv::SetMode;
  using Arm=mavros_msgs::srv::CommandBool;
  using Tol=mavros_msgs::srv::CommandTOL;
  using Interval=mavros_msgs::srv::MessageInterval;
  enum class Stage { IDLE, SET_MODE, ARM, TAKEOFF, ALIGN, NAVIGATE, TASK, LAND, GROUND, FIGURE, FIGURE_END, DONE, ABORT };
  Stage stage_=Stage::IDLE;
  std::recursive_mutex mutex_;
  rclcpp::CallbackGroup::SharedPtr telemetry_group_, control_group_;
  std::vector<rclcpp::SubscriptionBase::SharedPtr> subscriptions_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr abort_;
  rclcpp::Publisher<Target>::SharedPtr target_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr stage_pub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
  rclcpp::Client<Mode>::SharedPtr mode_client_;
  rclcpp::Client<Arm>::SharedPtr arm_client_;
  rclcpp::Client<Tol>::SharedPtr takeoff_client_;
  rclcpp::Client<Interval>::SharedPtr interval_client_;
  double interval_request_at_=-1e9;
  size_t interval_index_=0;
  mavros_msgs::msg::State vehicle_;
  uint8_t landed_=0;
  uint32_t satellites_=0;
  double satellites_at_=-1e9;
  sensor_msgs::msg::NavSatFix fix_;
  mavros_msgs::msg::GPSRAW gps_;
  Vec position_, velocity_, launch_, target_, poi_, map_origin_, alignment_hold_, command_position_;
  double yaw_=0, target_yaw_=0, compass_=0, relative_alt_=0, launch_relative_alt_=0;
  double state_at_=-1e9, pose_at_=-1e9, velocity_at_=-1e9, fix_at_=-1e9, compass_at_=-1e9, gps_at_=-1e9, altitude_at_=-1e9, landed_at_=-1e9;
  double entered_=0, settled_=-1, last_request_=-1e9, figure_start_=0, report_at_=0, previous_tick_=0, gps_good_since_=-1;
  bool pending_=false, takeoff_accepted_=false, publish_=false, outdoor_;
  size_t waypoint_=0, path_index_=0;
  std::vector<mission::Waypoint> waypoints_;
  std::vector<std::vector<Vec>> routes_;
  std::unique_ptr<mission::LocalFrame> geo_;
  std::unique_ptr<VoxelMap> map_;
  std::unique_ptr<AStarPlanner> planner_;
  std::unique_ptr<mission::FlightPathGuard> flight_guard_;
  double hard_, soft_, pass_, settle_time_, speed_tolerance_, yaw_tolerance_, yaw_hold_, timeout_, omega_, amplitude_, altitude_, square_, fence_, max_altitude_, map_yaw_, navigation_speed_, max_tracking_error_, stage_timeout_, gps_settle_, compass_tolerance_;
  int min_satellites_, laps_;
  static double clockSeconds() {
    return std::chrono::duration<double>(std::chrono::steady_clock::now().time_since_epoch()).count();
  }
  template<class T, class F> void subscribe(const std::string &topic, F callback) {
    rclcpp::SubscriptionOptions options; options.callback_group=telemetry_group_;
    subscriptions_.push_back(create_subscription<T>(topic,rclcpp::SensorDataQoS(),
      [this,callback](typename T::ConstSharedPtr message) {
        std::lock_guard<std::recursive_mutex> lock(mutex_); callback(*message);
      },options));
  }
  double positive(const std::string &name,double fallback) {
    double v=declare_parameter(name,fallback);
    if(!std::isfinite(v) || v<=0) throw std::runtime_error(name+" must be positive and finite");
    return v;
  }
  Vec fromMap(Vec p) const {
    double x=p.x-map_origin_.x,y=p.y-map_origin_.y;
    return {launch_.x+std::cos(map_yaw_)*x-std::sin(map_yaw_)*y,
            launch_.y+std::sin(map_yaw_)*x+std::cos(map_yaw_)*y,launch_.z+p.z-map_origin_.z};
  }
  Vec toMap(Vec p) const {
    double x=p.x-launch_.x,y=p.y-launch_.y;
    return {map_origin_.x+std::cos(map_yaw_)*x+std::sin(map_yaw_)*y,
            map_origin_.y-std::sin(map_yaw_)*x+std::cos(map_yaw_)*y,map_origin_.z+p.z-launch_.z};
  }
  static Point3D point(Vec p) { return {float(p.x),float(p.y),float(p.z)}; }
  void stage(Stage next,const char *name) {
    stage_=next; entered_=clockSeconds(); settled_=-1;
    std_msgs::msg::String s; s.data=name; stage_pub_->publish(s);
    RCLCPP_INFO(get_logger(),"State: %s",name);
  }
  bool stable(bool condition,double duration) {
    if(!condition) { settled_=-1; return false; }
    if(settled_<0) settled_=clockSeconds();
    return clockSeconds()-settled_>=duration;
  }
  bool reached(double tolerance,bool hard) {
    bool near=mission::distance(position_,target_)<tolerance;
    if(!hard) return near;
    return stable(near && mission::distance(velocity_,{})<speed_tolerance_,settle_time_);
  }
  bool gpsReady(double t) const {
    return t-fix_at_<timeout_ && t-compass_at_<timeout_ && t-altitude_at_<timeout_ &&
      fix_.status.status>=0 &&
      ((t-gps_at_<timeout_ && gps_.fix_type>=3 && gps_.satellites_visible!=255 && gps_.satellites_visible>=min_satellites_) ||
       (t-satellites_at_<timeout_ && satellites_!=255 && satellites_>=static_cast<uint32_t>(min_satellites_))) &&
      std::isfinite(fix_.latitude) && std::isfinite(fix_.longitude) && std::isfinite(fix_.altitude) &&
      std::isfinite(compass_) && compass_>=0 && compass_<=360 && std::isfinite(relative_alt_) &&
      std::abs(fix_.latitude)<=85 && std::abs(fix_.longitude)<=180;
  }
  void stop(const std::string &why,bool land) {
    if(stage_==Stage::ABORT || stage_==Stage::DONE) return;
    publish_=false; stage(Stage::ABORT,"ABORT");
    RCLCPP_ERROR(get_logger(),"%s",why.c_str());
    // Never reassert GUIDED or send targets after a pilot mode change.
    if(land && vehicle_.connected && vehicle_.armed && vehicle_.mode=="GUIDED" && mode_client_->service_is_ready()) {
      auto request=std::make_shared<Mode::Request>(); request->custom_mode="LAND";
      mode_client_->async_send_request(request,[this](rclcpp::Client<Mode>::SharedFuture f) {
        if(!f.get()->mode_sent) RCLCPP_ERROR(get_logger(),"Abort LAND request rejected; pilot takeover required");
      });
    }
  }
  template<class Service> bool ready(const std::shared_ptr<rclcpp::Client<Service>> &client) {
    return !pending_ && clockSeconds()-last_request_>2 && client->service_is_ready();
  }
  void setMode(const std::string &name) {
    if(!ready(mode_client_)) return;
    pending_=true; last_request_=clockSeconds();
    auto r=std::make_shared<Mode::Request>(); r->custom_mode=name;
    mode_client_->async_send_request(r,[this](rclcpp::Client<Mode>::SharedFuture f) {
      std::lock_guard<std::recursive_mutex> lock(mutex_); pending_=false;
      if(!f.get()->mode_sent) stop("Mode request rejected",false);
    });
  }
  void arm(bool value) {
    if(!ready(arm_client_)) return;
    pending_=true; last_request_=clockSeconds();
    auto r=std::make_shared<Arm::Request>(); r->value=value;
    arm_client_->async_send_request(r,[this](rclcpp::Client<Arm>::SharedFuture f) {
      std::lock_guard<std::recursive_mutex> lock(mutex_); pending_=false;
      auto response=f.get();
      if(!response->success) stop("Arming/disarming rejected, MAV_RESULT="+std::to_string(response->result),false);
    });
  }
  void takeoff() {
    if(takeoff_accepted_ || !ready(takeoff_client_)) return;
    pending_=true; last_request_=clockSeconds();
    auto r=std::make_shared<Tol::Request>();
    r->altitude=float(target_.z-launch_.z+launch_relative_alt_);
    r->yaw=std::numeric_limits<float>::quiet_NaN();
    takeoff_client_->async_send_request(r,[this](rclcpp::Client<Tol>::SharedFuture f) {
      std::lock_guard<std::recursive_mutex> lock(mutex_); pending_=false;
      auto response=f.get(); takeoff_accepted_=response->success;
      if(!response->success) stop("Takeoff rejected, MAV_RESULT="+std::to_string(response->result),true);
    });
  }
  void prepareOutdoor() {
    geo_=std::make_unique<mission::LocalFrame>(fix_.latitude,fix_.longitude,fix_.altitude);
    // The first converged fix is the geodetic origin, paired with the local EKF pose.
    double east=get_parameter("poi_east").as_double(), north=get_parameter("poi_north").as_double();
    if(!std::isfinite(east) || !std::isfinite(north)) throw std::runtime_error("Invalid POI");
    auto roundtrip=[this](Vec offset) {
      Vec llh=geo_->geodetic(offset); Vec enu=geo_->local(llh);
      RCLCPP_INFO(get_logger(),"Geodetic goal %.9f %.9f %.3f",llh.x,llh.y,llh.z);
      return Vec{launch_.x+enu.x,launch_.y+enu.y,launch_.z+enu.z};
    };
    poi_=roundtrip({east,north,altitude_});
    if(std::hypot(east,north)+std::max(square_/std::sqrt(2.0),amplitude_*std::sqrt(1.25))>fence_ || altitude_>max_altitude_)
      throw std::runtime_error("Outdoor geometry exceeds configured geofence");
    waypoints_.push_back({{launch_.x,launch_.y,launch_.z+altitude_},true,"takeoff"});
    for(Vec c:std::vector<Vec>{{-1,1,0},{1,1,0},{1,-1,0},{-1,-1,0},{-1,1,0}})
      waypoints_.push_back({roundtrip({east+c.x*square_/2,north+c.y*square_/2,altitude_}),true,"-"});
    waypoints_.push_back({poi_,true,"figure"});
    waypoints_.push_back({{launch_.x,launch_.y,launch_.z+altitude_},true,"land"});
    routes_.resize(waypoints_.size());
    for(size_t i=1;i<waypoints_.size();++i) routes_[i]={waypoints_[i].position};
  }
  void publishPath() {
    nav_msgs::msg::Path path; path.header.frame_id="map"; path.header.stamp=now();
    for(const auto &route:routes_) for(Vec p:route) {
      if(!outdoor_) p=fromMap(p);
      geometry_msgs::msg::PoseStamped pose; pose.header=path.header;
      pose.pose.position.x=p.x; pose.pose.position.y=p.y; pose.pose.position.z=p.z; pose.pose.orientation.w=1;
      path.poses.push_back(pose);
    }
    path_pub_->publish(path);
  }
  void selectTarget() {
    // Look ahead along the planner's own vertices. Every skipped connection is
    // validated, so raw fallback does not force a stop at each voxel centre.
    if(!outdoor_)
      while(path_index_+1<routes_[waypoint_].size() &&
            flight_guard_->segmentFree(point(toMap(position_)),point(routes_[waypoint_][path_index_+1])))
        ++path_index_;
    target_=routes_[waypoint_][path_index_]; if(!outdoor_) target_=fromMap(target_);
    double dx=target_.x-position_.x,dy=target_.y-position_.y;
    if(std::hypot(dx,dy)>0.02) target_yaw_=mission::unwrap(target_yaw_,std::atan2(dy,dx));
    alignment_hold_=position_;
    command_position_=position_;
    if(std::abs(mission::wrap(yaw_-target_yaw_))<yaw_tolerance_)
      stage(Stage::NAVIGATE,"NAVIGATE");
    else stage(Stage::ALIGN,"ALIGN");
  }
  void nextWaypoint() {
    // A soft arrival can leave a small offset from the nominal mission row.
    // Check the connector before changing to the next preplanned leg.
    if(!outdoor_ && waypoint_+1<routes_.size() && !routes_[waypoint_+1].empty() &&
       !flight_guard_->segmentFree(point(toMap(position_)),point(routes_[waypoint_+1].front()))) return;
    ++waypoint_; path_index_=0;
    if(waypoint_>=waypoints_.size()) { publish_=false; stage(Stage::DONE,"DONE"); return; }
    selectTarget();
  }
  void task() {
    const auto &w=waypoints_[waypoint_];
    if(w.task=="land" || w.task=="landtakeoff") {
      publish_=false; stage(Stage::LAND,"LAND"); return;
    }
    if(w.task=="figure") {
      target_yaw_=mission::unwrap(target_yaw_,mission::tangent(0));
      if(stable(std::abs(mission::wrap(yaw_-target_yaw_))<yaw_tolerance_,yaw_hold_)) {
        figure_start_=now().seconds(); stage(Stage::FIGURE,"FIGURE");
      }
      return;
    }
    if(w.task.rfind("yaw",0)==0) {
      target_yaw_=mission::unwrap(target_yaw_,w.yaw+map_yaw_);
      if(!stable(std::abs(mission::wrap(yaw_-target_yaw_))<yaw_tolerance_ &&
           mission::distance(position_,target_)<hard_ && mission::distance(velocity_,{})<speed_tolerance_,yaw_hold_)) return;
    }
    nextWaypoint();
  }
  void tick() {
    std::lock_guard<std::recursive_mutex> lock(mutex_);
    double t=clockSeconds();
    const double motion_time=now().seconds();
    double dt=previous_tick_>0?std::clamp(motion_time-previous_tick_,0.0,0.1):0.0; previous_tick_=motion_time;
    if(stage_==Stage::DONE || stage_==Stage::ABORT) return;
    bool telemetry=vehicle_.connected && t-state_at_<timeout_ && t-pose_at_<timeout_ && t-velocity_at_<timeout_ && t-landed_at_<timeout_;
    if(stage_==Stage::IDLE) {
      // A direct SITL connection may not stream these until explicitly requested.
      const uint32_t ids[]={245,32,30,33,24}; // landed state, pose, attitude, global position, GPS
      if(vehicle_.connected && interval_index_<5 && t-interval_request_at_>1 && interval_client_->service_is_ready()) {
        auto request=std::make_shared<Interval::Request>();
        request->message_id=ids[interval_index_++]; request->message_rate=10;
        interval_request_at_=t;
        interval_client_->async_send_request(request,[this](rclcpp::Client<Interval>::SharedFuture response){
          if(!response.get()->success) RCLCPP_WARN(get_logger(),"Telemetry-rate request rejected; waiting for existing stream");
        });
      }
      RCLCPP_INFO_THROTTLE(get_logger(),*get_clock(),5000,"Waiting for telemetry: connected=%d local=%d landed=%d gps=%d",
        vehicle_.connected,t-pose_at_<timeout_,t-landed_at_<timeout_,!outdoor_ || gpsReady(t));
      // Finish requesting the attitude stream before allowing the handshake.
      // Without ATTITUDE, MAVROS can publish a position with a zero quaternion.
      if(interval_index_<5 && interval_client_->service_is_ready()) return;
      if(!telemetry || (outdoor_ && !gpsReady(t))) { gps_good_since_=-1; return; }
      if(outdoor_) {
        if(gps_good_since_<0) gps_good_since_=t;
        if(t-gps_good_since_<gps_settle_) return;
      }
      if(outdoor_ && std::abs(mission::wrap(yaw_-mission::compassYaw(compass_)))>compass_tolerance_) {
        RCLCPP_WARN_THROTTLE(get_logger(),*get_clock(),3000,"Compass and ENU pose disagree beyond compass_tolerance; refusing to arm"); return;
      }
      if(vehicle_.armed || landed_!=mavros_msgs::msg::ExtendedState::LANDED_STATE_ON_GROUND) { stop("Start requires a disarmed vehicle on the ground",false); return; }
      launch_=position_; command_position_=position_; target_yaw_=yaw_; launch_relative_alt_=outdoor_?relative_alt_:0;
      try { if(outdoor_) prepareOutdoor(); } catch(const std::exception &e) { stop(e.what(),false); return; }
      target_=outdoor_?waypoints_[0].position:fromMap(waypoints_[0].position);
      if(std::hypot(target_.x-launch_.x,target_.y-launch_.y)>hard_) { stop("First takeoff waypoint must be above map origin / launch",false); return; }
      publishPath(); stage(Stage::SET_MODE,"SET_MODE");
    }
    if(!telemetry) { stop("MAVROS telemetry stale/disconnected",false); return; }
    if(outdoor_ && !gpsReady(t)) { stop("GPS/compass/relative altitude invalid or stale",true); return; }
    if(pending_ && t-last_request_>10) { stop("MAVROS service response timeout",true); return; }
    if(t-entered_>stage_timeout_ && stage_!=Stage::FIGURE) { stop("Mission stage timed out",true); return; }
    bool flight=stage_==Stage::TAKEOFF || stage_==Stage::ALIGN || stage_==Stage::NAVIGATE || stage_==Stage::TASK || stage_==Stage::FIGURE || stage_==Stage::FIGURE_END;
    if(flight && (!vehicle_.armed || vehicle_.mode!="GUIDED")) { stop("Pilot mode change or unexpected disarm",false); return; }
    if(outdoor_ && (std::hypot(position_.x-launch_.x,position_.y-launch_.y)>fence_ || position_.z-launch_.z>max_altitude_)) { stop("Geofence exceeded",true); return; }
    switch(stage_) {
      case Stage::SET_MODE:
        if(vehicle_.mode=="GUIDED" && !pending_) stage(Stage::ARM,"ARM"); else setMode("GUIDED");
        break;
      case Stage::ARM:
        if(vehicle_.mode!="GUIDED") { stop("Mode changed before arming",false); break; }
        if(vehicle_.armed && !pending_) { takeoff_accepted_=false; stage(Stage::TAKEOFF,"TAKEOFF"); }
        else arm(true);
        break;
      case Stage::TAKEOFF:
        takeoff();
        publish_=takeoff_accepted_ && position_.z>launch_.z+0.25;
        if(takeoff_accepted_ && landed_==mavros_msgs::msg::ExtendedState::LANDED_STATE_IN_AIR &&
           (!outdoor_ || std::abs(relative_alt_-launch_relative_alt_-altitude_)<hard_) && reached(hard_,true)) {
          publish_=true; nextWaypoint();
        }
        break;
      case Stage::ALIGN:
        if(outdoor_ && waypoint_>=1 && waypoint_<=5) stage(Stage::NAVIGATE,"NAVIGATE");
        else if(stable(std::abs(mission::wrap(yaw_-target_yaw_))<yaw_tolerance_,0.2)) stage(Stage::NAVIGATE,"NAVIGATE");
        break;
      case Stage::NAVIGATE: {
        const bool final=path_index_+1==routes_[waypoint_].size();
        bool hard=final && (waypoints_[waypoint_].hard || waypoints_[waypoint_].task!="-");
        if(reached(final?(hard?hard_:soft_):pass_,hard)) {
          if(!final) {
            Vec next=routes_[waypoint_][path_index_+1];
            if(!outdoor_ && !flight_guard_->segmentFree(point(toMap(position_)),point(next))) break;
            ++path_index_; selectTarget();
          } else stage(Stage::TASK,"TASK");
        }
        break;
      }
      case Stage::TASK: task(); break;
      case Stage::FIGURE: {
        double phase=omega_*(now().seconds()-figure_start_);
        if(phase>=2*mission::pi*laps_) {
          target_=poi_; stage(Stage::FIGURE_END,"FIGURE_END"); break;
        }
        if(phase<0) { stop("ROS clock moved backwards",true); break; }
        target_=mission::figure(phase,amplitude_,poi_);
        target_yaw_=mission::unwrap(target_yaw_,mission::tangent(phase));
        break;
      }
      case Stage::FIGURE_END:
        if(reached(hard_,true)) nextWaypoint();
        break;
      case Stage::LAND:
        if(vehicle_.mode!="LAND") {
          if(vehicle_.mode!="GUIDED") { stop("Pilot mode change while landing",false); break; }
          setMode("LAND");
        } else if(landed_==mavros_msgs::msg::ExtendedState::LANDED_STATE_ON_GROUND) {
          if(vehicle_.armed) arm(false);
          else if(!pending_) {
            if(waypoints_[waypoint_].task=="landtakeoff") stage(Stage::GROUND,"GROUND");
            else stage(Stage::DONE,"DONE");
          }
        }
        break;
      case Stage::GROUND:
        if(vehicle_.mode!="LAND") { stop("Pilot mode change during ground wait",false); break; }
        if(t-entered_>=3) { target_=fromMap(waypoints_[waypoint_].position); stage(Stage::SET_MODE,"SET_MODE"); }
        break;
      default: break;
    }
    if(publish_) {
      if(outdoor_ && waypoint_>=1 && waypoint_<=5)
        target_yaw_=mission::unwrap(target_yaw_,std::atan2(poi_.y-position_.y,poi_.x-position_.x));
      Target msg; msg.header.stamp=now(); msg.header.frame_id="map";
      msg.coordinate_frame=Target::FRAME_LOCAL_NED; // MAVROS accepts ENU and converts to NED.
      msg.type_mask=Target::IGNORE_VX|Target::IGNORE_VY|Target::IGNORE_VZ|Target::IGNORE_AFX|Target::IGNORE_AFY|Target::IGNORE_AFZ|Target::IGNORE_YAW_RATE;
      Vec p=target_;
      if(stage_==Stage::ALIGN) { p=alignment_hold_; command_position_=p; }
      else if(stage_==Stage::NAVIGATE) {
        // Slew position targets along the validated leg at a bounded speed.
        // Do not let the reference run away when the vehicle cannot keep up.
        double remaining=mission::distance(command_position_,target_);
        if(remaining>1e-6 && mission::distance(position_,command_position_)<max_tracking_error_) {
          double ratio=std::min(1.0,navigation_speed_*dt/remaining);
          command_position_.x+=ratio*(target_.x-command_position_.x);
          command_position_.y+=ratio*(target_.y-command_position_.y);
          command_position_.z+=ratio*(target_.z-command_position_.z);
        }
        p=command_position_;
      } else command_position_=p;
      if(stage_==Stage::FIGURE) {
        // Analytic velocity feed-forward reduces lag at the crossing.
        double phase=omega_*(now().seconds()-figure_start_);
        msg.type_mask &= ~(Target::IGNORE_VX|Target::IGNORE_VY|Target::IGNORE_VZ);
        msg.velocity.x=amplitude_*omega_*std::cos(phase);
        msg.velocity.y=amplitude_*omega_*std::cos(2*phase);
        msg.velocity.z=0;
        if(mission::distance(position_,p)>max_tracking_error_) {
          stop("Figure tracking error exceeded limit",true); return;
        }
      }
      msg.position.x=p.x; msg.position.y=p.y; msg.position.z=p.z; msg.yaw=float(target_yaw_);
      target_pub_->publish(msg);
    }
    if(outdoor_ && geo_ && t-report_at_>1) {
      report_at_=t; Vec enu=geo_->local({fix_.latitude,fix_.longitude,fix_.altitude});
      Vec reported=geo_->geodetic({position_.x-launch_.x,position_.y-launch_.y,position_.z-launch_.z});
      RCLCPP_INFO(get_logger(),"GPS %.9f %.9f alt %.2f; GPS ENU %.2f %.2f %.2f; local->geo %.9f %.9f; rel_alt %.2f; compass %.1f -> ENU %.1f; pose yaw %.1f; delta %.1f deg",
        fix_.latitude,fix_.longitude,fix_.altitude,enu.x,enu.y,enu.z,reported.x,reported.y,relative_alt_,compass_,mission::compassYaw(compass_)*180/mission::pi,yaw_*180/mission::pi,mission::wrap(yaw_-mission::compassYaw(compass_))*180/mission::pi);
    }
  }
public:
  MissionNode():Node("mission_control") {
    const auto kind=declare_parameter<std::string>("mission_type","indoor");
    if(kind!="indoor" && kind!="outdoor") throw std::runtime_error("mission_type must be indoor or outdoor");
    outdoor_=kind=="outdoor";
    hard_=positive("hard_tolerance",0.18); soft_=positive("soft_tolerance",0.4); pass_=positive("path_tolerance",0.35);
    settle_time_=positive("settle_seconds",1.0); speed_tolerance_=positive("settle_speed",0.15);
    yaw_tolerance_=positive("yaw_tolerance",0.0872665); yaw_hold_=positive("yaw_hold_seconds",2.0);
    navigation_speed_=positive("navigation_speed",0.6);
    max_tracking_error_=positive("max_tracking_error",1.5);
    stage_timeout_=positive("stage_timeout",180.0);
    gps_settle_=positive("gps_settle_seconds",3.0);
    compass_tolerance_=positive("compass_tolerance",0.35);
    timeout_=positive("telemetry_timeout",3.0); omega_=positive("omega",0.2); amplitude_=positive("amplitude",3.0);
    altitude_=positive("altitude",5.0); square_=positive("square_side",5.0);
    fence_=positive("geofence_radius",15.0); max_altitude_=positive("geofence_altitude",8.0);
    laps_=declare_parameter("laps",1); min_satellites_=declare_parameter("min_satellites",8);
    if(laps_<1 || min_satellites_<4 || min_satellites_>254 || amplitude_*omega_*std::sqrt(2.0)>1.5)
      throw std::runtime_error("Invalid laps, satellite threshold, or figure speed > 1.5 m/s");
    declare_parameter("poi_east",0.0); declare_parameter("poi_north",0.0);
    map_origin_={declare_parameter("map_origin_x",13.0),declare_parameter("map_origin_y",7.0),declare_parameter("map_origin_z",0.0)};
    map_yaw_=declare_parameter("map_to_local_yaw",0.0);
    if(!std::isfinite(map_origin_.x) || !std::isfinite(map_origin_.y) || !std::isfinite(map_origin_.z) || !std::isfinite(map_yaw_))
      throw std::runtime_error("Map transform must be finite");
    if(hard_>=soft_) throw std::runtime_error("hard_tolerance must be smaller than soft_tolerance");
    std::string mission_file=declare_parameter<std::string>("mission_file","");
    std::string map_file=declare_parameter<std::string>("map_file","");
    double voxel=positive("voxel_size",0.25),radius=positive("safety_radius",0.65);
    if(!outdoor_) {
      waypoints_=mission::loadMission(mission_file);
      map_=std::make_unique<VoxelMap>(float(voxel),float(radius));
      if(!map_->load(map_file)) throw std::runtime_error("Map loading failed");
      planner_=std::make_unique<AStarPlanner>(*map_);
      flight_guard_=std::make_unique<mission::FlightPathGuard>(*map_);
      routes_.resize(waypoints_.size());
      // Plan every leg before arming: no search can block the 25 Hz control timer.
      for(size_t i=1;i<waypoints_.size();++i) {
        double begin=clockSeconds();
        auto raw=planner_->plan(point(waypoints_[i-1].position),point(waypoints_[i].position));
        if(raw.empty()) throw std::runtime_error("Planner returned no route for mission leg "+std::to_string(i));
        // Keep the A1.1 implementation unchanged. At its consumer boundary,
        // restore exact endpoints even if a same-voxel route contains only goal.
        raw.insert(raw.begin(),point(waypoints_[i-1].position));
        raw.push_back(point(waypoints_[i].position));
        auto path=planner_->simplifyPath(raw);
        if(!flight_guard_->pathFree(path)) {
          RCLCPP_WARN(get_logger(),"Leg %zu: simplified route rejected by flight validator; using raw A* route",i);
          path=raw;
        }
        if(!flight_guard_->pathFree(path)) throw std::runtime_error("No safe route for mission leg "+std::to_string(i));
        for(size_t j=1;j<path.size();++j) routes_[i].push_back({path[j].x,path[j].y,path[j].z});
        if(routes_[i].empty()) routes_[i].push_back(waypoints_[i].position);
        RCLCPP_INFO(get_logger(),"Leg %zu: %zu -> %zu points, %.2f ms",i,raw.size(),path.size(),1000*(clockSeconds()-begin));
      }
    }
    telemetry_group_=create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    control_group_=create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    target_pub_=create_publisher<Target>("/mavros/setpoint_raw/local",10);
    stage_pub_=create_publisher<std_msgs::msg::String>("~/state",rclcpp::QoS(1).transient_local());
    path_pub_=create_publisher<nav_msgs::msg::Path>("~/planned_path",rclcpp::QoS(1).transient_local());
    mode_client_=create_client<Mode>("/mavros/set_mode");
    arm_client_=create_client<Arm>("/mavros/cmd/arming");
    takeoff_client_=create_client<Tol>("/mavros/cmd/takeoff");
    interval_client_=create_client<Interval>("/mavros/set_message_interval");
    subscribe<mavros_msgs::msg::State>("/mavros/state",[this](const auto &m){vehicle_=m;state_at_=clockSeconds();});
    subscribe<mavros_msgs::msg::ExtendedState>("/mavros/extended_state",[this](const auto &m){landed_=m.landed_state;landed_at_=clockSeconds();});
    subscribe<geometry_msgs::msg::PoseStamped>("/mavros/local_position/pose",[this](const auto &m){
      const auto &p=m.pose.position; const auto &q=m.pose.orientation;
      const double norm=q.x*q.x+q.y*q.y+q.z*q.z+q.w*q.w;
      if(!std::isfinite(p.x+p.y+p.z+norm) || std::abs(norm-1.0)>0.01) return;
      position_={p.x,p.y,p.z}; yaw_=std::atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z)); pose_at_=clockSeconds();
    });
    subscribe<geometry_msgs::msg::TwistStamped>("/mavros/local_position/velocity_local",[this](const auto &m){
      auto v=m.twist.linear; if(!std::isfinite(v.x+v.y+v.z)) return; velocity_={v.x,v.y,v.z};velocity_at_=clockSeconds();
    });
    subscribe<sensor_msgs::msg::NavSatFix>("/mavros/global_position/global",[this](const auto &m){fix_=m;fix_at_=clockSeconds();});
    subscribe<std_msgs::msg::UInt32>("/mavros/global_position/raw/satellites",[this](const auto &m){satellites_=m.data;satellites_at_=clockSeconds();});
    subscribe<mavros_msgs::msg::GPSRAW>("/mavros/gpsstatus/gps1/raw",[this](const auto &m){gps_=m;gps_at_=clockSeconds();});
    subscribe<std_msgs::msg::Float64>("/mavros/global_position/compass_hdg",[this](const auto &m){compass_=m.data;compass_at_=clockSeconds();});
    subscribe<std_msgs::msg::Float64>("/mavros/global_position/rel_alt",[this](const auto &m){relative_alt_=m.data;altitude_at_=clockSeconds();});
    abort_=create_service<std_srvs::srv::Trigger>("~/abort",[this](const std::shared_ptr<std_srvs::srv::Trigger::Request>,std::shared_ptr<std_srvs::srv::Trigger::Response> response){
      std::lock_guard<std::recursive_mutex> lock(mutex_);stop("Operator abort",true);response->success=true;response->message="Setpoints stopped; LAND requested if in GUIDED";
    });
    entered_=clockSeconds(); timer_=create_wall_timer(40ms,[this]{tick();},control_group_);
    RCLCPP_INFO(get_logger(),"Ready: %s mission; waiting for connected, fresh ground telemetry",kind.c_str());
  }
};
int main(int argc,char **argv) {
  rclcpp::init(argc,argv);
  try {
    auto node=std::make_shared<MissionNode>();
    rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions(),3);
    executor.add_node(node); executor.spin();
  } catch(const std::exception &e) { std::cerr<<"Mission error: "<<e.what()<<std::endl; rclcpp::shutdown(); return 1; }
  rclcpp::shutdown(); return 0;
}
