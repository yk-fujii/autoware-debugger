#include <ros/ros.h>
#include <std_msgs/Int32.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>
#include <visualization_msgs/MarkerArray.h>

#include <iostream>
#include <map>
#include <string>
#include <vector>

#include "autoware_msgs/ConfigLaneStop.h"
#include "autoware_msgs/LaneArray.h"
#include "autoware_msgs/traffic_light.h"
#include "waypoint_follower/libwaypoint_follower.h"
#include <dynamic_reconfigure/server.h>
#include <waypoint_debugger/WaypointDebuggerConfig.h>

#include <amathutils_lib/amathutils.hpp>
#include <ros/console.h>
#include <vector_map/vector_map.h>

namespace autoware_debug {
using namespace vector_map;
geometry_msgs::Point to_geoPoint(const vector_map_msgs::Point &vp) {
  geometry_msgs::Point gp;
  gp.x = vp.ly;
  gp.y = vp.bx;
  gp.z = vp.h;
  return gp;
}
class LaneDebugger {
private:
  tf::TransformListener *tflistener_;
  std::map<std::string, ros::Subscriber> Subs_;
  std::map<std::string, ros::Publisher> Pubs_;
  vector_map::VectorMap vmap_;

  ros::NodeHandle nh_;
  ros::NodeHandle nh_local_;
  int closestWaypointID_ = -1;
  visualization_msgs::MarkerArray lanes_marker_array;

  std_msgs::ColorRGBA color_green_;
  std_msgs::ColorRGBA color_red_;
  std_msgs::ColorRGBA color_blue_;

public:
  LaneDebugger() {
    color_green_.r = 0.2;
    color_green_.g = 1.0;
    color_green_.b = 0.2;
    color_red_.r = 1.0;
    color_red_.g = 0.2;
    color_red_.b = 0.2;
    color_blue_.r = 0.2;
    color_blue_.g = 0.2;
    color_blue_.b = 1.0;

    color_green_.a = color_red_.a = color_blue_.a = 1.0;
  }
  ~LaneDebugger() {
    if (tflistener_)
      delete tflistener_;
  }

  std::vector<geometry_msgs::Point>
  getNearestDTLane(geometry_msgs::PointStamped pt) {
    std::vector<geometry_msgs::Point> ret;
    std::vector<DTLane> DTLanes =
        vmap_.findByFilter([](const DTLane &dtlane) { return true; });

    geometry_msgs::PointStamped current_pt_map;

    tflistener_->transformPoint("map", pt, current_pt_map);

    vector_map_msgs::DTLane closest_dtlane;
    vector_map_msgs::Point closest_pid;
    double closest_distance = 999999.999;
    for (auto &dtlane : DTLanes) {
      vector_map_msgs::Point dtlane_pid =
          vmap_.findByKey(Key<Point>(dtlane.pid));
      geometry_msgs::Point p1;
      p1.x = dtlane_pid.ly;
      p1.y = dtlane_pid.bx;
      double distance = amathutils::find_distance(current_pt_map.point, p1);
      if (distance < closest_distance) {
        closest_dtlane = dtlane;
        closest_pid = dtlane_pid;
        closest_distance = distance;
      }
    }

    std::vector<Lane> Lanes =
        vmap_.findByFilter([](const Lane &lane) { return true; });
    std::vector<Lane> Lanes_lnids = vmap_.findByFilter(
        [&](const Lane &lane) { return lane.did == closest_dtlane.did; });

    if (Lanes_lnids.empty())
      return std::vector<geometry_msgs::Point>();

    Lane target_lane = Lanes_lnids.at(0);
    Lane prev_lane = vmap_.findByKey(Key<Lane>(target_lane.lnid));
    do {
      DTLane _dtlane = vmap_.findByKey(Key<DTLane>(prev_lane.did));
      Point lane_pid = vmap_.findByKey(Key<Point>(_dtlane.pid));
      geometry_msgs::Point pt = to_geoPoint(lane_pid);
      ret.insert(ret.begin(), pt);
      prev_lane = vmap_.findByKey(Key<Lane>(prev_lane.blid));
    } while (prev_lane.lnid != 0 && prev_lane.jct == 0);

    Lane next_lane = vmap_.findByKey(Key<Lane>(target_lane.lnid));
    do {
      DTLane _dtlane = vmap_.findByKey(Key<DTLane>(next_lane.did));
      Point lane_pid = vmap_.findByKey(Key<Point>(_dtlane.pid));
      geometry_msgs::Point pt = to_geoPoint(lane_pid);
      ret.push_back(pt);
      next_lane = vmap_.findByKey(Key<Lane>(next_lane.flid));
    } while (next_lane.lnid != 0 && next_lane.jct == 0);

    return ret;
  }
  void clickedPointCallback(const geometry_msgs::PointStamped &msg) {
    std::vector<geometry_msgs::Point> closestDtl = getNearestDTLane(msg);

    visualization_msgs::Marker dtlane_line_marker;
    dtlane_line_marker.header.frame_id = "map";
    dtlane_line_marker.header.stamp = ros::Time();
    dtlane_line_marker.ns = "DTLane";
    dtlane_line_marker.id = 1;
    dtlane_line_marker.type = visualization_msgs::Marker::LINE_STRIP;
    dtlane_line_marker.action = visualization_msgs::Marker::ADD;
    dtlane_line_marker.scale.x = 0.7;
    dtlane_line_marker.scale.y = 0.7;
    dtlane_line_marker.scale.z = 0.7;
    dtlane_line_marker.color.a = 1;
    dtlane_line_marker.color.r = 0.2;
    dtlane_line_marker.color.g = 1;
    dtlane_line_marker.color.b = 0.2;
    dtlane_line_marker.lifetime = ros::Duration(0);

    for (auto &lane : closestDtl) {
      dtlane_line_marker.points.push_back(lane);
    }
    lanes_marker_array.markers.push_back(dtlane_line_marker);
    Pubs_["lane_debug"].publish(lanes_marker_array);
    lanes_marker_array.markers.clear();
  }
  void currentPoseCallback(const geometry_msgs::PoseStamped &msg) {
    geometry_msgs::PointStamped pt;
    pt.header = msg.header;
    pt.point = msg.pose.position;
    clickedPointCallback(pt);
  }

  void init() {
    vmap_.subscribe(nh_,
                    Category::POINT | Category::LINE | Category::VECTOR |
                        Category::AREA | Category::POLE | // basic class
                        Category::DTLANE | Category::LANE | Category::NODE |
                        Category::STOP_LINE | Category::ROAD_SIGN);

    tflistener_ = new tf::TransformListener();

    //    Subs_["lane_waypoints_array"] = nh_.subscribe("lane_waypoints_array",
    //    10, laneArrayCallback);
    //    Subs_["closest_waypoint"] = nh_.subscribe("closest_waypoint", 1,
    //    closestWaypointCallback);
    Subs_["current_pose"] = nh_.subscribe(
        "current_pose", 1, &LaneDebugger::currentPoseCallback, this);
    Subs_["clicked_point"] = nh_.subscribe(
        "/clicked_point", 10, &LaneDebugger::clickedPointCallback, this);

    Pubs_["lane_debug"] =
        nh_.advertise<visualization_msgs::MarkerArray>("lane_debug", 9, true);
  }
  void run() {
    init();
    ros::Rate r(2);
    while (ros::ok()) {
      ros::spinOnce();
      r.sleep();
    }
  }
};
}

using namespace autoware_debug;

int main(int argc, char **argv) {
  ros::init(argc, argv, "LaneDebuggerNode");
  autoware_debug::LaneDebugger lane_debugger;
  lane_debugger.run();
  ros::shutdown();
  return 0;
}
