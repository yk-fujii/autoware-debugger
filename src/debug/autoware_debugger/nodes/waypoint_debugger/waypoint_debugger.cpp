#include <ros/ros.h>
#include <std_msgs/Int32.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>
#include <visualization_msgs/MarkerArray.h>

#include <iostream>
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

using namespace vector_map;
ros::Publisher g_local_mark_pub;
ros::Publisher area_pub;
ros::Publisher g_global_mark_pub;

vector_map::VectorMap g_vmap;

constexpr int32_t TRAFFIC_LIGHT_RED = 0;
constexpr int32_t TRAFFIC_LIGHT_GREEN = 1;
constexpr int32_t TRAFFIC_LIGHT_UNKNOWN = 2;

std_msgs::ColorRGBA _initial_color;
std_msgs::ColorRGBA _global_color;
std_msgs::ColorRGBA g_local_color;
const double g_global_alpha = 0.2;
const double g_local_alpha = 1.0;
int _closest_waypoint = -1;
visualization_msgs::MarkerArray g_global_marker_array;
visualization_msgs::MarkerArray g_local_waypoints_marker_array;

autoware_msgs::LaneArray aLaneArray;

tf::TransformListener *tflistener;

int g_config_count = 0;

namespace autoware_debug {
void publishMarker() {
  visualization_msgs::MarkerArray marker_array;

  // insert global marker
  marker_array.markers.insert(marker_array.markers.end(),
                              g_local_waypoints_marker_array.markers.begin(),
                              g_local_waypoints_marker_array.markers.end());

  g_global_mark_pub.publish(marker_array);
  g_local_waypoints_marker_array.markers.clear();
}

void createWaypointMarker(
    const autoware_msgs::LaneArray &lane_waypoints_array) {
  visualization_msgs::Marker lane_waypoint_marker;
  lane_waypoint_marker.header.frame_id = "map";
  lane_waypoint_marker.header.stamp = ros::Time();
  lane_waypoint_marker.ns = "local_point_marker";
  lane_waypoint_marker.id = 0;
  lane_waypoint_marker.type = visualization_msgs::Marker::CUBE_LIST;
  lane_waypoint_marker.action = visualization_msgs::Marker::ADD;
  lane_waypoint_marker.scale.x = 1;
  lane_waypoint_marker.scale.y = 1;
  lane_waypoint_marker.scale.z = 1;
  lane_waypoint_marker.color.r = 1.0;
  lane_waypoint_marker.color.a = 1.0;
  lane_waypoint_marker.frame_locked = true;

  int count = 0;

  for (auto lane : lane_waypoints_array.lanes) {
    for (unsigned int i = 0; i < lane.waypoints.size(); i++) {
      if (count++ == g_config_count) {
        geometry_msgs::Point point;
        point = lane.waypoints[i].pose.pose.position;
        lane_waypoint_marker.points.push_back(point);
      }
    }
  }
  g_local_waypoints_marker_array.markers.push_back(lane_waypoint_marker);

  lane_waypoint_marker.id = 1;
  lane_waypoint_marker.color.r = 1.0;
  lane_waypoint_marker.color.g = 0.5;
  lane_waypoint_marker.color.b = 1.0;
  lane_waypoint_marker.color.a = 1.0;
  lane_waypoint_marker.points.clear();

  static int _count = 0;
  visualization_msgs::Marker stop_sign;
  stop_sign.header = lane_waypoint_marker.header;
  stop_sign.ns = "sign";
  stop_sign.scale.x = 0.6;
  stop_sign.scale.y = 0.6;
  stop_sign.scale.z = 0.6;
  stop_sign.color.a = 0.8;
  stop_sign.color.r = 1;
  stop_sign.color.g = 1;
  stop_sign.color.b = 1;
  stop_sign.type = visualization_msgs::Marker::MESH_RESOURCE;
  stop_sign.mesh_resource = "package://waypoint_debugger/imgs/stop.dae";
  stop_sign.mesh_use_embedded_materials = true;
  for (auto lane : lane_waypoints_array.lanes) {
    for (unsigned int i = 0; i < lane.waypoints.size(); i++) {
      geometry_msgs::Point point;
      point = lane.waypoints[i].pose.pose.position;
      point.z += 0.5;

      if (lane.waypoints[i].wpstate.stop_state != 0) {
        lane_waypoint_marker.points.push_back(point);
        stop_sign.pose = lane.waypoints[i].pose.pose;
        stop_sign.pose.position.z += 2;
        stop_sign.id = i;
        g_local_waypoints_marker_array.markers.push_back(stop_sign);
        fprintf(stderr, "push\n");
      }
    }
    g_local_waypoints_marker_array.markers.push_back(lane_waypoint_marker);
  }
}

std::string getSteerText(unsigned int &state) {
  switch (state) {
  case (autoware_msgs::WaypointState::STR_LEFT):
    return "left";
  case (autoware_msgs::WaypointState::STR_RIGHT):
    return "right";
  case (autoware_msgs::WaypointState::STR_STRAIGHT):
    return "straight";
  };
  return "unknown";
}

void createWaypointLabelMarker(
    const autoware_msgs::LaneArray &lane_waypoints_array) {
  visualization_msgs::Marker waypoint_label_marker;
  waypoint_label_marker.header.frame_id = "map";
  waypoint_label_marker.header.stamp = ros::Time();
  waypoint_label_marker.ns = "waypoint_label";
  waypoint_label_marker.id = 0;
  waypoint_label_marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
  waypoint_label_marker.action = visualization_msgs::Marker::ADD;
  waypoint_label_marker.scale.x = 0.5;
  waypoint_label_marker.scale.y = 0.5;
  waypoint_label_marker.scale.z = 0.5;
  waypoint_label_marker.color.a = 0.9;
  waypoint_label_marker.color.r = 0.1;
  waypoint_label_marker.color.g = 0.5;
  waypoint_label_marker.color.b = 0.2;
  waypoint_label_marker.lifetime = ros::Duration(10);
  int id;

  for (auto &lane : lane_waypoints_array.lanes) {
    for (size_t wpi = 1; wpi < lane.waypoints.size(); wpi++) {
      waypoint_label_marker.id = ++id;
      waypoint_label_marker.pose.position =
          lane.waypoints.at(wpi).pose.pose.position;
      waypoint_label_marker.pose.position.z += 2.0;

      double v0 = lane.waypoints.at(wpi - 1).twist.twist.linear.x;
      double v = lane.waypoints.at(wpi).twist.twist.linear.x;
      amathutils::point p0(lane.waypoints.at(wpi).pose.pose.position.x,
                           lane.waypoints.at(wpi).pose.pose.position.y,
                           lane.waypoints.at(wpi).pose.pose.position.z);
      amathutils::point p1(lane.waypoints.at(wpi - 1).pose.pose.position.x,
                           lane.waypoints.at(wpi - 1).pose.pose.position.y,
                           lane.waypoints.at(wpi - 1).pose.pose.position.z);

      double distance = amathutils::find_distance(&p0, &p1);
      double ag = amathutils::getGravityAcceleration(
          amathutils::getAcceleration(v0, v, distance));

      waypoint_label_marker.color.g = 0.5 + ag * 2;
      waypoint_label_marker.color.b = 0.5 + ag;

      std::ostringstream strs;
      strs << ag;
      waypoint_label_marker.text =
          getSteerText(
              (unsigned int &)lane.waypoints.at(wpi).wpstate.steering_state) +
          "\n" + strs.str();
      g_local_waypoints_marker_array.markers.push_back(waypoint_label_marker);
    }
  }
}

void createSignMarker() {
  visualization_msgs::Marker stopline_marker;
  stopline_marker.header.frame_id = "map";
  stopline_marker.header.stamp = ros::Time();
  stopline_marker.ns = "road_sign";
  stopline_marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
  stopline_marker.action = visualization_msgs::Marker::ADD;
  stopline_marker.scale.x = 1;
  stopline_marker.scale.y = 1;
  stopline_marker.scale.z = 1;
  stopline_marker.color.a = 1;
  stopline_marker.color.r = 1;
  stopline_marker.color.g = 1;
  stopline_marker.color.b = 0.5;
  stopline_marker.lifetime = ros::Duration(10);

  stopline_marker.pose.position.x =
      stopline_marker.points.at(stopline_marker.points.size() - 1).x;
  stopline_marker.pose.position.y =
      stopline_marker.points.at(stopline_marker.points.size() - 1).y;
  stopline_marker.pose.position.z =
      stopline_marker.points.at(stopline_marker.points.size() - 1).z + 1;
}

geometry_msgs::Point to_geoPoint(const vector_map_msgs::Point &vp) {
  geometry_msgs::Point gp;
  gp.x = vp.ly;
  gp.y = vp.bx;
  gp.z = vp.h;
  return gp;
}

void createStoplineMarkerByLane() {
  int i = 0;

  visualization_msgs::Marker stopline_marker;
  stopline_marker.header.frame_id = "map";
  stopline_marker.header.stamp = ros::Time();
  stopline_marker.ns = "stopline";
  stopline_marker.type = visualization_msgs::Marker::LINE_LIST;
  //	stopline_marker.action = visualization_msgs::Marker::DELETE;
  //	g_local_waypoints_marker_array.markers.push_back(stopline_marker);

  stopline_marker.action = visualization_msgs::Marker::ADD;
  stopline_marker.scale.x = 1;
  stopline_marker.scale.y = 1;
  stopline_marker.scale.z = 1;
  stopline_marker.color.a = 1;
  stopline_marker.color.r = 1;
  stopline_marker.color.g = 0.5;
  stopline_marker.color.b = 0.5;
  stopline_marker.lifetime = ros::Duration(10);

  visualization_msgs::Marker based_marker = stopline_marker;
  geometry_msgs::Point null_pt;

  std::vector<StopLine> stoplines =
      g_vmap.findByFilter([](const StopLine &stopline) { return true; });
  for (auto &stopline : stoplines) {
    stopline_marker = based_marker;
    if (g_vmap.findByKey(Key<RoadSign>(stopline.signid)).type ==
        vector_map_msgs::RoadSign::TYPE_STOP) {
      stopline_marker.color.r = 1;
      stopline_marker.color.g = 0;
      stopline_marker.color.b = 1;
    } else {
      stopline_marker.color = based_marker.color;
    }
    geometry_msgs::Point bp = to_geoPoint(g_vmap.findByKey(
        Key<Point>(g_vmap.findByKey(Key<Line>(stopline.lid)).bpid)));
    geometry_msgs::Point fp = to_geoPoint(g_vmap.findByKey(
        Key<Point>(g_vmap.findByKey(Key<Line>(stopline.lid)).fpid)));

    stopline_marker.points.push_back(bp);
    stopline_marker.points.push_back(fp);
    stopline_marker.pose.position = null_pt;
    stopline_marker.type = visualization_msgs::Marker::LINE_STRIP;
    stopline_marker.id = i++;
    stopline_marker.ns = "stopline";
    g_local_waypoints_marker_array.markers.push_back(stopline_marker);

    geometry_msgs::Point cp;
    cp.x = (bp.x * 2 + fp.x) / 3;
    cp.y = (bp.y * 2 + fp.y) / 3;

    stopline_marker.ns = "label";
    stopline_marker.pose.position = cp;
    stopline_marker.pose.position.z =
        stopline_marker.points.at(stopline_marker.points.size() - 1).z + 2;
    /*
    stopline_marker.pose.position.x =
    stopline_marker.points.at(stopline_marker.points.size()-1).x;
    stopline_marker.pose.position.y =
    stopline_marker.points.at(stopline_marker.points.size()-1).y;
    stopline_marker.pose.position.z =
    stopline_marker.points.at(stopline_marker.points.size()-1).z + 1;
    */
    stopline_marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
    stopline_marker.text =
        std::to_string(stopline.id) + "::" + std::to_string(stopline.signid) +
        "::" +
        std::to_string(g_vmap.findByKey(Key<RoadSign>(stopline.signid)).type);
    stopline_marker.id = i++;

    stopline_marker.points.clear();
    stopline_marker.ns = "a";
    g_local_waypoints_marker_array.markers.push_back(stopline_marker);
    stopline_marker.id = i++;
    stopline_marker.scale.x = 0.5;
    stopline_marker.scale.y = 0.5;
    stopline_marker.scale.z = 0.5;

    stopline_marker.pose.position.z -= 1;
    stopline_marker.type = visualization_msgs::Marker::CUBE;
    g_local_waypoints_marker_array.markers.push_back(stopline_marker);
  }
}

void configParameter(const std_msgs::Int32 &msg) {
  g_config_count = msg.data;
  g_local_waypoints_marker_array.markers.clear();

  createWaypointMarker(aLaneArray);
  createWaypointLabelMarker(aLaneArray);
  publishMarker();
}

void callbackDynamicParam(WaypointDebugger::WaypointDebuggerConfig &config,
                          uint32_t level) {
  g_config_count = config.int_param;
  ROS_INFO("set dynamicparam");
}

void laneArrayCallback(const autoware_msgs::LaneArrayConstPtr &msg) {
  g_local_waypoints_marker_array.markers.clear();

  aLaneArray = *msg;

  createWaypointMarker(*msg);
  createWaypointLabelMarker(*msg);

  publishMarker();
}

//# Ver 1.00
// int32 did
// float64 dist
// int32 pid
// float64 dir
// float64 apara
// float64 r
// float64 slope
// float64 cant
// float64 lw
// float64 rw
//

int closestWaypointID_ = -1;
visualization_msgs::MarkerArray areas_marker_array;

std::vector<geometry_msgs::Point>
getNearestDTLane(geometry_msgs::PointStamped pt) {
  std::vector<geometry_msgs::Point> ret;
  std::vector<DTLane> DTLanes =
      g_vmap.findByFilter([](const DTLane &dtlane) { return true; });

  geometry_msgs::PointStamped current_pt_map;

  tflistener->transformPoint("map", pt, current_pt_map);

  amathutils::point current_point(current_pt_map.point.x,
                                  current_pt_map.point.y, 0.0);
  vector_map_msgs::DTLane closest_dtlane;
  vector_map_msgs::Point closest_pid;
  double closest_distance = 999999.999;
  for (auto &dtlane : DTLanes) {
    vector_map_msgs::Point dtlane_pid =
        g_vmap.findByKey(Key<Point>(dtlane.pid));
    amathutils::point p1(dtlane_pid.ly, dtlane_pid.bx, 0.0);
    double distance = amathutils::find_distance(&current_point, &p1);
    if (distance < closest_distance) {
      closest_dtlane = dtlane;
      closest_pid = dtlane_pid;
      closest_distance = distance;
    }
  }

  std::vector<Lane> Lanes =
      g_vmap.findByFilter([](const Lane &lane) { return true; });
  std::vector<Lane> Lanes_lnids = g_vmap.findByFilter(
      [&](const Lane &lane) { return lane.did == closest_dtlane.did; });

  if (Lanes_lnids.empty())
    return std::vector<geometry_msgs::Point>();

  Lane target_lane = Lanes_lnids.at(0);
  Lane prev_lane = g_vmap.findByKey(Key<Lane>(target_lane.lnid));
  do {
    DTLane _dtlane = g_vmap.findByKey(Key<DTLane>(prev_lane.did));
    Point lane_pid = g_vmap.findByKey(Key<Point>(_dtlane.pid));
    geometry_msgs::Point pt = to_geoPoint(lane_pid);
    ret.insert(ret.begin(), pt);
    prev_lane = g_vmap.findByKey(Key<Lane>(prev_lane.blid));
  } while (prev_lane.lnid != 0 && prev_lane.jct == 0);

  Lane next_lane = g_vmap.findByKey(Key<Lane>(target_lane.lnid));
  do {
    DTLane _dtlane = g_vmap.findByKey(Key<DTLane>(next_lane.did));
    Point lane_pid = g_vmap.findByKey(Key<Point>(_dtlane.pid));
    geometry_msgs::Point pt = to_geoPoint(lane_pid);
    ret.push_back(pt);
    next_lane = g_vmap.findByKey(Key<Lane>(next_lane.flid));
  } while (next_lane.lnid != 0 && next_lane.jct == 0);

  return ret;
}
void clickedPointCallback(const geometry_msgs::PointStamped &msg) {
  ROS_INFO("[%s]", __func__);

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
  areas_marker_array.markers.push_back(dtlane_line_marker);
  area_pub.publish(areas_marker_array);
  areas_marker_array.markers.clear();
}
void currentPoseCallback(const geometry_msgs::PoseStamped &msg) {
  geometry_msgs::PointStamped pt;
  pt.header = msg.header;
  pt.point = msg.pose.position;
  clickedPointCallback(pt);
}
void closestWaypointCallback(const std_msgs::Int32 &msg) {
  closestWaypointID_ = msg.data;
}
}
using namespace autoware_debug;

int main(int argc, char **argv) {
  ros::init(argc, argv, "waypoints_debugger");
  ros::NodeHandle nh;

  g_vmap.subscribe(nh,
                   Category::POINT | Category::LINE | Category::VECTOR |
                       Category::AREA | Category::POLE | // basic class
                       Category::DTLANE | Category::LANE | Category::NODE |
                       Category::STOP_LINE | Category::ROAD_SIGN);

  tflistener = new tf::TransformListener();
  // subscribe global waypoints
  ros::Subscriber lane_array_sub =
      nh.subscribe("lane_waypoints_array", 10, laneArrayCallback);
  ros::Subscriber closest_sub =
      nh.subscribe("closest_waypoint", 1, closestWaypointCallback);
  ros::Subscriber current_pose_sub =
      nh.subscribe("current_pose", 1, currentPoseCallback);
  ros::Subscriber config_sub =
      nh.subscribe("config/debug/waycount", 10, configParameter);
  ros::Subscriber clicked_sub =
      nh.subscribe("/clicked_point", 10, clickedPointCallback);
  g_global_mark_pub = nh.advertise<visualization_msgs::MarkerArray>(
      "waypoints_debug", 10, true);

  area_pub =
      nh.advertise<visualization_msgs::MarkerArray>("areas_debug", 10, true);

  dynamic_reconfigure::Server<WaypointDebugger::WaypointDebuggerConfig> server;
  dynamic_reconfigure::Server<
      WaypointDebugger::WaypointDebuggerConfig>::CallbackType f =
      boost::bind(&callbackDynamicParam, _1, _2);
  server.setCallback(f);

  // initialize path color
  _initial_color.b = 0.0;
  _initial_color.g = 0.7;
  _initial_color.r = 1.7;
  _global_color = _initial_color;
  _global_color.a = g_global_alpha;
  g_local_color = _initial_color;
  g_local_color.a = g_local_alpha;

  ros::Rate r(2);
  while (ros::ok()) {
    ros::spinOnce();
    createStoplineMarkerByLane();
    createWaypointMarker(aLaneArray);
    createWaypointLabelMarker(aLaneArray);
    publishMarker();
    r.sleep();
  }
}
