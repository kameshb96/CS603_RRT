//========================================================================
//  This software is free: you can redistribute it and/or modify
//  it under the terms of the GNU Lesser General Public License Version 3,
//  as published by the Free Software Foundation.
//
//  This software is distributed in the hope that it will be useful,
//  but WITHOUT ANY WARRANTY; without even the implied warranty of
//  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
//  GNU Lesser General Public License for more details.
//
//  You should have received a copy of the GNU Lesser General Public License
//  Version 3 in the file COPYING that came with this distribution.
//  If not, see <http://www.gnu.org/licenses/>.
//========================================================================
/*!
\file    rrt_main.cc
\brief   Main entry point for COMPSCI603 assignment on RRT
\author  Joydeep Biswas, (C) 2019
*/
//========================================================================

#include <signal.h>
#include <stdlib.h>
#include <math.h>
#include <stdio.h>
#include <string.h>
#include <inttypes.h>
#include <utility>
#include <vector>

#include "gflags/gflags.h"
#include "eigen3/Eigen/Dense"
#include "eigen3/Eigen/Geometry"
#include "gflags/gflags.h"
#include "geometry_msgs/PoseArray.h"
#include "visualization_msgs/Marker.h"
#include "visualization_msgs/MarkerArray.h"
#include "ros/ros.h"
#include "timer.h"
#include "gui_helpers.h"

#include "rrt.h"

DECLARE_string(helpon);
DECLARE_int32(v);

using Eigen::Vector2f;
using std::string;
using std::vector;
using std::make_pair;
using std::pair;
using gui_helpers::ClearMarker;
using gui_helpers::AddLine;
using gui_helpers::AddPoint;
using gui_helpers::Color4f;
using COMPSCI603::CarMove;
using COMPSCI603::CarPose;
using COMPSCI603::Map;
using COMPSCI603::ApplyMove;
using visualization_msgs::Marker;

Map map_;
ros::Publisher map_publisher_;
ros::Publisher tree_publisher_;
ros::Publisher path_publisher_;
ros::Publisher points_publisher_;
visualization_msgs::Marker map_msg_;
visualization_msgs::Marker tree_msg_;
visualization_msgs::Marker path_msg_;
visualization_msgs::Marker points_msg_;

geometry_msgs::Point EigenToRosPoint(const Vector2f& p) {
  geometry_msgs::Point p2;
  p2.x = p.x();
  p2.y = p.y();
  p2.z = 0;
  return p2;
}

void InitializeMsgs() {
  std_msgs::Header header;
  header.frame_id = "map";
  header.seq = 0;

  map_msg_.header = header;
  map_msg_.type = visualization_msgs::Marker::LINE_LIST;
  map_msg_.action = visualization_msgs::Marker::ADD;
  map_msg_.color.a = 1;
  map_msg_.color.r = 66.0 / 255.0;
  map_msg_.color.g = 134.0 / 255.0;
  map_msg_.color.b = 244.0 / 255.0;
  map_msg_.pose.position.x = 0;
  map_msg_.pose.position.y = 0;
  map_msg_.pose.position.z = 0;
  map_msg_.pose.orientation.x = 0.0;
  map_msg_.pose.orientation.y = 0.0;
  map_msg_.pose.orientation.z = 0.0;
  map_msg_.pose.orientation.w = 1.0;
  map_msg_.scale.x = 0.1;
  map_msg_.scale.y = 1;
  map_msg_.scale.z = 1;
  map_msg_.ns = "map";
  map_msg_.id = 0;

  tree_msg_.header = header;
  tree_msg_.type = visualization_msgs::Marker::LINE_LIST;
  tree_msg_.action = visualization_msgs::Marker::ADD;
  tree_msg_.color.a = 1;
  tree_msg_.color.r = 178.0 / 255.0;
  tree_msg_.color.g = 178.0 / 255.0;
  tree_msg_.color.b = 178.0 / 255.0;
  tree_msg_.pose.position.x = 0;
  tree_msg_.pose.position.y = 0;
  tree_msg_.pose.position.z = 0;
  tree_msg_.pose.orientation.x = 0.0;
  tree_msg_.pose.orientation.y = 0.0;
  tree_msg_.pose.orientation.z = 0.0;
  tree_msg_.pose.orientation.w = 1.0;
  tree_msg_.scale.x = 0.1;
  tree_msg_.scale.y = 1;
  tree_msg_.scale.z = 1;
  tree_msg_.ns = "tree";
  tree_msg_.id = 0;

  path_msg_.header = header;
  path_msg_.type = visualization_msgs::Marker::LINE_LIST;
  path_msg_.action = visualization_msgs::Marker::ADD;
  path_msg_.color.a = 0.2;
  path_msg_.color.r = 67.0 / 255.0;
  path_msg_.color.g = 178.0 / 255.0;
  path_msg_.color.b = 89.0 / 255.0;
  path_msg_.pose.position.x = 0;
  path_msg_.pose.position.y = 0;
  path_msg_.pose.position.z = 0;
  path_msg_.pose.orientation.x = 0.0;
  path_msg_.pose.orientation.y = 0.0;
  path_msg_.pose.orientation.z = 0.0;
  path_msg_.pose.orientation.w = 1.0;
  path_msg_.scale.x = 0.1;
  path_msg_.scale.y = 1;
  path_msg_.scale.z = 1;
  path_msg_.ns = "path";
  path_msg_.id = 0;

  points_msg_.header = header;
  points_msg_.type = visualization_msgs::Marker::LINE_LIST;
  points_msg_.action = visualization_msgs::Marker::ADD;
  points_msg_.color.a = 0.2;
  points_msg_.color.r = 67.0 / 255.0;
  points_msg_.color.g = 178.0 / 255.0;
  points_msg_.color.b = 89.0 / 255.0;
  points_msg_.pose.position.x = 0;
  points_msg_.pose.position.y = 0;
  points_msg_.pose.position.z = 0;
  points_msg_.pose.orientation.x = 0.0;
  points_msg_.pose.orientation.y = 0.0;
  points_msg_.pose.orientation.z = 0.0;
  points_msg_.pose.orientation.w = 1.0;
  points_msg_.scale.x = 0.1;
  points_msg_.scale.y = 1;
  points_msg_.scale.z = 1;
  points_msg_.ns = "points";
  points_msg_.id = 0;

}

void DrawMove(const CarPose& p,
              const CarMove& m,
              const Color4f& color,
              Marker* msg) {
  const float kDistIncr = 0.3;
  const float dtheta = m.distance * m.curvature;

  const CarPose p1 = ApplyMove(p, m);
  if (fabs(dtheta) > kEpsilon && fabs(m.distance) > kEpsilon) {
    // Draw an arc.
    const float dir = (m.distance > 0.0) ? 1.0 : -1.0;
    const CarMove incr_move(kDistIncr * dir, m.curvature);
    const int n = floor(fabs(m.distance) / kDistIncr);
    CarPose p0 = p;
    // Draw n equal length segments along the arc.
    for (int i = 0; i < n; ++i) {
      const CarPose p1 = ApplyMove(p0, incr_move);
      AddLine(p0.loc, p1.loc, color, msg);
      p0 = p1;
    }
    // Final segment to finish the arc.
    AddLine(p0.loc, p1.loc, color, msg);
  } else {
    // Straight line.
    AddLine(p.loc, p1.loc, color, msg);
  }
              }

void DrawCar(const CarPose& p,
             const Color4f& col,
             Marker* msg) {
  const Eigen::Affine2f tf =
      Eigen::Translation2f(p.loc) * Eigen::Rotation2Df(p.angle);
  const int kNumLines = 6;
  static const Vector2f car[kNumLines][2] = {
    { Vector2f(-0.5, -0.85), Vector2f(-0.5, 0.85) },
    { Vector2f(-0.5, 0.85), Vector2f(4.0, 0.85) },
    { Vector2f(4.0, -0.85), Vector2f(4.0, 0.85) },
    { Vector2f(4.0, -0.85), Vector2f(-0.5, -0.85) },
    { Vector2f(0.0, 0.0), Vector2f(0.5, 0.0) },
    { Vector2f(0.0, 0.0), Vector2f(0.0, 0.5) }
  };

  for (int i = 0; i < kNumLines; ++i) {
    AddLine(tf * car[i][0], tf * car[i][1], col, msg);
  }
}

namespace COMPSCI603 {

// Publish a visualization of the current state of the algorithm
// Start: The initial pose as passed in to RRTPlan, this will be drawn in Red
// Goal: The goal pose as passed in to RRTPlan this will be drawn in Green
// Current: The most recent node to be added to the tree, will be drawn in blue
// Tree Edges: The edges that exist in the current tree. An edge here is
//   defined as an initial pose and a movement to the next pose. The edges will
//   be drawn via displaying the arc defined by the moves
// Path: A path from start to another node, represented as a series of car
//   moves. This will be drawn as a series of arks
void PublishVisualization(const CarPose& start,
                          const CarPose& goal,
                          const CarPose& current,
                          const vector<pair<CarPose, CarMove>>& tree_edges,
                          const vector<CarMove>& path) {
  static const Color4f kTreeColor(0.3, 0.3, 0.3, 0.5);
  static const Color4f kPathColor(0.3, 0.3, 1.0, 1.0);
  static const Color4f kMapColor(0.4, 0.4, 1.0, 1.0);/*
  static double t_last = 0;
  if (GetMonotonicTime() - t_last < 0.005) {
    // Rate-limit visualization.
    return;
  }
  t_last = GetMonotonicTime();*/

  ClearMarker(&tree_msg_);
  ClearMarker(&path_msg_);
  ClearMarker(&points_msg_);

  DrawCar(start, Color4f::kRed, &points_msg_);
  DrawCar(goal, Color4f::kGreen, &points_msg_);
  DrawCar(current, Color4f::kBlue, &points_msg_);

  CarPose p = start;
  for (const CarMove& m : path) {
    DrawMove(p, m, kPathColor, &path_msg_);
    p = ApplyMove(p, m);
  }

  for (const pair<CarPose, CarMove>& e : tree_edges) {
    DrawMove(e.first, e.second, kTreeColor, &tree_msg_);
  }

  map_msg_.header.stamp = ros::Time::now();
  map_msg_.header.seq++;
  tree_msg_.header.stamp = ros::Time::now();
  tree_msg_.header.seq++;
  path_msg_.header.stamp = ros::Time::now();
  path_msg_.header.seq++;
  points_msg_.header.stamp = ros::Time::now();
  points_msg_.header.seq++;
  map_publisher_.publish(map_msg_);
  tree_publisher_.publish(tree_msg_);
  path_publisher_.publish(path_msg_);
  points_publisher_.publish(points_msg_);
  ros::spinOnce();
  Sleep(0.01);
}
}  // namespace COMPSCI603


// Load vector map from file.
// Expects the first line to be min_x min_y max_x max_y
// Subsequent lines are coordinates separated by commas: x1, y1, x2, y2
void LoadMap(const string& file) {
  FILE* fid = fopen(file.c_str(), "r");
  if (fid == NULL) {
    fprintf(stderr, "ERROR: Unable to load map %s\n", file.c_str());
    exit(1);
  }
  if (fscanf(fid, "%f %f %f %f",
            &(map_.min_x),
            &(map_.min_y),
            &(map_.max_x),
            &(map_.max_y)) != 4) {
    fprintf(stderr, "ERROR: Map parse error\n");
    exit(1);
  }
  if (FLAGS_v > 2) {
    printf("Map min: %f %f max: %f %f\n",
           map_.min_x,
           map_.min_y,
           map_.max_x,
           map_.max_y);
  }
  float x1(0), y1(0), x2(0), y2(0);
  while (fscanf(fid, "%f,%f,%f,%f", &x1, &y1, &x2, &y2) == 4) {
    map_.lines.push_back(COMPSCI603::Line(Vector2f(x1, y1), Vector2f(x2, y2)));
  }
  fclose(fid);

  for (const COMPSCI603::Line& l : map_.lines) {
    map_msg_.points.push_back(EigenToRosPoint(l.p0));
    map_msg_.points.push_back(EigenToRosPoint(l.p1));
  }
}

void SignalHandler(int signum) {
  printf("Exiting with signal %d\n", signum);
  exit(0);
}

int main(int argc, char** argv) {
  signal(SIGINT, SignalHandler);
  google::ParseCommandLineFlags(&argc, &argv, true);
  if (argc != 8) {
    fprintf(stderr,
            "USAGE: ./bin/rrt start_x start_y start_r "
            "goal_x goal_y goal_r mapfile\n");
    return 1;
  }
  // Initialize ROS.
  ros::init(argc, argv, "rrt", ros::init_options::NoSigintHandler);
  ros::NodeHandle n;
  LoadMap(argv[7]);
  InitializeMsgs();
  map_publisher_ = n.advertise<visualization_msgs::Marker>("map", 1);
  tree_publisher_ = n.advertise<visualization_msgs::Marker>("tree", 1);
  path_publisher_ = n.advertise<visualization_msgs::Marker>("path", 1);
  points_publisher_ = n.advertise<visualization_msgs::Marker>("points", 1);
  using COMPSCI603::CarMove;
  using COMPSCI603::CarPose;
  vector<CarMove> path;
  const CarPose start(Vector2f(atof(argv[1]), atof(argv[2])), atof(argv[3]));
  const CarPose goal(Vector2f(atof(argv[4]), atof(argv[5])), atof(argv[6]));
  printf("Start: %f,%f %f Goal: %f,%f %f \n",
         start.loc.x(),
         start.loc.y(),
         start.angle,
         goal.loc.x(),
         goal.loc.y(),
         goal.angle);
  COMPSCI603::RRTPlan(map_, start, goal, &path);
  return 0;
}
