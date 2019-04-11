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
using gui_helpers::Color4f;
using COMPSCI603::CarMove;
using COMPSCI603::CarPose;
using COMPSCI603::Obstacle;
using COMPSCI603::ObstacleMap;
using visualization_msgs::Marker;

ObstacleMap map_;
ros::Publisher map_publisher_;
ros::Publisher tree_publisher_;
ros::Publisher path_publisher_;
visualization_msgs::Marker map_msg_;
visualization_msgs::Marker tree_msg_;
visualization_msgs::Marker path_msg_;

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
  path_msg_.type = visualization_msgs::Marker::LINE_STRIP;
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
}

CarPose ApplyMove(const CarPose& p, const CarMove& m) {
  CarPose p2 = p;
  const float dtheta = m.distance * m.curvature;
  p2.angle += dtheta;

  if (fabs(dtheta) > kEpsilon) {
    const float r = 1.0 / m.curvature;
    using Eigen::Rotation2Df;
    const Vector2f center = Rotation2Df(p.angle) * Vector2f(0, r) + p.loc;
    using Eigen::Translation2f;
    p2.loc = Translation2f(center) * Rotation2Df(dtheta) *
        Translation2f(-center) * p.loc;
  }
  return p2;
}

void DrawMove(const CarPose& p, const CarMove& m, Marker* msg) {
  const float kDistIncr = 0.01;
  const float dtheta = m.distance * m.curvature;

  const CarPose p1 = ApplyMove(p, m);
  if (fabs(dtheta) > kEpsilon) {
    // Draw an arc.
    const CarMove incr_move(kDistIncr, m.curvature);
    const int n = floor(m.distance / kDistIncr);
    CarPose p0 = p;
    // Draw n equal length segments along the arc.
    for (int i = 0; i < n; ++i) {
      const CarPose p1 = ApplyMove(p0, incr_move);
      AddLine(p0.loc, p1.loc, Color4f(0.3, 0.3, 0.3, 0.5), msg);
      p0 = p1;
    }
    // Final segment to finish the arc.
    AddLine(p0.loc, p1.loc, Color4f(0.3, 0.3, 0.3, 0.5), msg);
  } else {
    // Straight line.
    AddLine(p.loc, p1.loc, Color4f(0.3, 0.3, 0.3, 0.5), msg);
  }
}

namespace COMPSCI603 {
void PublishVisualization(const ObstacleMap& map,
                          const CarPose& start,
                          const CarPose& goal,
                          const vector<pair<CarPose, CarMove>>& tree_edges,
                          const vector<CarMove>& path) {
  static double t_last = 0;
  if (GetMonotonicTime() - t_last < 0.016) {
    // Rate-limit visualization.
    return;
  }
  t_last = GetMonotonicTime();

  ClearMarker(&map_msg_);
  ClearMarker(&tree_msg_);
  ClearMarker(&path_msg_);
  for (const Obstacle& o : map) {
    const Vector2f& p0(o.p0);
    const Vector2f& p1(o.p1);
    const Vector2f& p2(o.p2);
    const Vector2f& p3(o.p3);
    AddLine(p0, p1, Color4f(0.4, 0.4, 1.0, 1), &map_msg_);
    AddLine(p1, p2, Color4f(0.4, 0.4, 1.0, 1), &map_msg_);
    AddLine(p2, p3, Color4f(0.4, 0.4, 1.0, 1), &map_msg_);
    AddLine(p3, p0, Color4f(0.4, 0.4, 1.0, 1), &map_msg_);
  }

  CarPose p = start;
  for (const CarMove& m : path) {
    DrawMove(p, m, &path_msg_);
    p = ApplyMove(p, m);
  }

  for (const pair<CarPose, CarMove>& e : tree_edges) {
    DrawMove(e.first, e.second, &tree_msg_);
  }
}
}  // namespace COMPSCI603

void LoadMap(const string& file) {
  FILE* fid = fopen(file.c_str(), "r");
  if (fid == NULL) {
    fprintf(stderr, "ERROR: Unable to load map %s\n", file.c_str());
    exit(1);
  }

  bool load_complete = false;
  while (!load_complete) {
    int i = 0;
    Vector2f points[4];
    for (; i < 4; ++i) {
      if (fscanf(fid, "%f,%f", &(points[i].x()), &(points[i].y())) != 2) {
        break;
      }
    }
    if (i == 0) {
      // No more lines to read.
      load_complete = true;
    } else if (i == 4) {
      // Succesfully loaded one more obstacle.
      map_.push_back(Obstacle(points));
      if (FLAGS_v > 2) {
        printf("Obstacle: %f,%f %f,%f %f,%f %f,%f\n",
               map_.back().p0.x(),
               map_.back().p0.y(),
               map_.back().p1.x(),
               map_.back().p1.y(),
               map_.back().p2.x(),
               map_.back().p2.y(),
               map_.back().p3.x(),
               map_.back().p3.y());
      }
    } else {
      // Something's wrong: interrupted mid-line.
      fclose(fid);
      fprintf(stderr, "ERROR: malformed map file %s\n", file.c_str());
      exit(2);
    }
  }
  fclose(fid);

}

int main(int argc, char** argv) {
  google::ParseCommandLineFlags(&argc, &argv, false);
  if (argc != 8) {
    fprintf(stderr,
            "USAGE: ./bin/rrt start_x start_y start_r "
            "goal_x goal_y goal_r mapfile\n");
    return 1;
  }
  // Initialize ROS.
  ros::init(argc, argv, "rrt");
  ros::NodeHandle n;
  LoadMap(argv[7]);
  InitializeMsgs();
  map_publisher_ = n.advertise<visualization_msgs::Marker>("map", 1);
  tree_publisher_ = n.advertise<visualization_msgs::Marker>("tree", 1);
  path_publisher_ = n.advertise<visualization_msgs::Marker>("path", 1);
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
