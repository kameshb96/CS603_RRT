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
  * \file    rrt.h
  * \brief   COMPSCI603 RRT Interface
  * \author  Joydeep Biswas, (C) 2019
  */
//========================================================================

#include <vector>

#include "eigen3/Eigen/Dense"

#ifndef __RRT_H__
#define __RRT_H__

static const float kEpsilon = 1e-6;


namespace COMPSCI603 {


struct Line {
  Eigen::Vector2f p0;
  Eigen::Vector2f p1;
  Line() {}
  Line(const Eigen::Vector2f& p0, const Eigen::Vector2f& p1) : p0(p0), p1(p1) {}

  // Returns true and populates the location at intercept if this line collides
  // with the line specified by p2 and p3
  bool Intersection(const Eigen::Vector2f& p2,
                    const Eigen::Vector2f& p3,
                    Eigen::Vector2f* intersection) const;
};

struct Map {
  // A set of lines representing the obstacles in the map
  std::vector<Line> lines;

  // The minimum and maximum X and Y values of the map
  float min_x;
  float min_y;
  float max_x;
  float max_y;
};

// The pose of the car, as defined in the handout.
struct CarPose {
  Eigen::Vector2f loc;
  float angle;
  CarPose() {}
  CarPose(const Eigen::Vector2f& loc, const float angle) :
      loc(loc), angle(angle) {}
};

// A move of the car, defined by its curvature (1 / radius) and the distance
// traversed along it. Positive curvature => ccw rotation, negative => cw.
// Note: The car can travel straight (curvature = 0, radius = inf)
struct CarMove {
  float distance;
  float curvature;
  CarMove() {}
  CarMove(float d, float c) : distance(d), curvature(c) {}
};

// Helper Function
// Given an initial pose p and a move m, return the next pose
CarPose ApplyMove(const CarPose& p, const CarMove& m);

// Publish a visualization of the current state of the algorithm
// Start: The initial pose as passed in to RRTPlan, this will be drawn in Red
// Goal: The goal pose as passed in to RRTPlan this will be drawn in Green
// Current: The most recent node to be added to the tree, will be drawn in blue
// Tree Edges: The edges that exist in the current tree. An edge here is
//   defined as an initial pose and a movement to the next pose. The edges will
//   be drawn via displaying the arc defined by the moves
// Path: A path from start to another node, represented as a series of car
//   moves. This will be drawn as a series of arks
void PublishVisualization(
    const CarPose& start,
    const CarPose& goal,
    const CarPose& current,
    const std::vector<std::pair<CarPose, CarMove>>& tree_edges,
    const std::vector<CarMove>& path);

// Build an RRT to plan a path to get from start to goal, avoiding the obstacles
// in the map. Set path to the solution found, if one exists. If no solution
// is found, return false, else return true.
bool RRTPlan(const Map& map,
             const CarPose& start,
             const CarPose& goal,
             std::vector<CarMove>* path);

}  // namespace COMPSCI603

#endif   // __RRT_H__
