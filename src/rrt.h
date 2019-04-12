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
  bool Intersection(const Eigen::Vector2f& p2,
                    const Eigen::Vector2f& p3,
                    Eigen::Vector2f* intersection) const;
};

struct Map {
  std::vector<Line> lines;
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
struct CarMove {
  float distance;
  float curvature;
  CarMove() {}
  CarMove(float d, float c) : distance(d), curvature(c) {}
};

CarPose ApplyMove(const CarPose& p, const CarMove& m);

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
