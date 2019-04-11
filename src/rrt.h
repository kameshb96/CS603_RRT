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

struct Obstacle {
  Eigen::Vector2f p0;
  Eigen::Vector2f p1;
  Eigen::Vector2f p2;
  Eigen::Vector2f p3;
  Obstacle() {}
  Obstacle(const Eigen::Vector2f& p0,
           const Eigen::Vector2f& p1,
           const Eigen::Vector2f& p2,
           const Eigen::Vector2f& p3) : p0(p0), p1(p1), p2(p2), p3(p3) {}
  Obstacle(const Eigen::Vector2f points[4]) :
      p0(points[0]), p1(points[1]), p2(points[2]), p3(points[3]) {}
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

typedef std::vector<Obstacle> ObstacleMap;


void PublishVisualization(
    const ObstacleMap& map,
    const CarPose& start,
    const CarPose& goal,
    const std::vector<std::pair<CarPose, CarMove>>& tree_edges,
    const std::vector<CarMove>& path);

// Build an RRT to plan a path to get from start to goal, avoiding the obstacles
// in the map. Set path to the solution found, if one exists. If no solution
// is found, return false, else return true.
bool RRTPlan(const ObstacleMap& map,
             const CarPose& start,
             const CarPose& goal,
             std::vector<CarMove>* path);

}  // namespace COMPSCI603

#endif   // __RRT_H__
