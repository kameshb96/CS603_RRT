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
  * \brief   COMPSCI603 RRT Implementation
  * \author  Joydeep Biswas, (C) 2019
  */
//========================================================================

#include "rrt.h"

#include <float.h>
#include <math.h>

#include <algorithm>
#include <random>
#include <vector>

#include "eigen3/Eigen/Dense"
#include "eigen3/Eigen/Geometry"

#include "math/math_util.h"

using Eigen::Vector2f;
using std::min;
using std::max;
using std::pair;
using std::make_pair;
using std::vector;
using math_util::DegToRad;
using math_util::AngleDist;


namespace COMPSCI603 {

// Returns the cross product between 2d vectors
float Cross(const Vector2f& v1, const Vector2f& v2) {
  return (v1.x() * v2.y() - v1.y() * v2.x());
}

// Returns true and populates the location at intercept if this line collides
// with the line specified by p2 and p3
bool Line::Intersection(const Vector2f& p2,
                        const Vector2f& p3,
                        Vector2f* intersection) const {
  // Bounding-box broad phase check.
  if (min(p0.x(), p1.x()) > max(p2.x(), p3.x())) return false;
  if (max(p0.x(), p1.x()) < min(p2.x(), p3.x())) return false;
  if (min(p0.y(), p1.y()) > max(p2.y(), p3.y())) return false;
  if (max(p0.y(), p1.y()) < min(p2.y(), p3.y())) return false;
  // Narrow-phase check.
  const Vector2f d1 = p1 - p0;
  const Vector2f d2 = p3 - p2;
  if (Cross(d1, p3 - p0) * Cross(d1, p2 - p0) >= 0.0) return false;
  if (Cross(d2, p1 - p2) * Cross(d2, p0 - p2) >= 0.0) return false;
  // Okay, the line segments definitely intersect.
  const float d = Cross(d2, -d1);
  // Just an extra check, should never happen
  if (d == 0.0f) return false;
  const float tb = Cross(p0 -p2, p0-p1) / d;
  *intersection = p2 + tb * d2;
  return true;
}

// Helper Function
// Given an initial pose p and a move m, return the next pose
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

// Build an RRT to plan a path to get from start to goal, avoiding the obstacles
// in the map. Set path to the solution found, if one exists. If no solution
// is found, return false, else return true.
bool RRTPlan(const Map& map,
             const CarPose& start,
             const CarPose& goal,
             vector<CarMove>* path) {

  // TODO: Fill in this function

  return false;
}

}  // namespace COMPSCI603
