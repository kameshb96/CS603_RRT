#include "rrt.h"

#include <vector>

#include "eigen3/Eigen/Dense"
#include "eigen3/Eigen/Geometry"

using Eigen::Vector2f;
using std::pair;
using std::make_pair;
using std::vector;

namespace COMPSCI603 {

bool RRTPlan(const ObstacleMap& map,
             const CarPose& start,
             const CarPose& goal,
             std::vector<CarMove>* path) {
  
  vector<pair<CarPose, CarMove>> tree_edges;
  PublishVisualization(map, start, goal, tree_edges, *path);
  return false;
}

}  // namespace COMPSCI603
