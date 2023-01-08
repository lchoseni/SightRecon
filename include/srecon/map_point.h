#ifndef MAP_POINT_H
#define MAP_POINT_H

#include "common_include.h"
#include "feature.h"
#include "frame.h"

namespace srecon {

class Feature;

class MapPoint {
 private:
 public:
  typedef shared_ptr<MapPoint> Ptr;

  // frame - feature
  vector<std::weak_ptr<Feature>> features;
  // World coordinate
  Eigen::Vector3d w_pos;
  bool is_outliner;
  MapPoint(std::shared_ptr<Feature> &feature1,
           std::shared_ptr<Feature> &feature2, double x, double y, double z);

  bool reset();
  ~MapPoint(){};
};
}  // namespace srecon

#endif
