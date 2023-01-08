//
// Created by yeren on 1/25/22.
//

#include "srecon/map_point.h"

#include "srecon/utils/utils.h"
namespace srecon {

MapPoint::MapPoint(Feature::Ptr &feature1, Feature::Ptr &feature2, double x,
                   double y, double z) {
  w_pos = Eigen::Vector3d(x, y, z);
  is_outliner = false;
  features.push_back(weak_ptr<Feature>(feature1));
  features.push_back(weak_ptr<Feature>(feature2));
}

// remove corresponding features and related mappoint in feature.
bool MapPoint::reset() {
  is_outliner = true;
  for (auto &feature : features) {
    auto fea_ptr = feature.lock();
    if (fea_ptr) {
      fea_ptr->getMapPoint().reset();
    }
    feature.reset();
  }
  // remove null features.
  std::remove_if(features.begin(), features.end(),
                 is_uninitialized<Feature>);
  return true;
}

}  // namespace srecon