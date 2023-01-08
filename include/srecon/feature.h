#ifndef FEATURE_H
#define FEATURE_H

#include <opencv2/opencv.hpp>

#include "common_include.h"
#include "frame.h"
#include "map_point.h"

namespace srecon {

class Frame;
class MapPoint;

class Feature {
 private:
  std::shared_ptr<MapPoint> map_point;

 public:
  typedef std::shared_ptr<Feature> Ptr;

  weak_ptr<Frame> frame;
  double x, y, norm_x, norm_y;
  Feature();

  Feature(std::shared_ptr<Frame> &frame, cv::KeyPoint &kp);

  void setMapPoint(std::shared_ptr<MapPoint> &mp);

  std::shared_ptr<MapPoint> getMapPoint();

  ~Feature();
};

}  // namespace srecon

#endif
