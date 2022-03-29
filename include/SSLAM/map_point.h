#ifndef MAP_POINT_H
#define MAP_POINT_H

#include <utility>

#include "common_include.h"
//#include "feature.h"
#include "frame.h"

namespace sslam {

class Frame;

class MapPoint {

 public:

  typedef shared_ptr<MapPoint> Ptr;
  cv::Mat descriptor; // Descriptor for matching
  list<std::pair<shared_ptr<Frame>, cv::Point2d>> observedFrames;//观测帧和像素坐标
  Vec3 pos;       // Position in world
  cv::Vec3b rgb;

  MapPoint();

  MapPoint(const Vec3 &pos, const cv::Mat &descriptor, const cv::Vec3b &rgb) :
      descriptor(descriptor),
      pos(pos),
      rgb(rgb) {}

  template<typename T>
  cv::Point3_<T> getPosPoint3_CV() const {
    return cv::Point3_<T>(pos(0, 0), pos(1, 0), pos(2, 0));
  }

  template<typename T>
  cv::Matx<T, 1, 3> getPosMatx13() const {
    return cv::Matx<T, 1, 3>(pos(0, 0), pos(1, 0), pos(2, 0));
  };

  template<typename T>
  void setPos(cv::Matx<T, 1, 3> posMatx13) {
    pos(0) = posMatx13(0);
    pos(1) = posMatx13(1);
    pos(2) = posMatx13(2);
  }

  void addObervedFrame(const shared_ptr<Frame> &observedFrame, const cv::Point2d &pixelCoor) {
    if (observedFrame)
      observedFrames.push_back(
          std::pair<shared_ptr<Frame>, cv::Point2d>(observedFrame, pixelCoor));
  }
};
}

#endif