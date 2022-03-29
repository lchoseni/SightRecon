#ifndef FRONT_END_H
#define FRONT_END_H

#include <opencv2/features2d.hpp>

#include "common_include.h"
#include "frame.h"
#include "camera.h"

namespace sslam {
class FrontEnd {
 public:
  std::vector<Frame> frames_;

  cv::Ptr<cv::GFTTDetector> gftt_;

  std::shared_ptr<Camera> left_camera_, right_camera_;

  FrontEnd(/* args */);

  ~FrontEnd();

  /*
   * Initilize system.
   */
  bool Initialize();

  bool AddFrame(shared_ptr<Frame> &frame);

  bool DetectFeatures(shared_ptr<Frame> frame);

  int FindFeaturesInRight(shared_ptr<Frame> frame);

  bool InitMap();

  bool Triangulation(Sophus::SE3d &T, std::vector<Feature::Ptr> &pt1, std::vector<Feature::Ptr> &pt2,
                     std::vector<cv::Point3d> &points);
};

}

#endif
