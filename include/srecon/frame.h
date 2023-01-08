#ifndef FRAME_H
#define FRAME_H

#include <opencv2/opencv.hpp>

#include "camera.h"
#include "common_include.h"
#include "feature.h"
#include "map_point.h"

namespace srecon {

class Feature;

class Frame {
 public:
  typedef std::shared_ptr<Frame> Ptr;
  // index for all frames, it indicates next frame id.
  static unsigned int global_index;
  // id of current frame
  unsigned int id;
  unsigned int time;
  Camera::Ptr cam;
  Eigen::Matrix3d R;
  Eigen::Vector3d T;
  Eigen::Matrix3d gt_R;
  Eigen::Vector3d gt_T;
  std::vector<std::shared_ptr<Feature>> features;

  cv::Mat img;
  ;
  // Key point matches between left img and right img.

  Frame();

  void SetCamera(Camera::Ptr sharedPtr);

  Camera::Ptr GetCamera();

  void detectFeature(cv::Ptr<cv::GFTTDetector> &detecor,
                     vector<cv::KeyPoint> &kps);

  static unsigned int GetNextIndex();
};

}  // namespace srecon

#endif
