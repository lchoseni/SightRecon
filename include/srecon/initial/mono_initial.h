#ifndef MONO_INITIAL_H
#define MONO_INITIAL_H

#include <Eigen/Core>

#include "../frame.h"
#include "../map.h"
#include "initial.h"

namespace srecon {

class MonoInitial : public Initial {
 private:
 public:
  ImageDataset::Ptr dataset;
  Map::Ptr map;
  MonoInitial(ImageDataset::Ptr dataset, Map::Ptr &map,
              cv::Ptr<cv::GFTTDetector> detecor);
  ~MonoInitial();

  bool computeRT(Frame::Ptr &frame1, Frame::Ptr &frame2,
                 vector<cv::KeyPoint> &kps1, vector<cv::KeyPoint> &kps2,
                 vector<cv::Point2f> &kps1_2d, vector<cv::Point2f> &kps2_2d,
                 vector<cv::Point2f> &fkps1_2d, vector<cv::Point2f> &fkps2_2d,
                 cv::Mat &cv_R, cv::Mat &cv_T, cv::Mat &inlinerMask);
  int init(Frame::Ptr &result_frame1, Frame::Ptr &result_frame2, double scale);
};

}  // namespace srecon

#endif