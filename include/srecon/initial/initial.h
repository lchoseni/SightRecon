#ifndef INITIAL_H
#define INITIAL_H

#include "../camera.h"
#include "../common_include.h"
#include "../dataset/image_dataset.h"
#include "../frame.h"

namespace srecon {

class Initial {
 public:
  std::vector<Frame::Ptr> frames;

  cv::Ptr<cv::GFTTDetector> detecor;

  Camera::Ptr camera;

  Initial(cv::Ptr<cv::GFTTDetector> detecor) : detecor(detecor) {}
  ~Initial() {}

  virtual int init(Frame::Ptr &result_frame1, Frame::Ptr &result_frame2,
                   double scale) = 0;
};

}  // namespace srecon

#endif