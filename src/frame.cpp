#include "srecon/frame.h"

#include "srecon/config.h"

namespace srecon {
unsigned int Frame::global_index = 0;

Frame::Frame() : id(GetNextIndex()) {}

//    Frame::~Frame() {
//      cout<< "invoke" << endl;
//    }

unsigned int Frame::GetNextIndex() {
  global_index++;
  return global_index - 1;
}

void Frame::SetCamera(std::shared_ptr<Camera> sharedPtr) {
  this->cam = sharedPtr;
}

std::shared_ptr<Camera> Frame::GetCamera() { return cam; }

void Frame::detectFeature(cv::Ptr<cv::GFTTDetector> &detecor,
                          vector<cv::KeyPoint> &kps) {
  if (features.size() < (size_t)Config::Get<int>(Config::num_features)) {
    cv::Mat mask(img.size(), CV_8UC1, 255);
    int rect_size = Config::Get<int>(Config::rect_size);
    cv::Point2d rect(rect_size, rect_size);
    for (auto &fea : features) {
      cv::Point2d pt(fea->y, fea->x);
      cv::rectangle(mask, pt - rect, pt + rect, 0, CV_FILLED);
    }

    detecor->detect(img, kps, mask);
  }
}

}  // namespace srecon