#include "SSLAM/frame.h"

namespace sslam {

Frame::Frame(int id): id_(id) {
}

//    Frame::~Frame() {
//      cout<< "invoke" << endl;
//    }



bool Frame::SetLeftKP(std::vector<cv::KeyPoint> &kps) {
  left_key_points_ = kps;
  return true;
}

void Frame::DrawKeyPoints() {
  cv::Scalar scalar = cv::Scalar();

  cv::Mat out_left, out_right;
  cv::drawKeypoints(left_img_, left_key_points_, out_left);
  cv::drawKeypoints(right_img_, right_key_points_, out_right);

  cv::imshow("img1", out_left);
  cv::imshow("img2", out_right);
  cv::waitKey(0);
}

void Frame::SetCamera(std::shared_ptr<Camera> &sharedPtr) {
  this->cam_ = sharedPtr;
}

std::shared_ptr<Camera> Frame::GetCamera() {
  return cam_;
}

bool Frame::isInFrame(const Vec3 &pt_world, const SE3 &pose) {

  Vec2 p_cam = cam_->World2Pixel(pt_world, pose);
  return p_cam(0, 0) > 0 && p_cam(1, 0) > 0
      && p_cam(0, 0) < left_img_.cols
      && p_cam(1, 0) < left_img_.rows;
}

}