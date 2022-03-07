#include "SSLAM/frame.h"

namespace sslam {
    unsigned int Frame::global_index = 0;

    Frame::Frame() : id_(GetNextIndex()) {
    }

    Frame::~Frame() {
    }


    unsigned int Frame::GetNextIndex() {
        return ++global_index;
    }

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
        this->cam = sharedPtr;
    }

    std::shared_ptr<Camera> Frame::GetCamera() {
        return cam;
    }

}