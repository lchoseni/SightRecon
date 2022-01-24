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

    bool Frame::SetLeftKP(std::vector <cv::KeyPoint> &kps) {
        left_key_points_ = kps;
        return true;
    }

}