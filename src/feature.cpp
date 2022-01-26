#include "SSLAM/feature.h"

namespace sslam {

    Feature::Feature(){

    }

    Feature::Feature(Frame frame, cv::KeyPoint &kp) : frame_(&frame), key_point_(kp) {
    }

    Feature::~Feature() {

    }
}