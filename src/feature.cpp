#include "SSLAM/feature.h"

namespace sslam {

    Feature::Feature(){

    }

    Feature::Feature(shared_ptr<Frame> frame, cv::KeyPoint &kp) : frame_(frame), key_point_(kp) {
        Feature::Ptr a;
    }

    Feature::~Feature() {

    }
}