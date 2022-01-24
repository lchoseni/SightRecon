#include "SSLAM/feature.h"

namespace sslam {
    Feature::Feature(std::shared_ptr<Frame> frame, cv::KeyPoint &kp) : frame_(frame), key_point_(kp) {
    }
}