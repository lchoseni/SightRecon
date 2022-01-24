#ifndef FEATURE_H
#define FEATURE_H

#include <opencv2/opencv.hpp>

#include "common_include.h"
#include "frame.h"


namespace sslam {

    class Frame;

    class Feature {
    private:
        cv::KeyPoint key_point_;
        std::weak_ptr<sslam::Frame> frame_;
        bool is_on_left_image_;

    public:
        Feature(std::shared_ptr<Frame> frame, cv::KeyPoint &kp);

        ~Feature();
    };

}

#endif