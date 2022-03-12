#ifndef FEATURE_H
#define FEATURE_H

#include <opencv2/opencv.hpp>

#include "common_include.h"
#include "frame.h"


namespace sslam {

    class Frame;

    class Feature {
    private:
        shared_ptr<Frame> frame_;

    public:
        typedef std::shared_ptr<Feature> Ptr ;

        cv::KeyPoint key_point_;

        bool is_on_left_image_;

        Feature();

        Feature(shared_ptr<Frame> frame, cv::KeyPoint &kp);

        ~Feature();
    };

}

#endif
