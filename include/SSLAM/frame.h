#ifndef FRAME_H
#define FRAME_H

#include<opencv2/opencv.hpp>

#include "feature.h"

namespace sslam
{

    class Frame
    {
    private:
        // index for all frames, it indicates next frame id.
        static unsigned int global_index;
        // id of current frame
        unsigned int id_;
        SE3 pose_;
        std::vector<Feature> features_;

    public:
        cv::Mat img_, left_img_, right_img_;
        std::vector<Feature> left_features_;
        std::vector<Feature> right_features_;
        Frame();
        ~Frame();
        static unsigned int GetNextIndex();
    };

}

#endif