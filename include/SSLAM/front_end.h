#ifndef FRONT_END_H
#define FRONT_END_H

#include <opencv2/features2d.hpp>

#include "common_include.h"
#include "frame.h"

namespace sslam {
    class FrontEnd {
    public:
        std::vector <Frame> frames_;

        cv::Ptr <cv::GFTTDetector> gftt_;

        FrontEnd(/* args */);

        ~FrontEnd();

        /*
         * Initilize system.
         */
        bool Initialize();

        bool AddFrame(Frame &frame);

        bool DetectFeatures(Frame &frame);

        int FindFeaturesInRight(Frame &frame);


    };


}

#endif