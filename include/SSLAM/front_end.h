#ifndef FRONT_END_H
#define FRONT_END_H

#include <opencv2/features2d.hpp>

#include "common_include.h"
#include "frame.h"

namespace sslam{
    class FrontEnd
    {
    private:
        std::vector<Frame> frames_;
    public:

        cv::Ptr<cv::GFTTDetector> gftt_;

        FrontEnd(/* args */);
        ~FrontEnd();

        /*
         * Initilize system.
         */
        bool Initilize();

        bool AddFrame(Frame &frame);

        bool DetectFeatures(Frame &frame);

        bool FindFeaturesInRight(Frame &frame);


    };
    
    
}

#endif