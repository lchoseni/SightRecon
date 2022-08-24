#ifndef SSLAM_FEATURE_EXTRACTOR_H
#define SSLAM_FEATURE_EXTRACTOR_H

#include "../common_include.h"

namespace SSLAM{
    class FeatureExtractor{
        public:
        virtual int extract(cv::Mat &frame, vector<cv::KeyPoint> &keyPoints) = 0;
    };
}

#endif