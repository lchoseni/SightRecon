#ifndef SRECON_FEATURE_EXTRACTOR_H
#define SRECON_FEATURE_EXTRACTOR_H

#include "../common_include.h"

namespace srecon{
    class FeatureExtractor{
        public:
        virtual int extract(cv::Mat &frame, vector<cv::KeyPoint> &keyPoints) = 0;
    };
}

#endif