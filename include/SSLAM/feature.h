#ifndef FEATURE_H
#define FEATURE_H

#include "common_include.h"
#include "map_point.h"

namespace sslam
{
    class Feature
    {
    private:
        unsigned int u_, v_;
        std::weak_ptr<Frame> frame_;
        
        
    public:
        Feature(Frame &frame);
        ~Feature();
    };

}

#endif