//#ifndef MAP_POINT_H
//#define MAP_POINT_H
//
#include "common_include.h"
#include "feature.h"


namespace sslam {
    class MapPoint {

    public:

        std::map<int, std::weak_ptr<sslam::Feature>> features_;

        MapPoint(/* args */);

        ~MapPoint();
    };
}




//#endif
