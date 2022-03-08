#ifndef GRAPH_H
#define GRAPH_H

#include "frame.h"
#include "common_include.h"
#include "front_end.h"

namespace sslam{

    typedef struct {
        Mat33 R;
        Vec3 T;
    }RELA_RT;

    class Graph
    {
    private:
        map<unsigned int, map<unsigned int, RELA_RT>> id_to_RTs;
        map<unsigned int, map<unsigned int, RELA_RT>> id_to_feature_matches;
        string ref_img;
    public:
//        Graph(/* args */);
        shared_ptr<FrontEnd> front_end_;
        void AddFrame(Frame &frame);
        void ComputeRAndTOfTwoImgs(Frame &frame1, Frame &frame2);
        void ComputeAllRAndT();
//        void ComputeNCC(Frame &frame1, Frame &frame2);

//        ~Graph();
    };
    

    

}

#endif