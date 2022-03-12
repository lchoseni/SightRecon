#ifndef GRAPH_H
#define GRAPH_H

#include "frame.h"
#include "common_include.h"
#include "front_end.h"
#include "dataset.h"

namespace sslam{

    typedef struct {
        cv::Mat R;
        cv::Mat T;
    }RELA_RT;

    class Graph
    {
    private:
        map<unsigned int, map<unsigned int, RELA_RT>> id_to_RTs_;
        Dataset *dataset_;
        shared_ptr<Frame> ref_img_;
    public:
        Graph(Dataset *dataset);
        shared_ptr<FrontEnd> front_end_;
        void AddFrame(Frame &frame);
        bool ComputeRAndTOfTwoImgs(shared_ptr<Frame> frame1, shared_ptr<Frame> frame2, cv::Mat &R_, cv::Mat &t_);
        void ComputeAllRAndT();
        void ComputeNCC(Frame &frame1, Frame &frame2, int row_pix, int col_pix);


//        ~Graph();
    };
    

    

}

#endif