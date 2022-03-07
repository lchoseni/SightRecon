#ifndef GRAPH_H
#define GRAPH_H

#include "frame.h"

namespace sslam{


    class Graph
    {
    private:
        /* data */
    public:
//        Graph(/* args */);

        void AddFrame(Frame &frame);
        void ComputeRAndTOfTwoImgs(Frame &frame1, Frame &frame2);
        void ComputeAllRAndT();

//        ~Graph();
    };
    

    

}

#endif