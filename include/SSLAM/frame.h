#ifndef FRAME_H
#define FRAME_H

#include<opencv2/opencv.hpp>

#include "common_include.h"
#include "feature.h"


namespace sslam {

    class Feature;

    class Frame {
    private:
        // index for all frames, it indicates next frame id.
        static unsigned int global_index;
        // id of current frame
        unsigned int id_;
        Sophus::SE3d pose_;
        std::vector<Feature> features_;

    public:
        cv::Mat img_, left_img_, right_img_;
        std::vector<cv::KeyPoint> left_key_points_;
        std::vector<cv::KeyPoint> right_key_points_;
        std::vector<Feature> left_features_;
        std::vector<Feature> right_features_;

        Frame();

        ~Frame();

        static unsigned int GetNextIndex();

        bool SetLeftKP(std::vector<cv::KeyPoint> &kps);

        void DrawKeyPoints();
    };

}

#endif