#ifndef FRAME_H
#define FRAME_H

#include<opencv2/opencv.hpp>

#include "common_include.h"
#include "feature.h"
#include "camera.h"


namespace sslam {

    class Feature;

    class Frame {

    private:
        // index for all frames, it indicates next frame id.
        static unsigned int global_index;
        // id of current frame
        unsigned int id_;
        std::shared_ptr<sslam::Camera> cam;
        Sophus::SE3d pose_;
        std::vector<Feature> features_;

    public:
        cv::Mat img_, left_img_, right_img_;
        std::vector<cv::KeyPoint> left_key_points_;
        std::vector<cv::KeyPoint> right_key_points_;
        std::vector<std::shared_ptr<Feature>> left_features_;
        std::vector<std::shared_ptr<Feature>> right_features_;
        // Key point matches between left img and right img.
        std::vector<std::vector<std::shared_ptr<Feature>>> matches;

        Frame();

        ~Frame();

        void SetCamera(std::shared_ptr<Camera> &sharedPtr);

        std::shared_ptr<sslam::Camera> GetCamera();

        static unsigned int GetNextIndex();

        bool SetLeftKP(std::vector<cv::KeyPoint> &kps);

        void DrawKeyPoints();
    };

}

#endif
