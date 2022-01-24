#include <opencv2/opencv.hpp>

#include "../include/SSLAM/front_end.h"
#include "../include/SSLAM/config.h"


namespace sslam {
    FrontEnd::FrontEnd() {
        gftt_ = cv::GFTTDetecor::create(Config::Get<std::string>("num_features"), 0.01, 1)
    }

    bool FrontEnd::DetectFeatures(Frame &frame) {

        std::vector <cv::KeyPoint> key_points;
        switch (sslam::Config::Get<int>(sslam::Config::source_type)) {
            case Config::stereo:
                gftt_->detect(frame.left_img_, key_points);
                if (key_points.size() == 0) {
                    return false;
                }
                for (size_t i = 0; i < key_points.size(); i++) {
                    frame.left_features_.push_back(new Feature(frame));

                    return true;
                }
        }
    }


    bool FrontEnd::FindFeaturesInRight(Frame &frame) {

        if (Config::source_type != Config::stereo){
            return false;
        }

        std::vector<cv::KeyPoint> key_points;
    }


}