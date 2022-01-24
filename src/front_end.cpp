#include <opencv2/opencv.hpp>

#include "SSLAM/front_end.h"
#include "SSLAM/config.h"


namespace sslam {
    FrontEnd::FrontEnd() {
        gftt_ = cv::GFTTDetector::create(Config::Get<int>("num_features"), 0.01, 1);
    }

    bool FrontEnd::DetectFeatures(Frame &frame) {

        std::vector <cv::KeyPoint> key_points;
        switch (sslam::Config::Get<int>(sslam::Config::source_type)) {
            case 1:
                gftt_->detect(frame.left_img_, key_points);
                if (key_points.size() == 0) {
                    return false;
                }
                frame.SetLeftKP(key_points);
                for (size_t i = 0; i < key_points.size(); i++) {
                    frame.left_features_.push_back(new Feature(std::weak_ptr<Frame>(new Frame()), key_points[i]));
                    return true;
                }
        }
    }


    int FrontEnd::FindFeaturesInRight(Frame &frame) {

        if (Config::Get<int>(Config::source_type) != Config::stereo) {
            return false;
        }

        std::vector <cv::KeyPoint> key_points;
        key_points.resize(frame.left_key_points_.size());

        std::vector <uchar> status;
        std::vector<float> err;
        cv::TermCriteria criteria = cv::TermCriteria((cv::TermCriteria::COUNT) + (cv::TermCriteria::EPS), 10, 0.03);
        cv::calcOpticalFlowPyrLK(frame.left_img_, frame.right_img_, frame.left_key_points_, key_points, status, err,
                                 cv::Size(11, 11), 3, criteria, cv::OPTFLOW_USE_INITIAL_FLOW);

        int num_good_points = 0;
        for (int i = 0; i < status.size(); ++i) {
            if (status[i]) {
                cv::KeyPoint kp(key_points[i], 7);
                Feature feat(frame, kp);
                feat.is_on_left_image_ = false;
                frame.right_features_.push_back(feat);
                num_good_points++;
            } else {
                frame.right_features_.push_back(nullptr);
            }
        }

        return num_good_points;

    }

    bool FrontEnd::ReadNextImg() {

    }

    bool FrontEnd::AddFrame(Frame &frame) {
        frames_ = frame;
        return true;
    }

    bool FrontEnd::Initialize() {

    }


}