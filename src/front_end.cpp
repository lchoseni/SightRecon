#include <opencv2/opencv.hpp>

#include "SSLAM/front_end.h"
#include "SSLAM/config.h"


namespace sslam {
    FrontEnd::FrontEnd() {
        gftt_ = cv::GFTTDetector::create(Config::Get<int>("num_features"), 0.01, 1);
    }

    FrontEnd::~FrontEnd() {

    }

    bool FrontEnd::DetectFeatures(Frame &frame) {

        cv::imshow("img", frame.left_img_);
        cv::waitKey(1);
        std::vector<cv::KeyPoint> key_points;
        switch (sslam::Config::Get<int>(sslam::Config::source_type)) {
            case 1:
                gftt_->detect(frame.left_img_, key_points);
                if (key_points.size() == 0) {
                    return false;
                }
                frame.SetLeftKP(key_points);
                for (size_t i = 0; i < key_points.size(); i++) {
                    frame.left_features_.push_back(Feature(frame, key_points[i]));
                }
        }
        return true;
    }


    int FrontEnd::FindFeaturesInRight(Frame &frame) {

        if (Config::Get<int>(Config::source_type) != Config::stereo) {
            return false;
        }

        std::vector<cv::Point2f> left_kps, right_kps;
//        std::cout << "feature size is " << frame.left_features_.size()<< std::endl;
        for (auto &kp: frame.left_features_) {
            left_kps.push_back(kp.key_point_.pt);
            right_kps.push_back(kp.key_point_.pt);
        }


//        right_kps.resize(frame.left_key_points_.size());

        std::vector<uchar> status;
        std::vector<float> err;
        cv::TermCriteria criteria = cv::TermCriteria((cv::TermCriteria::COUNT) + (cv::TermCriteria::EPS), 10, 0.03);
        cv::calcOpticalFlowPyrLK(frame.left_img_, frame.right_img_, left_kps, right_kps, status, err,
                                 cv::Size(11, 11), 3, criteria, cv::OPTFLOW_USE_INITIAL_FLOW);

        int num_good_points = 0;
        for (size_t i = 0; i < status.size(); ++i) {
            if (status[i]) {
                cv::KeyPoint kp(right_kps[i], 7);
                Feature feat(frame, kp);
                feat.is_on_left_image_ = false;
                frame.right_features_.push_back(feat);
                frame.right_key_points_.push_back(kp);
                num_good_points++;
            } else {
                frame.right_features_.push_back(Feature());
            }
        }

        return num_good_points;

    }


    bool FrontEnd::AddFrame(Frame &frame) {
        return true;
    }

    bool FrontEnd::Initialize() {
        return true;
    }

    bool FrontEnd::Triangulation(Sophus::SE3d &T, Vec3 pos_1, Vec3 pos_2) {
        // Calculate the pose from the first img to the second
        // img as the initial pose.
        // Use SVD to get depth.

    }

    bool FrontEnd::InitMap() {


    }


}