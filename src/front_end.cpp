#include <opencv2/opencv.hpp>

#include "SSLAM/front_end.h"
#include "SSLAM/config.h"
#include "SSLAM/dataset.h"


namespace sslam {
    FrontEnd::FrontEnd() {
        gftt_ = cv::GFTTDetector::create(Config::Get<int>("num_features"), 0.01, 1);
        std::vector<std::shared_ptr<Eigen::Matrix<double, 3, 3>>> Ks;
        std::vector<std::shared_ptr<Vec3>> ts;
        sslam::Dataset::GetCameraPara(Ks, ts);

        for (size_t i = 0; i < Ks.size(); ++i){
            std::cout << *Ks[i].get() << *ts[i].get() << std::endl;
        }
            left_camera_ = std::shared_ptr<sslam::Camera>(
                        new Camera((*Ks[0])(0, 0), (*Ks[0])(1, 1), (*Ks[0])(0, 2), (*Ks[0])(1, 2), Sophus::SE3d(Sophus::SO3d(), *ts[0])));
            right_camera_ = std::shared_ptr<sslam::Camera>(
                        new Camera((*Ks[1])(0, 0), (*Ks[1])(1, 1), (*Ks[1])(0, 2), (*Ks[1])(1, 2), Sophus::SE3d(Sophus::SO3d(), *ts[1])));

    }

    FrontEnd::~FrontEnd() {

    }

    bool FrontEnd::DetectFeatures(Frame &frame) {

        std::vector<cv::KeyPoint> key_points;
        switch (sslam::Config::Get<int>(sslam::Config::source_type)) {
            case 1:
                gftt_->detect(frame.left_img_, key_points);
                if (key_points.size() == 0) {
                    return false;
                }
                frame.SetLeftKP(key_points);
                for (size_t i = 0; i < key_points.size(); i++) {
                    frame.left_features_.push_back(Feature::Ptr(new Feature(frame, key_points[i])));
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
            left_kps.push_back(kp->key_point_.pt);
            right_kps.push_back(kp->key_point_.pt);
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
                Feature::Ptr feat(new Feature(frame, kp));
                feat->is_on_left_image_ = false;
                frame.right_features_.push_back(feat);
                frame.right_key_points_.push_back(kp);
                frame.matches.push_back(std::vector<std::shared_ptr<Feature>>{frame.left_features_[i], feat});
                num_good_points++;
            } else {
                frame.right_features_.push_back(nullptr);
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


    // Calculate the pose from the first img to the second
    // img as the initial pose.
    bool FrontEnd::Triangulation(Sophus::SE3d &T, std::vector<Feature::Ptr> &pt1, std::vector<Feature::Ptr> &pt2,
                                 std::vector<cv::Point3d> &points) {

        cv::Mat T1 = (cv::Mat_<float>(3, 4) <<
                1, 0, 0, 0,
                0, 1, 0, 0,
                0, 0, 1, 0);

        cv::Mat T2 = (cv::Mat_<float>(3, 4) <<
                      1,0,0, T.translation()(0, 0),
                      0,1,0, T.translation()(1, 0),
                      0,0,1, T.translation()(2, 0));

        std::cout<< T.translation() << T2 <<std::endl;
        std::vector<cv::Point2f> pts_1, pts_2;
        for (size_t i = 0; i < pt1.size(); ++i){
            if (pt2[i] == nullptr) continue;
            pts_1.push_back(cv::Point2f(pt1[i].get()->key_point_.pt.x, pt1[i].get()->key_point_.pt.y));
            pts_2.push_back(cv::Point2f(pt2[i].get()->key_point_.pt.x, pt1[i].get()->key_point_.pt.y));
        }
        cv::Mat pts_4d;
        cv::triangulatePoints(T1, T2, pts_1, pts_2, pts_4d);

        for (int i = 0; i < pts_4d.cols; i++) {
            cv::Mat x = pts_4d.col(i);
            x /= x.at<float>(3, 0); // 归一化
            cv::Point3d p(
              x.at<float>(0, 0),
              x.at<float>(1, 0),
              x.at<float>(2, 0)
            );
            points.push_back(p);
          }

        return true;
    }

    bool FrontEnd::InitMap() {

        return true;
    }


}
