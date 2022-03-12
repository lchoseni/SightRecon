#include <opencv2/core.hpp>

#include "SSLAM/graph.h"
#include "SSLAM/common_include.h"
#include "SSLAM/camera.h"
#include "SSLAM/frame.h"
#include "SSLAM/dataset.h"

namespace sslam {


    Graph::Graph(Dataset *dataset) : dataset_(dataset)
    {}


    bool Graph::ComputeRAndTOfTwoImgs(shared_ptr<Frame> frame1, shared_ptr<Frame> frame2, cv::Mat &R_, cv::Mat &t_){

        if (frame1 == nullptr || frame2 == nullptr)
        {
            return false;
        }
        

        front_end_->DetectFeatures(frame1);
        front_end_->DetectFeatures(frame2);

        cv::Mat descriptors_1, descriptors_2;
        cv::Ptr<cv::FeatureDetector> detector = cv::ORB::create();
        cv::Ptr<cv::DescriptorExtractor> descriptor = cv::ORB::create();
        cv::Ptr<cv::DescriptorMatcher> matcher = cv::DescriptorMatcher::create("BruteForce-Hamming");

        detector->detect(frame1->left_img_, frame1->left_key_points_);
        detector->detect(frame2->left_img_, frame2->left_key_points_);

        descriptor->compute(frame1->left_img_, frame1->left_key_points_, descriptors_1);
        descriptor->compute(frame2->left_img_, frame2->left_key_points_, descriptors_2);


        vector<cv::DMatch> matches;
        matcher->match(descriptors_1, descriptors_2, matches);

        auto min_max = minmax_element(matches.begin(), matches.end(),
                                        [](const cv::DMatch &m1, const cv::DMatch &m2) { return m1.distance < m2.distance; });

        double min_dist = min_max.first->distance;
        double max_dist = min_max.second->distance;

        printf("-- Max dist : %f \n", max_dist);
        printf("-- Min dist : %f \n", min_dist);

        std::vector<cv::DMatch> good_matches;
        for (int i = 0; i < descriptors_1.rows; i++) {
            if (matches[i].distance <= max(2 * min_dist, 1.0)) {
            good_matches.push_back(matches[i]);
            }
        }

        // cv::Mat img_match;
        // cv::Mat img_goodmatch;
        // drawMatches(frame1.left_img_, frame1.left_key_points_, frame2.left_img_, frame2.left_key_points_, matches, img_match);
        // drawMatches(frame1.left_img_, frame1.left_key_points_, frame2.left_img_, frame2.left_key_points_, good_matches, img_goodmatch);
        // imshow("all matches", img_match);
        // imshow("good matches", img_goodmatch);

        std::vector<cv::Point2f> pts1, pts2;
        std::vector<cv::KeyPoint> ky_pts1, ky_pts2;
        for (auto & match : good_matches) {
            ky_pts1.push_back(frame1->left_key_points_[match.queryIdx]);
            ky_pts2.push_back(frame2->left_key_points_[match.trainIdx]);
        }

        cv::KeyPoint::convert(ky_pts1, pts1);
        cv::KeyPoint::convert(ky_pts2, pts2);
        // std::cout << pts1.size() << " " << pts2.size() << std::endl;

        if (pts1.size() < 8 || pts2.size() < 8)
        {
            return false;
        }
        

        cv::Mat K1, K2;

        cv::eigen2cv(frame1->GetCamera()->K(), K1);
        cv::eigen2cv(frame2->GetCamera()->K(), K2);

        cv::Mat ess = cv::findEssentialMat(pts1, pts2, K1);
        // cout << "Essential matrix is " << ess << endl;
        
        cv::Mat R,t;
        if(cv::recoverPose(ess, pts1, pts2, R, t) <= 0){
            return false;
        }
        cout << "R is " << R << ", and T is " << t << endl;
        R_ = R;
        t_ = t;
        return true;
    }

    void Graph::ComputeAllRAndT() {
        
        shared_ptr<Frame> frame = nullptr;
        while ((frame = dataset_->GetNextFrame()) != nullptr){
            cout << "haha" <<endl;
            if (ref_img_ == nullptr){
                ref_img_ = frame;
                continue;
            }
            cv::Mat R, t;
            if (ComputeRAndTOfTwoImgs(ref_img_, frame, R, t) == false) 
                continue;
        
            RELA_RT rela_rt;
            rela_rt.R = R;
            rela_rt.T = t;
            if (id_to_RTs_.count(ref_img_ ->id_) == 0){
                map<unsigned int, RELA_RT> map;
                map.insert(make_pair(frame->id_, rela_rt));
                id_to_RTs_.insert(make_pair(ref_img_->id_, map));
            } else{
                id_to_RTs_[ref_img_->id_].insert(make_pair(frame->id_, rela_rt));
            }
            cout << frame->id_ << endl;
        }
    }

    void ComputeNCC(Frame &frame1, Frame &frame2){
        


    }
}