#include <opencv2/core.hpp>

#include "SSLAM/graph.h"
#include "SSLAM/common_include.h"
#include "SSLAM/camera.h"
#include "SSLAM/frame.h"

namespace sslam {



    void Graph::ComputeRAndTOfTwoImgs(sslam::Frame &frame1, sslam::Frame &frame2){

        front_end_->DetectFeatures(frame1);
        front_end_->DetectFeatures(frame2);

        cv::Mat descriptors_1, descriptors_2;
        cv::Ptr<cv::FeatureDetector> detector = cv::ORB::create();
        cv::Ptr<cv::DescriptorExtractor> descriptor = cv::ORB::create();
        cv::Ptr<cv::DescriptorMatcher> matcher = cv::DescriptorMatcher::create("BruteForce-Hamming");

        detector->detect(frame1.left_img_, frame1.left_key_points_);
        detector->detect(frame2.left_img_, frame2.left_key_points_);

        descriptor->compute(frame1.left_img_, frame1.left_key_points_, descriptors_1);
        descriptor->compute(frame2.left_img_, frame2.left_key_points_, descriptors_2);


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
            ky_pts1.push_back(frame1.left_key_points_[match.queryIdx]);
            ky_pts2.push_back(frame2.left_key_points_[match.trainIdx]);
        }

        cv::KeyPoint::convert(ky_pts1, pts1);
        cv::KeyPoint::convert(ky_pts2, pts2);
        std::cout << pts1.size() << " " << pts2.size() << std::endl;
        const cv::Mat h = cv::findHomography(pts1, pts2);
        std::cout << "Homography is " << h << std::endl;
        // cv::Mat outimg;
        // cv::warpPerspective(frame1.left_img_, outimg, h, outimg.size());
        // cv::imshow("after h", outimg);

        std::vector<cv::Mat> Rs, Ts, Ns;
        cv::Mat K1, K2;
        cout << (frame1.GetCamera() == nullptr) << endl;
        frame1.GetCamera()->K();
        cv::eigen2cv(frame1.GetCamera()->K(), K1);
        cv::eigen2cv(frame2.GetCamera()->K(), K2);

        int solutions = cv::decomposeHomographyMat(h, K1, Rs, Ts, Ns);
        cv::filterHomographyDecompByVisibleRefpoints

        std::cout << "solutions is " << solutions << std::endl;
        cout << "R is " << Rs[0] << ", and T is " << Ts[0] << ", and N is " << Ns[0] << endl;
        // cv::waitKey(0);
    }

    void Graph::ComputeAllRAndT() {

    }
}