#include <opencv2/core.hpp>

#include "SSLAM/graph.h"
#include "SSLAM/common_include.h"
#include "SSLAM/camera.h"
#include "SSLAM/frame.h"

namespace sslam {



    void Graph::ComputeRAndTOfTwoImgs(sslam::Frame &frame1, sslam::Frame &frame2){

        std::vector<cv::Point2f> pts1, pts2;
        std::vector<cv::KeyPoint> ky_pts1, ky_pts2;

        for (auto & match : frame1.matches) {
            ky_pts1.push_back(match[0]->key_point_);
            ky_pts2.push_back(match[1]->key_point_);
        }


        cv::KeyPoint::convert(ky_pts2, pts1);
        cv::KeyPoint::convert(ky_pts2, pts2);
        std::cout << pts1.size() << " " << pts2.size() << std::endl;
        const cv::Mat h = cv::findHomography(pts1, pts2);
        std::cout << "Homography is " << h << std::endl;

        std::vector<cv::Mat> Rs, Ts, Ns;
        cv::Mat K1, K2;
        cout << (frame1.GetCamera() == nullptr) << endl;
        frame1.GetCamera()->K();
        cv::eigen2cv(frame1.GetCamera()->K(), K1);
        cv::eigen2cv(frame2.GetCamera()->K(), K2);


        cout << K1 << K2 <<endl;
        int solutions = cv::decomposeHomographyMat(h, K1, Rs, Ts, Ns);

        std::cout << "solutions is " << solutions << std::endl;
        cout << "R is " << Rs[0] << ", and T is " << Ts[0] << ", and N is " << Ns[0] << endl;

        cv::imshow("a", frame1.left_img_);
        cv::imshow("v", frame1.right_img_);
//        cv::imshow("d", h * frame1.left_img_ );

        cv::waitKey(0);


    }
}