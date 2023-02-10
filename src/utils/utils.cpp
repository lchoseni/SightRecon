//
// Created by lchoseni on 2022/3/17.
//
#include "srecon/utils/utils.h"

#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>

#include "srecon/frame.h"
#include "srecon/map.h"

namespace srecon {
void DrawShowPoint(int row, int col, cv::Mat img, int wait,
                   std::string win_name) {
  cv::Point center(col, row);
  cv::circle(img, center, 20, cv::Scalar(0, 0, 255), 3);
  cv::imshow(win_name, img);
  cv::waitKey(wait);
}

Eigen::Matrix3d skewSymmetric(Eigen::Vector3d &q) {
  Eigen::Matrix3d result;
  result << 0, -q(2), q(1), q(2), 0, -q(0), -q(1), q(0), 0;
  return result;
}

cv::Point2f pixel2cam(const cv::Point2d &p, const cv::Mat &K) {
  return cv::Point2f((p.x - K.at<double>(0, 2)) / K.at<double>(0, 0),
                     (p.y - K.at<double>(1, 2)) / K.at<double>(1, 1));
}

void triangulatePts(cv::Mat R1, cv::Mat T1, cv::Mat R2, cv::Mat T2, cv::Mat K,
                    vector<cv::Point2f> &kps1_2d, vector<cv::Point2f> &kps2_2d,
                    cv::Mat &pts_3d) {
  if (!kps1_2d.empty() && !kps2_2d.empty()) {
    // Triangulate points
    cv::Mat RT1 =
        (cv::Mat_<double>(3, 4) << R1.at<double>(0, 0), R1.at<double>(0, 1),
         R1.at<double>(0, 2), T1.at<double>(0, 0), R1.at<double>(1, 0),
         R1.at<double>(1, 1), R1.at<double>(1, 2), T1.at<double>(1, 0),
         R1.at<double>(2, 0), R1.at<double>(2, 1), R1.at<double>(2, 2),
         T1.at<double>(2, 0));
    cv::Mat RT2 =
        (cv::Mat_<double>(3, 4) << R2.at<double>(0, 0), R2.at<double>(0, 1),
         R2.at<double>(0, 2), T2.at<double>(0, 0), R2.at<double>(1, 0),
         R2.at<double>(1, 1), R2.at<double>(1, 2), T2.at<double>(1, 0),
         R2.at<double>(2, 0), R2.at<double>(2, 1), R2.at<double>(2, 2),
         T2.at<double>(2, 0));

    // vector<cv::Point2f> kps1, kps2;
    // for (size_t i = 0; i < kps1_2d.size(); i++) {
    //   kps1.push_back(pixel2cam(kps1_2d[i], K));
    //   kps2.push_back(pixel2cam(kps2_2d[i], K));
    // }
    cv::triangulatePoints(K * RT1, K * RT2, kps1_2d, kps2_2d, pts_3d);
  }
}
}  // namespace srecon