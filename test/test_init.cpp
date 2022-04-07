//
// Created by yeren on 1/24/22.
//

#include "SSLAM/common_include.h"
#include "SSLAM/front_end.h"
#include "SSLAM/config.h"
#include "SSLAM/dataset.h"
#include "SSLAM/frame.h"


cv::Scalar get_color(double depth) {
    depth = depth * 300;
  double up_th = 50, low_th = 10, th_range = up_th - low_th;
  if (depth > up_th) depth = up_th;
  if (depth < low_th) depth = low_th;
  return cv::Scalar(255 * depth / th_range, 0, 255 * (1 - depth / th_range));
}



int main(){
    sslam::Config::SetConfigFile(sslam::Config::config_file_);
    sslam::FrontEnd front_end;

    sslam::Dataset dataset;
    shared_ptr<sslam::Frame> frame = dataset.GetNextFrame();
    front_end.DetectFeatures(frame);
    std::cout << "Key points of left img is " << frame->left_key_points_.size() << std::endl;;
    std::cout << "Key points of right img is " << front_end.FindFeaturesInRight(frame)<< std::endl;;

    std::vector<cv::Point3d> pts;
    front_end.Triangulation(front_end.right_camera_->pose_, frame->left_features_, frame->right_features_, pts);

    cv::Mat img1 = frame->left_img_.clone();
    cv::Mat img2 = frame->right_img_.clone();
    for (size_t i = 0; i < frame->matches.size(); ++i){
        if (frame->right_features_.at(i) == nullptr) continue;
        std::vector<std::shared_ptr<sslam::Feature>> match = frame->matches.at(i);
        double depth1 = pts[i].z;
        cv::circle(img1, match[0].get()->key_point_.pt, 2, get_color(depth1), 2);

        cv::Mat p2_trans = (cv::Mat_<double>(3, 3) << 1, 0, 0, 0, 1, 0, 0, 0, 1)
                * (cv::Mat_<double>(3, 1) << pts[i].x , pts[i].y , pts[i].z)
                + (cv::Mat_<double>(3, 1)
                   << front_end.right_camera_.get()->pose_.translation()(0, 0)
                   , front_end.right_camera_.get()->pose_.translation()(1, 0)
                   , front_end.right_camera_.get()->pose_.translation()(2, 0));

        double depth2 = p2_trans.at<double>(2, 0);
        cv::circle(img2, match[1].get()->key_point_.pt, 2, get_color(depth2), 2);

    }
    cv::imshow("img1", img1);
    cv::imshow("img2", img2);
    cv::waitKey();
    frame->DrawKeyPoints();
}
