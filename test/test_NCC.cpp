//
// Created by lchoseni on 2022/3/17.
//

#include "SSLAM/graph.h"
#include "SSLAM/dataset.h"
#include "SSLAM/front_end.h"
#include "SSLAM/config.h"
#include "SSLAM/frame.h"
#include "SSLAM/hmm.h"
#include "SSLAM/fusion.h"
#include "SSLAM/mean_shift.h"

int main() {
  sslam::Config::SetConfigFile(sslam::Config::config_file_);
  sslam::FrontEnd front_end;

  for (int ref_idx = 0; ref_idx < 9; ++ref_idx) {
    sslam::Dataset dataset;
    sslam::Graph graph = sslam::Graph(&dataset, dataset.GetNextFrame(ref_idx));

    graph.InitialRandomDepth();
    cv::Mat K;
    cv::eigen2cv(graph.ref_img_->cam_->K(), K);
    graph.InitialRandomNormal(K);

    graph.ComputeAllRAndT();

//    // Load reference depth data.
//    graph.ref_img_->depth = cv::Mat(graph.ref_img_->left_img_.rows, graph.ref_img_->left_img_.cols, CV_64F);
//    stringstream ss;
//    ss << "depth-d10-0-r11-i11-pc-hmm-" << graph.ref_img_->id_ << ".txt";
//    cout << "Load depth of " << ss.str() << endl;
//
//    std::ifstream fin(ss.str());
//    double temp_val;
//    for (int row = 0; row < graph.ref_img_->left_img_.rows; ++row) {
//      for (int col = 0; col < graph.ref_img_->left_img_.cols; ++col) {
//        fin >> temp_val;
//
//        graph.ref_img_->depth.at<double>(row, col) = temp_val;
//      }
//    }
//    fin.close();
//    graph.ref_img_->depth.copyTo(graph.depth);
    for (int i = 0; i < 12; ++i) {
        graph.Propagate();

      graph.Rotate();
    }
  }



}