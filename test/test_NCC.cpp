//
// Created by lchoseni on 2022/3/17.
//

#include "SSLAM/graph.h"
#include "SSLAM/dataset.h"
#include "SSLAM/front_end.h"
#include "SSLAM/config.h"
#include "SSLAM/frame.h"
#include "SSLAM/hmm.h"

int main() {
  sslam::Config::SetConfigFile(sslam::Config::config_file_);
  sslam::FrontEnd front_end;

  sslam::Dataset dataset;
  sslam::Graph graph = sslam::Graph(&dataset, dataset.GetNextFrame());

  int win_size = 5;
  int src_id = 0;
//  hmm.ComputeBackwardMessage()
  graph.InitialRandomDepth();
  cv::Mat K;
  cv::eigen2cv(graph.ref_img_->cam_->K(), K);
  graph.InitialRandomNormal(K);

    graph.ComputeAllRAndT();
  for (int i = 0; i < 12; ++i) {
    graph.Propagate();
     graph.Rotate();
  }

}