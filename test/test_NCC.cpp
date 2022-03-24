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

    graph.Rotate();
    graph.Rotate();

  for (int i = 0; i < 20; ++i) {
    graph.ComputeAllRAndT();
    graph.Propagate();
    graph.Rotate();
  }

}