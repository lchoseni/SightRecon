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

//    graph.Rotate();
//    graph.Rotate();


  cv::Mat a = (cv::Mat_<double>(3, 3) << 1, 2, 3, 1, 0, 1, 0, 0, 1);
  cv::Mat b = (cv::Mat_<double>(3, 1) << 0, 0, 1);

  cout << a * b;

  for (int i = 0; i < 6; ++i) {
    graph.ComputeAllRAndT();
    graph.Propagate();
     graph.Rotate();
  }
//   cv::Mat r1 = (cv::Mat_<double>(3,  3) << 0.962742, -0.0160548, -0.269944 ,
// -0.270399, -0.0444283, -0.961723 ,
// 0.00344709, 0.998884, -0.0471142 );

//   cv::Mat t1 = (cv::Mat_<double>(3,  1) <<-14.1604, -3.32084, 0.0862032 );

//   cv::Mat r2 = (cv::Mat_<double>(3,  3) << 0.795163, -0.050195, -0.604314 ,
// -0.606377, -0.0736593, -0.791759 ,
// -0.00477103, 0.996019 ,-0.0890082 );

//   cv::Mat t2 = (cv::Mat_<double>(3,  1) <<-10.8142, -4.53704, 0.122293  );


//   cout << "R " << r2 * r1.inv() << endl;

//   cout << "T " << endl << - r2 * r1.inv() * t1 + t2 <<endl;

}