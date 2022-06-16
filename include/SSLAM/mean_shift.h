#ifndef SSLAM_INCLUDE_SSLAM_MEAN_SHIFT_H_
#define SSLAM_INCLUDE_SSLAM_MEAN_SHIFT_H_

#include "common_include.h"

namespace sslam {

class MeanShift {

 public:
  double bandwidth_, threshold_;
  vector<vector<double>> *points_;
  vector<vector<double>> final_points_;
  vector<int> labels_;

  MeanShift(double bandwidth, double threshold, vector<vector<double>> *pts);

  int RandomChoose();
  double Distance(vector<double> &p1, vector<double> &p2);
  double GaussianKernel(double distance, double bandwidth);
  int Shift();
  int Label();
  Vec3 GetMean();

};

}
#endif //SSLAM_INCLUDE_SSLAM_MEAN_SHIFT_H_
