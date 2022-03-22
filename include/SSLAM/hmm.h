#ifndef SSLAM_HMM
#define SSLAM_HMM


#include "common_include.h"

namespace sslam {
class Hmm {

 public:
  Eigen::Matrix<double, 2, 2> transition_prob_;
//    Eigen::Matrix<double> emission_prob_;

  Hmm(Eigen::Matrix<double, 2, 2> transition_prob);

  double ComputeForwardMessage(int row, int col, int width, double z1_emission, double z0_emission, double later0,
                               double tran_0_1);

  double ComputeBackwardMessage(int row, int col, int width, double z1_emission, double z0_emission, double later0,
                                double tran_0_1);
};

}

#endif