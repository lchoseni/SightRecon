#include "SSLAM/hmm.h"

namespace sslam {


Hmm::Hmm(Eigen::Matrix<double, 2, 2> transition_prob) : transition_prob_(transition_prob) {

}

double Hmm::ComputeForwardMessage(int row, int col, double z1_emission, double z0_emission, double prev0,
                                double tran_0_1) {


    double zn0, zn1;
    zn0 = (tran_0_1 * prev0 + (1 - tran_0_1) * (1 - prev0)) * z0_emission;
    zn1 = (( 1 - tran_0_1) * prev0 + tran_0_1 * (1 - prev0)) * z1_emission;
    return zn1 / (zn0 + zn1);


}

double Hmm::ComputeBackwardMessage(int row, int col,  double z1_emission, double z0_emission, double later0,
                                   double tran_0_1) {

    double zn0, zn1;
    zn0 = tran_0_1 * z1_emission * later0 + (1 - tran_0_1) * z0_emission * (1 - later0);
    zn1 = (1 - tran_0_1) * z1_emission * later0 + tran_0_1 * z0_emission * (1 - later0);
    return zn1 / (zn0 + zn1);

}

}