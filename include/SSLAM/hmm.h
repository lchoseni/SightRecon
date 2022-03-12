#include "common_include.h"


namespace sslam {
    class Hmm {

    public:
        Eigen::Matrix<double, 2, 2> transition_prob_;
//    Eigen::Matrix<double> emission_prob_;

        Hmm(Eigen::Matrix<double, 2, 2> transition_prob);

        void ComputeForwardMessage(int q);

        void ComputeBackwardMessage();

    };
}