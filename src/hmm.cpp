#include "SSLAM/hmm.h"


namespace sslam {
    Hmm::Hmm(Eigen::Matrix<double, 2, 2> transition_prob) : transition_prob_(transition_prob) {

    }

    void Hmm::ComputeForwardMessage(int q) {
        double alpha_0 = transition_prob_(0, 0);

        for (size_t i = 0; i < q; i++)
        {
            
        }
        
    }


    void Hmm::ComputeBackwardMessage() {

    }

}