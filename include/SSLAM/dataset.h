//
// Created by yeren on 1/24/22.
//

#ifndef SSLAM_DATASET_H
#define SSLAM_DATASET_H

#include "common_include.h"
#include "frame.h"

namespace sslam {
    class Dataset {
    private:
        int cur_img_index;
        const std::string dataset_dir;

    public:
        Dataset();

        ~Dataset();

        sslam::Frame GetNextFrame();

        static bool GetCameraPara(std::vector<std::shared_ptr<Eigen::Matrix<double, 3, 3>>> &Ks,
                                  std::vector<std::shared_ptr< Vec3>> &ts);

    };
}

#endif //SSLAM_DATASET_H
