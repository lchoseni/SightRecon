//
// Created by yeren on 1/24/22.
//

#ifndef SSLAM_DATASET_H
#define SSLAM_DATASET_H

#include "common_include.h"
#include "frame.h"
#include "camera.h"

namespace sslam {
    class Dataset {
    private:
        int cur_img_index;

        const std::string dataset_dir;
        static std::shared_ptr<Camera> left_camera_, right_camera_;

    public:
        Dataset();

        ~Dataset();

        shared_ptr<Frame> GetNextFrame();

        static bool GetCameraPara(std::vector<std::shared_ptr<Eigen::Matrix<double, 3, 3>>> &Ks,
                                  std::vector<std::shared_ptr< Vec3>> &ts);

        static std::shared_ptr<Camera> GetCamera(int id);

    };
}

#endif //SSLAM_DATASET_H
