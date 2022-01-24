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

        Frame GetNextFrame();

    };
}

#endif //SSLAM_DATASET_H
