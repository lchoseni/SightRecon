//
// Created by yeren on 1/24/22.
//

#include <boost/format.hpp>

#include "SSLAM/dataset.h"
#include "SSLAM/config.h"


namespace sslam {

    Dataset::Dataset() : cur_img_index(0), dataset_dir(Config::Get<std::string>(Config::dataset_dir)) {

    }

    Dataset::~Dataset() {

    }

    Frame Dataset::GetNextFrame() {
        boost::format fmt("%s/image_%d/%06d.png");
        cv::Mat left, right;

        left = cv::imread((fmt % dataset_dir % 0 % cur_img_index).str(), cv::IMREAD_GRAYSCALE);
        right = cv::imread((fmt % dataset_dir % 1 % cur_img_index).str(), cv::IMREAD_GRAYSCALE);


        cv::Mat resized_left, resized_right;

        cv::resize(left, resized_left, cv::Size(), 0.5, 0.5, cv::INTER_NEAREST);
        cv::resize(right, resized_right, cv::Size(), 0.5, 0.5, cv::INTER_NEAREST);

        Frame new_frame;
        new_frame.left_img_ = resized_left;
        new_frame.right_img_ = resized_right;
        cur_img_index++;
        return new_frame;
    }
}