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

    sslam::Frame Dataset::GetNextFrame() {
        boost::format data_fmt("%s/image_%d/%06d.png");
        cv::Mat left, right;

        left = cv::imread((data_fmt % dataset_dir % 0 % cur_img_index).str(), cv::IMREAD_GRAYSCALE);
        right = cv::imread((data_fmt % dataset_dir % 1 % cur_img_index).str(), cv::IMREAD_GRAYSCALE);


        cv::Mat resized_left, resized_right;

        cv::resize(left, resized_left, cv::Size(), 0.5, 0.5, cv::INTER_NEAREST);
        cv::resize(right, resized_right, cv::Size(), 0.5, 0.5, cv::INTER_NEAREST);

        Frame new_frame;
        new_frame.left_img_ = resized_left;
        new_frame.right_img_ = resized_right;
        cur_img_index++;
        return new_frame;
    }

    bool Dataset::GetCameraPara(std::vector<std::shared_ptr<Eigen::Matrix<double, 3, 3>>> &Ks, std::vector<std::shared_ptr< Vec3>> &ts) {
        std::string  data_dir = Config::Get<std::string>(Config::dataset_dir);
        std::ifstream fin(data_dir + "/calib.txt");
        if (!fin){
            return false;
        }

        for (int i = 0; i < 4; ++i) {
            char camera_name[3];
            for (int k = 0; k < 3; ++k) {
                fin >> camera_name[k];
            }
            double projection_data[12];
            for (int k = 0; k < 12; ++k) {
                fin >> projection_data[k];
            }

            std::shared_ptr<Eigen::Matrix<double, 3, 3>> K(new Eigen::Matrix<double, 3, 3>());
            *K << projection_data[0], projection_data[1], projection_data[2],
                    projection_data[4], projection_data[5], projection_data[6],
                    projection_data[8], projection_data[9], projection_data[10];
            std::shared_ptr<Vec3> t(new Vec3());
            *t << projection_data[3], projection_data[7], projection_data[11];
            *t = K->inverse() * *t;
            *K = *K * 0.5;
            Ks.push_back(K);
            ts.push_back(t);
        }
        fin.close();
        return true;
    }
}
