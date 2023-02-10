#include "srecon/dataset/ktti_dataset.h"

#include <boost/format.hpp>
#include <iostream>

#include "srecon/config.h"
#include "unistd.h"

namespace srecon {

KttiDataset::KttiDataset() : ImageDataset() {
    dataset_dir = Config::Get<std::string>(Config::img_dataset_dir);
    readGroundTruth();
}

bool KttiDataset::readGroundTruth() {
    if (!Config::Get<std::string>(Config::img_gt_file).empty()) {
        std::ifstream fin(Config::Get<std::string>(Config::img_gt_file));
        if (!fin) {
            return false;
        }
        int count = 0;
        while (!fin.eof()) {
            Eigen::Matrix3d K;
            Eigen::Vector3d t;
            double projection_data[12];
            for (int k = 0; k < 12; ++k) {
                fin >> projection_data[k];
            }

            K << projection_data[0], projection_data[1], projection_data[2],
                projection_data[4], projection_data[5], projection_data[6],
                projection_data[8], projection_data[9], projection_data[10];
            t << projection_data[3], projection_data[7], projection_data[11];
            map_id_R.insert(std::make_pair(count, K));
            map_id_T.insert(std::make_pair(count, t));
            count++;
        }
        fin.close();
        return true;
    } else {
        return false;
    }
}

/**
 * @brief Get the next frame in the dataset like video frames or some else
 * continuous frames.
 *
 * @return srecon::Frame
 */
shared_ptr<Frame> KttiDataset::GetNextFrame() {
    boost::format data_fmt("%s/image_0/%06d.png");
    cv::Mat left, right;
    LOG(INFO) << (data_fmt % dataset_dir % img_index).str().c_str() << endl;
    if (access((data_fmt % dataset_dir % img_index).str().c_str(), F_OK) == -1) {
        return NULL;
    }

    left = cv::imread((data_fmt % dataset_dir % img_index).str(),
                      cv::IMREAD_GRAYSCALE);
    right = cv::imread((data_fmt % dataset_dir % img_index).str(),
                       cv::IMREAD_GRAYSCALE);

    cv::Mat resized_left, resized_right;

    cv::resize(left, resized_left, cv::Size(), 1.0, 1.0, cv::INTER_NEAREST);
    cv::resize(right, resized_right, cv::Size(), 1.0, 1.0, cv::INTER_NEAREST);

    Frame *new_frame = new Frame();
    new_frame->img = resized_left;
    new_frame->SetCamera(cameras[0]);
    if (map_id_R.find(img_index) != map_id_R.end()) {
        new_frame->gt_R = map_id_R.find(img_index)->second;
    }
    if (map_id_T.find(img_index) != map_id_T.end()) {
        new_frame->gt_T = map_id_T.find(img_index)->second;
    }
    img_index++;
    return shared_ptr<Frame>(new_frame);
}

bool KttiDataset::GetCameraPara(Eigen::Matrix<double, 3, 3> &K, Vec3 &t) {
    std::string data_dir = Config::Get<std::string>(Config::img_dataset_dir);
    std::ifstream fin(data_dir + "/calib.txt");
    if (!fin) {
        return false;
    }

    for (int i = 0; i < 1; ++i) {
        char camera_name[3];
        for (char &k : camera_name) {
            fin >> k;
        }
        double projection_data[12];
        for (int k = 0; k < 12; ++k) {
            fin >> projection_data[k];
        }

        K << projection_data[0], projection_data[1], projection_data[2],
            projection_data[4], projection_data[5], projection_data[6],
            projection_data[8], projection_data[9], projection_data[10];
        t << projection_data[3], projection_data[7], projection_data[11];
    }
    fin.close();
    return true;
}

Camera::Ptr KttiDataset::GetCamera(int id) {
    if (cameras.empty()) {
        Eigen::Matrix<double, 3, 3> K;
        Vec3 t;
        if (!GetCameraPara(K, t)) {
            return NULL;
        }

        cameras.push_back(Camera::Ptr(new Camera(K(0, 0), K(1, 1), K(0, 2), K(1, 2),
                                                 Sophus::SE3d(Sophus::SO3d(), t))));
    }
    switch (id) {
        case 0:
            return cameras[0];
        default:
            return NULL;
    }
}

}  // namespace srecon
