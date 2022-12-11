#include "srecon/euroc_dataset.h"

#include <boost/format.hpp>
#include <iostream>
#include <opencv2/core/core.hpp>
#include <opencv2/core/eigen.hpp>

#include "srecon/camera.h"
#include "srecon/config.h"
#include "unistd.h"

namespace srecon {

bool EurocDataset::setCameras() {
    std::string camera_file = Config::Get<std::string>(Config::img_dataset_dir) + "/sensor.yaml";
    cout << "Sensor file " << camera_file << endl;
    if (access(camera_file.c_str(), F_OK) == -1) {
        cerr << "Can not access " << camera_file << endl;
        return false;
    }
    cv::FileStorage file(camera_file.c_str(), cv::FileStorage::READ);
    cv::FileNode fn = file["intrinsics"];
    if (fn.type() != cv::FileNode::SEQ) {
        cerr << "intrinsics is not a sequence! FAIL" << endl;
        return 1;
    }
    cv::FileNodeIterator it = fn.begin(), it_end = fn.end();
    int i = 0;
    double intri[4];
    for (; it != it_end; ++it) {
        intri[i++] = (double)*it;
    }
    cameras.push_back(Camera::Ptr(new Camera(intri[0], intri[1], intri[2], intri[3], Eigen::Matrix3d::Identity(), Eigen::Vector3d::Zero())));
    return true;
}

EurocDataset::EurocDataset() {
    this->imu_dataset_dir = Config::Get<std::string>(Config::imu_dataset_dir);
    this->image_dataset_dir = Config::Get<std::string>(Config::img_dataset_dir);
    ;
    if (!readGroundTruth()) {
        cerr << "Read ground truth failed!" << endl;
    }
    if (!setCameras()) {
        cerr << "Read camera parameters failed!" << endl;
    }
}

bool EurocDataset::readGroundTruth() {
    return readImageNames() && readIMUData() && readGt();
}

bool EurocDataset::readGt() {
    std::string img_gt_file = Config::Get<std::string>(Config::img_gt_file);
    if (img_gt_file.empty() || access(img_gt_file.c_str(), F_OK) == -1) {
        cerr << "Can not access image graound truth file " << img_gt_file << endl;
        return false;
    }
    ifstream fin(img_gt_file);
    if (!fin) {
        cerr << "Can not access image graound truth file " << img_gt_file << endl;
        return false;
    }
    std::string line;
    // ignore first line
    for (int i = 0; i < 42; i++) {
        fin >> line;
    }
    double data[16];
    unsigned long int time;
    while (!fin.eof()) {
        fin >> line;
        // read each line.
        time = stoul(line.substr(0, line.find(',')));
        size_t start_pos = line.find(',') + 1;
        for (int i = 0; i < 16; i++) {
            data[i] = stod((line.substr(start_pos, line.find(',', start_pos) + 1 - start_pos)));
            start_pos = line.find(',', start_pos) + 1;
        }

        ImgImuData img_imu_data;
        img_imu_data.time = time;
        for (int i = 0; i < 3; i++)
            img_imu_data.p[i] = data[i];

        for (int i = 0; i < 4; i++)
            img_imu_data.q[i] = data[i + 3];

        for (int i = 0; i < 3; i++)
            img_imu_data.v[i] = data[i + 7];

        for (int i = 0; i < 3; i++)
            img_imu_data.bw[i] = data[i + 10];

        for (int i = 0; i < 3; i++)
            img_imu_data.ba[i] = data[i + 13];

        img_imu_gt.push_back(img_imu_data);
    }
    fin.close();
    return true;
}

bool EurocDataset::readImageNames() {
    std::string img_data_file = Config::Get<std::string>(Config::img_dataset_dir) + "/data.csv";
    if (access(img_data_file.c_str(), F_OK) == -1) {
        cerr << "Can not access " << img_data_file << endl;
        return false;
    }
    std::ifstream fin(img_data_file);
    if (!fin) {
        return false;
    }
    std::string image_name;
    // ignore first line
    fin >> image_name;
    fin >> image_name;
    while (!fin.eof()) {
        fin >> image_name;
        image_names.push_back(image_name.substr(image_name.find(',')));
    }
    fin.close();
    return true;
}

bool EurocDataset::readIMUData() {
    std::string imu_data_file = Config::Get<std::string>(Config::imu_dataset_dir) + "/data.csv";
    if (access(imu_data_file.c_str(), F_OK) == -1) {
        cerr << "Can not access " << imu_data_file << endl;
        return false;
    }
    std::ifstream fin(imu_data_file);
    if (!fin) {
        return false;
    }
    // ignore first line
    std::string tmp_str;
    std::string line;
    for (int i = 0; i < 14; i++) {
        fin >> tmp_str;
    }
    double data[6];
    unsigned long int time;
    while (!fin.eof()) {
        fin >> line;
        time = stoul(line.substr(0, line.find(',')));
        size_t start_pos = line.find(',') + 1;
        for (int i = 0; i < 6; i++) {
            data[i] = stod((line.substr(start_pos, line.find(',', start_pos) + 1 - start_pos)));
            start_pos = line.find(',', start_pos) + 1;
        }
        // data[5] = stod(line.substr(line.find(',', i) + 1, line.find(',', i + 1)));
        IMUData imu_data;
        imu_data.time = time;
        imu_data.gyro[0] = data[0];
        imu_data.gyro[1] = data[1];
        imu_data.gyro[2] = data[2];
        imu_data.a[0] = data[3];
        imu_data.a[1] = data[4];
        imu_data.a[2] = data[5];
        imu_datas.push_back(imu_data);
    }
    fin.close();
    return true;
}

Camera::Ptr EurocDataset::GetCamera(int id) { return cameras[id]; }

IMUData EurocDataset::getIMUData() {
    
    return imu_datas[imu_idx++];
}

Frame::Ptr EurocDataset::GetNextFrame() {
    boost::format data_fmt("%s/data/%s");
    std::string image_file_name = image_names[img_index];
    cv::Mat img_mat;
    cout << (data_fmt % dataset_dir % image_file_name).str().c_str() << endl;
    if (access((data_fmt % dataset_dir % image_file_name).str().c_str(), F_OK) == -1) {
        return NULL;
    }

    img_mat = cv::imread((data_fmt % dataset_dir % image_file_name).str(),
                         cv::IMREAD_GRAYSCALE);

    cv::Mat resized_img;

    cv::resize(img_mat, resized_img, cv::Size(), 1.0, 1.0, cv::INTER_NEAREST);

    Frame *new_frame = new Frame();
    new_frame->img = resized_img;
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

}  // namespace srecon