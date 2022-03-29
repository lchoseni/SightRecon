//
// Created by yeren on 1/24/22.
//
#include <boost/format.hpp>
#include <iostream>
#include "unistd.h"

#include "SSLAM/dataset.h"
#include "SSLAM/config.h"

namespace sslam {

Dataset::Dataset() : cur_img_index(0), dataset_dir(Config::Get<std::string>(Config::dataset_dir)) {
  cur_img_index = 5;
}

std::shared_ptr<Camera> Dataset::left_camera_ = nullptr;
std::shared_ptr<Camera> Dataset::right_camera_ = nullptr;

Dataset::~Dataset() {

}

/**
 * @brief Get the next frame in the dataset like video frames or some else continuous frames.
 *
 * @return sslam::Frame
 */
shared_ptr<Frame> Dataset::GetNextFrame() {
  boost::format data_fmt("%simages/%04d.jpg");
  boost::format ext_m_fmt("%sgt_dense_cameras/%04d.jpg.camera");
  cv::Mat left, right;
  cout << (data_fmt % dataset_dir % cur_img_index).str().c_str() << endl;
  if (access((data_fmt % dataset_dir % cur_img_index).str().c_str(), F_OK) == -1) {
    return nullptr;
  }
//  cur_img_index += 4;
  if (cur_img_index >11) {
    return nullptr;
  }

  left = cv::imread((data_fmt % dataset_dir % cur_img_index).str(), CV_LOAD_IMAGE_COLOR);
  right = cv::imread((data_fmt % dataset_dir % cur_img_index).str(), CV_LOAD_IMAGE_COLOR);

  cv::Mat resized_left, resized_right;
//    cout << left.rows << ", " << left.cols << endl;
//    cv::imshow("s", left);
//    cv::waitKey(0);
  cv::resize(left, resized_left, cv::Size(), 0.25, 0.25, cv::INTER_NEAREST);
  cv::resize(right, resized_right, cv::Size(), 0.25, 0.25, cv::INTER_NEAREST);

//  cv::imwrite((data_fmt % dataset_dir % cur_img_index).str().c_str(), resized_left);
  cout << (ext_m_fmt % dataset_dir % cur_img_index).str().c_str() << endl;

  std::ifstream fin((ext_m_fmt % dataset_dir % cur_img_index).str());
  if (!fin) {
    return nullptr;
  }

  double projection_data[12];
  for (int k = 0; k < 12; ++k) {

    fin >> projection_data[k];
  }
  for (int k = 0; k < 12; ++k) {

    fin >> projection_data[k];
  }
  Eigen::Matrix<double, 3, 3> R;
  R << projection_data[0], projection_data[1], projection_data[2],
      projection_data[3], projection_data[4], projection_data[5],
      projection_data[6], projection_data[7], projection_data[8];
  Vec3 t;
  t << projection_data[9], projection_data[10], projection_data[11];

  Frame *new_frame = new Frame();
  new_frame->left_img_ = resized_left;
  new_frame->right_img_ = resized_right;
  new_frame->SetCamera(left_camera_);
//  new_frame->Tcw = SE3(*K, *t);
  new_frame->R_c_w = R;
  new_frame->C_c_w = t;
  cur_img_index++;
  return shared_ptr<Frame>(new_frame);
}

bool Dataset::GetCameraPara(std::vector<std::shared_ptr<Eigen::Matrix<double, 3, 3>>> &Ks,
                            std::vector<std::shared_ptr<Vec3>> &ts) {
  std::string data_dir = Config::Get<std::string>(Config::dataset_dir);
  std::ifstream fin(data_dir + "/calib.txt");
  if (!fin) {
    return false;
  }

  for (int i = 0; i < 4; ++i) {
    char camera_name[3];
    for (char &k: camera_name) {
      fin >> k;
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
//           *t = K->inverse() * *t;
    *K = *K * 0.25;
    Ks.push_back(K);
    ts.push_back(t);
  }
  fin.close();
  return true;
}

std::shared_ptr<Camera> Dataset::GetCamera(int id) {
  if (left_camera_ == nullptr) {
    std::vector<std::shared_ptr<Eigen::Matrix<double, 3, 3>>> Ks;
    std::vector<std::shared_ptr<Vec3>> ts;
    if (!sslam::Dataset::GetCameraPara(Ks, ts)) {
      return nullptr;
    }

    left_camera_ = std::shared_ptr<Camera>(
        new Camera((*Ks[0])(0, 0), (*Ks[0])(1, 1), (*Ks[0])(0, 2), (*Ks[0])(1, 2),
                   Sophus::SE3d(Sophus::SO3d(), *ts[0])));
    right_camera_ = std::shared_ptr<Camera>(
        new Camera((*Ks[1])(0, 0), (*Ks[1])(1, 1), (*Ks[1])(0, 2), (*Ks[1])(1, 2),
                   Sophus::SE3d(Sophus::SO3d(), *ts[1])));
  }
  switch (id) {
    case 0:return left_camera_;
    case 1:return right_camera_;
    default:return nullptr;
  }

}

}
