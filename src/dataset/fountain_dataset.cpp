#include "srecon/dataset/fountain_dataset.h"

#include <dirent.h>
#include <sys/types.h>

#include <boost/format.hpp>
#include <iostream>

#include "srecon/config.h"
#include "unistd.h"

namespace srecon {

FountainDataset::FountainDataset() : ImageDataset() {
  dataset_dir = Config::Get<std::string>(Config::img_dataset_dir);
  readGroundTruth();
}
bool FountainDataset::readGroundTruth() {
  std::string gt_file = Config::Get<std::string>(Config::img_gt_file);
  if (gt_file.empty()) {
    cerr << "Can not find ground truth file config" << gt_file << endl;
    return false;
  }

  int count = 0;
  Eigen::Matrix3d init_R;
  Eigen::Vector3d init_T;
  while (1) {
    boost::format data_fmt("%s/%04d.png.camera");
    LOG(INFO) << " ground truth " << (data_fmt % gt_file % count).str().c_str()
         << endl;
    if (access((data_fmt % gt_file % count).str().c_str(), F_OK) == -1) {
      cerr << "Can not find ground truth " << (data_fmt % gt_file % count).str()
           << endl;
      return false;
    }
    std::ifstream fin((data_fmt % gt_file % count).str());
    if (!fin) {
      return false;
    }
    while (!fin.eof()) {
      Eigen::Matrix3d R;
      Eigen::Vector3d t;
      double tmp[12];
      double projection_data[12];
      for (int k = 0; k < 12; ++k) {
        fin >> tmp[k];
      }
      for (int k = 0; k < 12; ++k) {
        fin >> projection_data[k];
      }

      R << projection_data[0], projection_data[1], projection_data[2],
          projection_data[3], projection_data[4], projection_data[5],
          projection_data[6], projection_data[7], projection_data[8];

      t << projection_data[9], projection_data[10], projection_data[11];
      if (count == 0) {
        init_R = R;
        init_T = t;
      }
    //   map_id_R.insert(std::make_pair(count, R));
    //   map_id_T.insert(std::make_pair(count, t));
        map_id_R.insert(std::make_pair(count, R * init_R.transpose()));
        map_id_T.insert(
            std::make_pair(count, R * -(init_R.transpose() * init_T) + t));
      //   map_id_R.insert(std::make_pair(count, R.transpose()*init_R));
      //   map_id_T.insert(std::make_pair(count, R.transpose() *init_T -
      //   R.transpose() * t ));
    }
    count++;
    fin.close();
  }
  return true;
}

/**
 * @brief Get the next frame in the dataset like video frames or some else
 * continuous frames.
 *
 * @return srecon::Frame
 */
shared_ptr<Frame> FountainDataset::GetNextFrame() {
  boost::format data_fmt("%s/%04d.png");
  cv::Mat left;
  LOG(INFO) << (data_fmt % dataset_dir % img_index).str().c_str() << endl;
  if (access((data_fmt % dataset_dir % img_index).str().c_str(), F_OK) == -1) {
    return NULL;
  }

  left = cv::imread((data_fmt % dataset_dir % img_index).str(),
                    cv::IMREAD_GRAYSCALE);

  cv::Mat resized_left;

  cv::resize(left, resized_left, cv::Size(), 0.25, 0.25, cv::INTER_NEAREST);

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

bool FountainDataset::GetCameraPara(Eigen::Matrix<double, 3, 3> &K, Vec3 &t) {
  std::string gt_file = Config::Get<std::string>(Config::img_gt_file);
  std::ifstream fin(gt_file);
  if (!fin) {
    return false;
  }

  for (int i = 0; i < 1; ++i) {
    double projection_data[12];
    for (int k = 0; k < 12; ++k) {
      fin >> projection_data[k];
    }

    K << projection_data[0], projection_data[1], projection_data[2],
        projection_data[3], projection_data[4], projection_data[5],
        projection_data[6], projection_data[7], projection_data[8];
    t << projection_data[9], projection_data[10], projection_data[11];
  }
  fin.close();
  return true;
}

Camera::Ptr FountainDataset::GetCamera(int id) {
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
