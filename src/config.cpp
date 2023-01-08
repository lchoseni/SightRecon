//
// Created by yeren on 1/24/22.
//

#include "srecon/config.h"

namespace srecon {

Config* Config::config = NULL;

const std::string Config::source_type = "source_type";
const std::string Config::num_features = "num_features";
const std::string Config::rect_size = "rect_size";
const std::string Config::opt_win_size = "opt_win_size";
const std::string Config::match_threshold = "match_threshold";
const std::string Config::feature_amount_threshold = "feature_amount_threshold";
const std::string Config::inliners_threshold = "inliners_threshold";
const std::string Config::dis_threshold = "dis_threshold";
const std::string Config::img_gt_file = "img_gt_file";
const std::string Config::imu_gt_file = "imu_gt_file";
const int Config::single = 0;
const int Config::stereo = 1;


const std::string Config::img_dataset_dir = "img_dataset_dir";
const std::string Config::imu_dataset_dir = "imu_dataset_dir";

bool Config::SetConfigFile(const std::string& file) {
  if (config == NULL) {
    config = new Config;
    config->file = cv::FileStorage(file.c_str(), cv::FileStorage::READ);
  }
  return true;
}

Config::~Config() {
  if (file.isOpened()) {
    file.release();
  }
  delete config;
  config = NULL;
}

}  // namespace srecon