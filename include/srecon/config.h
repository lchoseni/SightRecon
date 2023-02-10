#ifndef SRECON_CONFIG_H
#define SRECON_CONFIG_H

#include "common_include.h"

namespace srecon {

class Config {
 private:
  static Config *config;
  cv::FileStorage file;

  Config() {}  // private constructor makes a singleton
 public:
  const static std::string source_type;
  const static std::string num_features;
  const static std::string rect_size;
  const static std::string opt_win_size;
  const static std::string match_threshold;
  const static std::string feature_amount_threshold;
  const static std::string inliners_threshold;
  const static std::string dis_threshold;
  const static std::string imu_gt_file;
  const static std::string img_gt_file;
  const static int single;
  const static int stereo;
  static std::string config_file;
  const static std::string img_dataset_dir;
  const static std::string imu_dataset_dir;

  ~Config();  // close the file when deconstructing

  // set a new config file
  static bool SetConfigFile(const std::string &file);


  // access the parameter values
  template <typename T>
  static T Get(const std::string &key) {
    return T(Config::config->file[key]);
  }
};
}  // namespace srecon

#endif  // SRECON_CONFIG_H
