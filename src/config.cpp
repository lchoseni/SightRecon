//
// Created by yeren on 1/24/22.
//

#include "SSLAM/config.h"

namespace sslam {

    Config* Config::config_ = nullptr;


    const std::string Config::source_type = "source_type";
    const int Config::single = 0;
    const int Config::stereo = 1;
    const std::string Config::config_file_ = "/home/yeren/test/Simple-SLAM/config/config.yaml";
//    const std::string Config::config_file_ = "/Users/lchoseni/Projects Stored In Git/C++/Simple-SLAM/config/config.yaml";
    const std::string Config::dataset_dir = "dataset_dir";

    bool Config::SetConfigFile(const std::string &file) {
        if (config_ == nullptr) {
            config_ = new Config;
            config_->file_ = cv::FileStorage(file.c_str(), cv::FileStorage::READ);
        }
        return true;
    }

    Config::~Config() {
        if (file_.isOpened()){
            file_.release();
        }
    }


}