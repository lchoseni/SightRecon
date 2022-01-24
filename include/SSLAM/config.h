#ifndef SSLAM_CONFIG_H
#define SSLAM_CONFIG_H

#include "common_include.h"

namespace sslam {

    class Config {
    private:
        static std::shared_ptr<Config> config_;
        cv::FileStorage file_;

        Config() {} // private constructor makes a singleton
    public:
        const static std::string source_type;
        const static int single;
        const static int stereo;
        const static std::string config_file_;


        ~Config(); // close the file when deconstructing

        // set a new config file
        static bool SetConfigFile(const std::string &file);

//        static cv::FileStorage GetFile(){
//            return file_;
//        }

        // access the parameter values
        template<typename T>
        static T Get(const std::string &key) {
            return T(Config::config_->file_[key]);
        }
    };
} // namespace myslam

#endif // SSLAM_CONFIG_H
