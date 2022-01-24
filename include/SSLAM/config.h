#ifndef SSLAM_CONFIG_H
#define SSLAM_CONFIG_H

#include "myslam/common_include.h"

namespace sslam
{

    class Config
    {
    private:
        cv::FileStorage file_;
        Config() {} // private constructor makes a singleton
    public:
        const static std::string source_type = "source_type";
        const static int single = 0;
        const static int stereo = 1;

        ~Config(); // close the file when deconstructing

        // set a new config file
        static bool SetConfigFile(const std::string &file);

        // access the parameter values
        template <typename T>
        static T Get(const std::string &key)
        {
            return T(Config::config_->file_[key]);
        }
    };
} // namespace myslam

#endif // SSLAM_CONFIG_H
