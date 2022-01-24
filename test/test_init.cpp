//
// Created by yeren on 1/24/22.
//

#include "SSLAM/common_include.h"
#include "SSLAM/front_end.h"
#include "SSLAM/config.h"
#include "SSLAM/dataset.h"



int main(){
    sslam::FrontEnd front_end;

    sslam::Config::SetConfigFile(sslam::Config::config_file_);
    sslam::Dataset dataset;
    dataset.GetNextFrame();
}