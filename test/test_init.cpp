//
// Created by yeren on 1/24/22.
//

#include "SSLAM/common_include.h"
#include "SSLAM/front_end.h"
#include "SSLAM/config.h"
#include "SSLAM/dataset.h"
#include "SSLAM/frame.h"



int main(){
    sslam::Config::SetConfigFile(sslam::Config::config_file_);
    sslam::FrontEnd front_end;

    sslam::Dataset dataset;
    sslam::Frame frame = dataset.GetNextFrame();
    front_end.DetectFeatures(frame);
    std::cout << "Key points of left img is " << front_end.FindFeaturesInRight(frame);


    frame.DrawKeyPoints();
}
