#include "SSLAM/graph.h"
#include "SSLAM/dataset.h"
#include "SSLAM/front_end.h"
#include "SSLAM/config.h"
#include "SSLAM/frame.h"

int main(){
    sslam::Config::SetConfigFile(sslam::Config::config_file_);
    sslam::FrontEnd front_end;

    sslam::Dataset dataset;
    sslam::Frame frame1 = dataset.GetNextFrame();
    front_end.DetectFeatures(frame1);
    front_end.FindFeaturesInRight(frame1);
    sslam::Frame frame2 = dataset.GetNextFrame();
    frame2 = dataset.GetNextFrame();

    frame2 = dataset.GetNextFrame();

    frame2 = dataset.GetNextFrame();

    frame2 = dataset.GetNextFrame();

    frame2 = dataset.GetNextFrame();

    frame2 = dataset.GetNextFrame();

    frame2 = dataset.GetNextFrame();

    frame2 = dataset.GetNextFrame();
    frame2 = dataset.GetNextFrame();
    frame2 = dataset.GetNextFrame();
    frame2 = dataset.GetNextFrame();

    frame2 = dataset.GetNextFrame();
    frame2 = dataset.GetNextFrame();
    frame2 = dataset.GetNextFrame();
    frame2 = dataset.GetNextFrame();
    frame2 = dataset.GetNextFrame();
    frame2 = dataset.GetNextFrame();
    frame2 = dataset.GetNextFrame();
    frame2 = dataset.GetNextFrame();
    frame2 = dataset.GetNextFrame();
    frame2 = dataset.GetNextFrame();
    frame2 = dataset.GetNextFrame();
    frame2 = dataset.GetNextFrame();
    frame2 = dataset.GetNextFrame();
    frame2 = dataset.GetNextFrame();
    frame2 = dataset.GetNextFrame();
    frame2 = dataset.GetNextFrame();
    frame2 = dataset.GetNextFrame();
    frame2 = dataset.GetNextFrame();

    front_end.DetectFeatures(frame2);
    sslam::Graph graph = sslam::Graph();
    graph.ComputeRAndTOfTwoImgs(frame1, frame2);

}