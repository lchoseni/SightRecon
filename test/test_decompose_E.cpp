#include "SSLAM/graph.h"
#include "SSLAM/dataset.h"
#include "SSLAM/front_end.h"
#include "SSLAM/config.h"
#include "SSLAM/frame.h"


int main(){
    sslam::Config::SetConfigFile(sslam::Config::config_file_);
    sslam::FrontEnd front_end;

    sslam::Dataset dataset;
    sslam::Graph graph = sslam::Graph(&dataset, dataset.GetNextFrame());
    graph.ComputeAllRAndT();

}