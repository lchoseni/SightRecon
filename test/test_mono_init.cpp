//
// Created by yeren on 1/24/22.
//

#include <unistd.h>

#include "srecon/common_include.h"
#include "srecon/config.h"
#include "srecon/fountain_dataset.h"
#include "srecon/frame.h"
#include "srecon/initial/mono_initial.h"
#include "srecon/ktti_dataset.h"
#include "srecon/mono_track.h"
#include "srecon/optimizer.h"
#include "srecon/viewer.h"

cv::Scalar get_color(float depth) {
    depth = depth * 300;
    float up_th = 50, low_th = 10, th_range = up_th - low_th;
    if (depth > up_th) depth = up_th;
    if (depth < low_th) depth = low_th;
    return cv::Scalar(255 * depth / th_range, 0, 255 * (1 - depth / th_range));
}

int main() {
    std::string config_file = "/home/yeren/OneDrive/C C++ Projects/Simple-SLAM/config/config.yaml";
    // std::string config_file = "/home/yeren/OneDrive/C C++ Projects/Simple-SLAM/config/fountain_config.yaml";
    srecon::Config::SetConfigFile(config_file);

    srecon::ImageDataset::Ptr dataset(new srecon::KttiDataset());
    // srecon::Dataset::Ptr dataset(new srecon::FountainDataset());
    srecon::Map::Ptr map(new srecon::Map());
    srecon::Viewer viewer(map);
    srecon::Optimizer optimizer;

    std::thread render(&srecon::Viewer::run, viewer);
    render.detach();

    cv::Ptr<cv::GFTTDetector> detecor = cv::GFTTDetector::create(
        srecon::Config::Get<int>(srecon::Config::num_features), 0.01, 20);
    srecon::MonoInitial initial(dataset, map, detecor);

    double scale;
    srecon::Frame::Ptr frame1, frame2;
    if (initial.init(frame1, frame2, scale) < 0) {
        std::cout << "Initialization failed!" << endl;
        return 0;
    }
    {
        srecon::Map::Ptr localMap(new srecon::Map());
        map->frames.push_back(frame1);
        map->frames.push_back(frame2);
        localMap->frames.push_back(frame1);
        localMap->frames.push_back(frame2);
    }

    cout << "R after initialization is\n"
         << frame2->R << endl
         << "T after initialization is\n"
         << frame2->T << endl;

    srecon::MonoTrack mtrack(&initial, dataset, map, detecor);

    frame1 = frame2;
    while (frame2 = dataset->GetNextFrame()) {
        if (!mtrack.track(frame1, frame2)) {
            pangolin::QuitAll();
            std::cout << "track failed!" << std::endl;
            return -1;
        }
        cout << "R after track is\n"
             << frame2->R << endl
             << "T after track is\n"
             << frame2->T << endl;
        map->frames.push_back(frame2);
        srecon::Map::Ptr localMap(new srecon::Map());
        localMap->frames.push_back(frame1);
        localMap->frames.push_back(frame2);
        // optimizer.optimize(localMap);
        frame1 = frame2;
        usleep(1000 * 200);
        if (frame1->id % 10 == 0) {
            // optimizer.optimize(map);
        }
    }
    optimizer.optimize(map);
    sleep(1000);
}
