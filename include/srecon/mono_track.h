#ifndef MONO_TRACK_H
#define MONO_TRACK_H

#include "common_include.h"
#include "dataset/image_dataset.h"
#include "initial/initial.h"
#include "map.h"
#include "map_point.h"

namespace srecon {
class MonoTrack {
 public:
  Initial *initializer;
  ImageDataset::Ptr dataset;
  Map::Ptr map;
  cv::Ptr<cv::GFTTDetector> detecor;

  MonoTrack(Initial *init, ImageDataset::Ptr dataset, Map::Ptr &map, 
            cv::Ptr<cv::GFTTDetector> detecor);

  bool track(Frame::Ptr &frame1, Frame::Ptr &frame2);

  bool remove3dInliners(vector<MapPoint::Ptr> map_points, cv::Mat inliners);
};
}  // namespace srecon
#endif