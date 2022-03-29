//
// Created by yeren on 1/25/22.
//

#ifndef SSLAM_MAP_H
#define SSLAM_MAP_H

#include "common_include.h"
#include "map_point.h"
#include "frame.h"

namespace sslam{

class Frame;

class GMap {
 public:
  typedef shared_ptr<GMap> Ptr;
  vector<MapPoint::Ptr> mapPoints;        // all landmarks
  vector<Frame::Ptr> frames;         // all key-frames

  GMap(){
    mapPoints = vector<MapPoint::Ptr>();
    frames = vector<Frame::Ptr>();
  }

  void addFrame(Frame::Ptr frame) {
    if (frame)
      frames.push_back(frame);
  }

  void addMapPoint(MapPoint::Ptr mapPoint) {

    if (mapPoint != nullptr)
      mapPoints.push_back(mapPoint);
  }

//  void visInCloudViewer();
};
    

}

#endif //SSLAM_MAP_H
