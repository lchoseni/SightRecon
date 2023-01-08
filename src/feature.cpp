#include "srecon/feature.h"

namespace srecon {

Feature::Feature() {}

Feature::Feature(Frame::Ptr &frame, cv::KeyPoint &kp) : frame(frame) {
  x = kp.pt.y;
  y = kp.pt.x;
  Eigen::Vector3d cam_p = frame->cam->Pixel2Camera(Eigen::Vector2d(x, y), 1);
  norm_x = cam_p.x();
  norm_y = cam_p.y();
}



void Feature::setMapPoint(MapPoint::Ptr &mp) {
  map_point = mp;
}

MapPoint::Ptr Feature::getMapPoint() { return map_point; }

Feature::~Feature() {}
}  // namespace srecon