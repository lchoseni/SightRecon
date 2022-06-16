#ifndef SSLAM_INCLUDE_SSLAM_FUSION_H_
#define SSLAM_INCLUDE_SSLAM_FUSION_H_

#include "common_include.h"
#include "frame.h"

namespace sslam{

  class Fusion{

   public:
    Frame::Ptr ref_img_;
    vector<Frame::Ptr> frames_;
    std::map<int, std::map<int, vector<vector<double>>>> map_row_map_col_pts;


    int fuse();

    Vec3 PixelToWorld(int &row, int &col, Mat33 &R, Vec3 &T, Mat33 &K_inv, double &depth);
      Vec3 WorldToPixel(Vec3 world, Mat33 &K);
  };

}

#endif //SSLAM_INCLUDE_SSLAM_FUSION_H_
