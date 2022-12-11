//
// Created by yeren on 1/24/22.
//

#ifndef IMAGE_DATASET_H
#define IMAGE_DATASET_H

#include "camera.h"
#include "common_include.h"
#include "frame.h"
#include "dataset.h"

namespace srecon {
class ImageDataset: virtual public Dataset  {
 protected:
  unsigned int img_index;
  vector<std::shared_ptr<Camera>> cameras;
  std::map<int, Eigen::Matrix3d> map_id_R;
  std::map<int, Eigen::Vector3d> map_id_T;

 public:
  typedef shared_ptr<ImageDataset> Ptr;
  ImageDataset();

  ~ImageDataset();

  virtual Frame::Ptr GetNextFrame() = 0;

  virtual Camera::Ptr GetCamera(int id) = 0;

};
}  // namespace srecon

#endif  // SRECON_DATASET_H
