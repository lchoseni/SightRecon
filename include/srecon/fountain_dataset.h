#ifndef FOUNTAIN_DATASET
#define FOUNTAIN_DATASET

#include "image_dataset.h"

namespace srecon {
class FountainDataset : public ImageDataset {
 private:
  bool readGroundTruth();

 public:
  FountainDataset();

  virtual Frame::Ptr GetNextFrame();

  bool GetCameraPara(Eigen::Matrix<double, 3, 3> &K, Vec3 &t);

  virtual Camera::Ptr GetCamera(int id);
};
}  // namespace srecon

#endif