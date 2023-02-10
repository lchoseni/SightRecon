#ifndef KTTI_DATASET
#define KTTI_DATASET

#include "image_dataset.h"

namespace srecon {

class KttiDataset : public ImageDataset {
 private:
  bool readGroundTruth();

 public:
  KttiDataset();
  Frame::Ptr GetNextFrame();

  bool GetCameraPara(Eigen::Matrix<double, 3, 3> &K, Vec3 &t);

  Camera::Ptr GetCamera(int id);
};

}  // namespace srecon

#endif