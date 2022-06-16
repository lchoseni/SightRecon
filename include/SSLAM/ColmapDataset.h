
#ifndef SSLAM_INCLUDE_SSLAM_COLMAPDATASET_H_
#define SSLAM_INCLUDE_SSLAM_COLMAPDATASET_H_


#include "dataset.h"

namespace sslam{
class ColmapDataset : public sslam::Dataset {
 public:
  shared_ptr<Frame> GetNextFrame(int idx);
  static shared_ptr<Camera> GetCamera(int id);
};

}
#endif //SSLAM_INCLUDE_SSLAM_COLMAPDATASET_H_
