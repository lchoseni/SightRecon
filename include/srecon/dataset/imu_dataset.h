#ifndef IMU_DATASET
#define IMU_DATASET

#include "../common_include.h"
#include "dataset.h"

namespace srecon {

typedef struct IMUData {
    //   typedef std::shared_ptr<IMUData> Ptr;
    double gyro[3];
    double a[3];
    unsigned int time;
} IMUData;

class IMUDataset :virtual public Dataset{
  protected:
    unsigned int imu_idx;

   public:
    virtual IMUData getIMUData() = 0;
};

}  // namespace srecon

#endif