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
    typedef std::shared_ptr<IMUData> Ptr;
} IMUData;

class IMUDataset : virtual public Dataset {
   protected:
    unsigned int imu_idx;

   public:
    typedef std::shared_ptr<IMUDataset> Ptr;
    virtual IMUData::Ptr getIMUData() = 0;
};

}  // namespace srecon

#endif