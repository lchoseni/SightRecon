#ifndef EUROC_DATASET
#define EUROC_DATASET


#include "../camera.h"
#include "../frame.h"
#include "image_dataset.h"
#include "imu_dataset.h"

namespace srecon {

typedef struct IMGIMUDATA {
    unsigned long int time;
    double p[3];
    double q[4];
    double v[3];
    double bw[3];
    double ba[3];
} ImgImuData;

class EurocDataset : public ImageDataset, public IMUDataset {
   private:
    double time = -1;
    std::string imu_dataset_dir;
    std::string image_dataset_dir;
    vector<std::string> image_names;
    vector<IMUData::Ptr> imu_datas;
    vector<ImgImuData> img_imu_gt;
    bool readImageNames();
    bool readGt();
    bool readIMUData();

   public:
    typedef std::shared_ptr<EurocDataset> Ptr;

    EurocDataset();
    Frame::Ptr GetNextFrame();
    bool setCameras();
    IMUData::Ptr getIMUData();
    Camera::Ptr GetCamera(int id);
    bool readGroundTruth();
};

}  // namespace srecon

#endif