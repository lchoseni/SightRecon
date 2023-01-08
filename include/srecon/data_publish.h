#pragma once

#include "common_include.h"
#include "dataset/image_dataset.h"
#include "dataset/imu_dataset.h"
#include "frame.h"

namespace srecon {

class DataPublish {
   private:
    IMUDataset::Ptr imuDataset;
    ImageDataset::Ptr imageDataset;
    vector<Frame::Ptr> frames;
    vector<IMUData::Ptr> imuData;
    unsigned int last_pub_img_idx;
    unsigned int last_pub_imu_idx;
    unsigned long int time;

   public:
    DataPublish(IMUDataset::Ptr imuDataset,
                ImageDataset::Ptr imageDataset);
    void loadAllData();
    Frame::Ptr publishImage();
    IMUData::Ptr publishIMU();
};
}  // namespace srecon