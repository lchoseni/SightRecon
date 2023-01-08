#include "srecon/data_publish.h"

namespace srecon {
DataPublish::DataPublish(IMUDataset::Ptr imuDataset,
                         ImageDataset::Ptr imageDataset) : imuDataset(imuDataset), imageDataset(imageDataset) {
    // init time
    Frame::Ptr firstFrame = imageDataset->GetNextFrame();
    IMUData::Ptr firstIMU = imuDataset->getIMUData();
    if(firstFrame->time != firstIMU->time){
        LOG(ERROR) << "IMU data time and image data time are not right";
    }
    time = firstFrame->time;
    frames.push_back(firstFrame);
    imuData.push_back(firstIMU);
    last_pub_img_idx = last_pub_imu_idx = 1;
    loadAllData();
}

void DataPublish::loadAllData(){
    Frame::Ptr tmp_f;
    while((tmp_f = imageDataset->GetNextFrame()) != NULL ){
        frames.push_back(tmp_f);
    }
    IMUData::Ptr tmp_imu;
    while((tmp_imu = imuDataset->getIMUData()) != NULL){
        imuData.push_back(tmp_imu);
    }
}

Frame::Ptr DataPublish::publishImage() {
    return NULL;
}

}  // namespace sreconz