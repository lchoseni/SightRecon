//
// Created by lchoseni on 2022/3/17.
//

#ifndef SSLAM_INCLUDE_SSLAM_UTILS_UTILS_H_
#define SSLAM_INCLUDE_SSLAM_UTILS_UTILS_H_

#include <opencv2/core.hpp>

namespace sslam {

void DrawShowPoint(int row, int col, cv::Mat img, int wait, std::string win_name);


}

#endif //SSLAM_INCLUDE_SSLAM_UTILS_UTILS_H_
