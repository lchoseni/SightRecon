//
// Created by lchoseni on 2022/3/17.
//
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>

#include "SSLAM/utils/utils.h"

namespace sslam{
void DrawShowPoint(int row, int col, cv::Mat img, int wait, std::string win_name){
  cv::Point center(col, row);
  cv::circle(img, center, 20, cv::Scalar(0, 0, 255), 3);
  cv::imshow(win_name, img);
  cv::waitKey(wait);
}

}