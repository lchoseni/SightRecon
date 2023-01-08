//
// Created by lchoseni on 2022/3/17.
//

#ifndef SRECON_INCLUDE_SRECON_UTILS_UTILS_H_
#define SRECON_INCLUDE_SRECON_UTILS_UTILS_H_

#include <opencv2/core.hpp>

#include "../common_include.h"
#include "../frame.h"
#include "../map_point.h"

namespace srecon {

void DrawShowPoint(int row, int col, cv::Mat img, int wait,
                   std::string win_name);

Eigen::Matrix3d skewSymmetric(Eigen::Vector3d &q);
void triangulatePts(cv::Mat R1, cv::Mat T1, cv::Mat R2, cv::Mat T2,
                    cv::Mat K, vector<cv::Point2f> &kps1_2d,
                    vector<cv::Point2f> &kps2_2d, cv::Mat &pts_3d);

template <typename T>
bool is_uninitialized(std::weak_ptr<T> const& weak) {
    using wt = std::weak_ptr<T>;
    return !weak.owner_before(wt{}) && !wt{}.owner_before(weak);
}

}  // namespace srecon

#endif  // SRECON_INCLUDE_SRECON_UTILS_UTILS_H_
