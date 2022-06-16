//
// Created by lchoseni on 2022/3/17.
//
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>

#include "SSLAM/utils/utils.h"
#include "SSLAM/common_include.h"

namespace sslam{
void DrawShowPoint(int row, int col, cv::Mat img, int wait, std::string win_name){
  cv::Point center(col, row);
  cv::circle(img, center, 20, cv::Scalar(0, 0, 255), 3);
  cv::imshow(win_name, img);
  cv::waitKey(wait);
}


/**
 * linear triangulation with SVD
 * @param poses     poses,
 * @param points    points in normalized plane
 * @param pt_world  triangulated point in the world
 * @return true if success
 */
inline bool Triangulation(const std::vector<SE3> &poses,
                   const std::vector<Vec3> points, Vec3 &pt_world) {
    MatXX A(2 * poses.size(), 4);
    VecX b(2 * poses.size());
    b.setZero();
    for (size_t i = 0; i < poses.size(); ++i) {
        Mat34 m = poses[i].matrix3x4();
        A.block<1, 4>(2 * i, 0) = points[i][0] * m.row(2) - m.row(0);
        A.block<1, 4>(2 * i + 1, 0) = points[i][1] * m.row(2) - m.row(1);
    }
    auto svd = A.bdcSvd(Eigen::ComputeThinU | Eigen::ComputeThinV);
    pt_world = (svd.matrixV().col(3) / svd.matrixV()(3, 3)).head<3>();

    if (svd.singularValues()[3] / svd.singularValues()[2] < 1e-2) {
        // 解质量不好，放弃
        return true;
    }
    return false;
}
}