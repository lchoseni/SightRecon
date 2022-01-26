//
// Created by yeren on 1/24/22.
//

#ifndef SSLAM_CAMERA_H
#define SSLAM_CAMERA_H

#include "common_include.h"

namespace sslam {
    class Camera {
    public:
        double fx_, fy_, cx_, cy_;
        Sophus::SE3d pose_;

        Camera(double fx, double fy, double cx, double cy);
        ~Camera();

        Eigen::Matrix<double, 3, 3> K() const {
            Eigen::Matrix<double, 3, 3> k;
            k << fx_, 0, cx_, 0, fy_, cy_, 0, 0, 1;
            return k;
        }

        Vec3 World2Camera(const Vec3 &world, const Sophus::SE3d &T_w_c);

        Vec2 Camera2Pixel(const Vec3 &camera);

        Vec2 World2Pixel(const Vec3 &world, const Sophus::SE3d &T_w_c);

        Vec3 Pixel2Camera(const Vec2 &pixel, double depth = 1);

        Vec3 Camera2World(const Vec3 &camera, const Sophus::SE3d &T_c_w);

        Vec3 Pixel2World(const Vec2 &pixel, const Sophus::SE3d &T_c_w, double depth = 1);

    };
}

#endif //SSLAM_CAMERA_H
