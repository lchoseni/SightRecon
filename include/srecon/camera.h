#ifndef SRECON_CAMERA_H
#define SRECON_CAMERA_H

#include "common_include.h"

namespace srecon {
class Camera {
   public:
    typedef std::shared_ptr<Camera> Ptr;
    double fx_, fy_, cx_, cy_;
    Sophus::SE3d pose_;
    Eigen::Matrix3d R;
    Eigen::Vector3d T;

    Camera(double fx, double fy, double cx, double cy, Sophus::SE3d pose);
    Camera(double fx, double fy, double cx, double cy, Eigen::Matrix3d R, Eigen::Vector3d T);

    ~Camera();

    Mat33 K() const {
        Mat33 k;
        k << fx_, 0, cx_, 0, fy_, cy_, 0, 0, 1;
        return k;
    }

    Vec3 World2Camera(const Vec3 &world, const Sophus::SE3d &T_w_c);

    Vec2 Camera3d2Camera2d(const Vec3 &camera);

    Vec2 Camera2Pixel(const Vec3 &camera);

    Vec2 World2Pixel(const Vec3 &world, const Sophus::SE3d &T_w_c);

    Vec3 Pixel2Camera(const Vec2 &pixel, double depth = 1);

    Vec3 Camera2World(const Vec3 &camera, const Sophus::SE3d &T_c_w);

    Vec3 Pixel2World(const Vec2 &pixel, const Sophus::SE3d &T_c_w, double depth = 1);
};
}  // namespace srecon

#endif  // SRECON_CAMERA_H
