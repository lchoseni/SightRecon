//
// Created by yeren on 1/24/22.
//

#include "SSLAM/common_include.h"
#include "SSLAM/camera.h"


namespace sslam {

    Camera::Camera(double fx, double fy, double cx, double cy, Sophus::SE3d pose)
        : fx_(fx), fy_(fy), cx_(cx), cy_(cy), pose_(pose) {

    }

    Camera::~Camera(){

    }

    Vec3 Camera::World2Camera(const Vec3 &world, const Sophus::SE3d &T_w_c) {
        return pose_ * T_w_c * world;
    }

    Vec2 Camera::Camera3d2Camera2d(const Vec3 &camera){
        return Vec2(
                    camera(0, 0) / camera(2, 0),
                    camera(1, 0) / camera(2, 0));
    }

    Vec2 Camera::Camera2Pixel(const Vec3 &camera) {
        return Vec2(
                fx_ * camera(0, 0) / camera(2, 0) + cx_,
                fy_ * camera(1, 0) / camera(2, 0) + cy_);
    }

    Vec2 Camera::World2Pixel(const Vec3 &world, const Sophus::SE3d &T_w_c) {
        return Camera2Pixel(World2Camera(world, T_w_c));
    }

    Vec3 Camera::Pixel2Camera(const Vec2 &pixel, double depth) {
        return Vec3(
                (pixel(0, 0) - cx_) * depth / fx_,
                (pixel(0, 1)) - cy_ * depth / fy_,
                depth
        );
    }

    Vec3 Camera::Camera2World(const Vec3 &camera, const Sophus::SE3d &T_c_w) {
        return T_c_w.inverse() * pose_.inverse() * camera;
    }

    Vec3 Camera::Pixel2World(const Vec2 &pixel, const Sophus::SE3d &T_c_w, double depth) {
        return Camera2World(Pixel2Camera(pixel, depth), T_c_w);
    }

}
