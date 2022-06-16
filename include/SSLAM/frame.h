#ifndef FRAME_H
#define FRAME_H

#include<opencv2/opencv.hpp>
#include <ceres/rotation.h>

#include "common_include.h"
#include "feature.h"
#include "camera.h"
#include "map_point.h"

namespace sslam {

class Feature;

class MapPoint;

class Frame {
 public:
  typedef shared_ptr<Frame> Ptr;
  // index for all frames, it indicates next frame id.
  // static int global_index_;
  // id of current frame
  unsigned int id_;
  std::shared_ptr<sslam::Camera> cam_;
  SE3 Tcw;
  Eigen::Matrix<double, 3, 3> R_c_w;
  Eigen::Matrix<double, 3, 1> C_c_w;
  std::vector<Feature> features_;
  cv::Mat depth;
  cv::Mat mask;

  cv::Mat left_img_, right_img_;
  std::vector<cv::KeyPoint> left_key_points_;
  std::vector<cv::KeyPoint> right_key_points_;
  std::vector<std::shared_ptr<Feature>> left_features_;
  std::vector<std::shared_ptr<Feature>> right_features_;
  cv::Mat descriptors;
  // 在descriptors或keyPoints中的序号和对应的地图点
  unordered_map<int, shared_ptr<MapPoint>> inlinePoints;

  // Key point matches between left img and right img.
  std::vector<std::vector<std::shared_ptr<Feature>>> matches;

  Frame(int id);

  void SetCamera(std::shared_ptr<Camera> &sharedPtr);

  std::shared_ptr<sslam::Camera> GetCamera();

  static unsigned int GetNextIndex();

  bool SetLeftKP(std::vector<cv::KeyPoint> &kps);

  void DrawKeyPoints();

  bool isInFrame(const Vec3 &pt_world, const SE3 &pose);

  cv::Mat getTcwMatCV(int rtype) {
    cv::Mat TcwCV, TcwCVR;
    cv::eigen2cv(Tcw.matrix(), TcwCV);
    TcwCV.convertTo(TcwCVR, rtype);
    return TcwCVR;
  }

  cv::Mat getTcw34MatCV(int rtype) {
    auto TcwCV = getTcwMatCV(rtype);
    cv::Mat Tcw34;
    TcwCV(cv::Range(0, 3), cv::Range(0, 4)).convertTo(Tcw34, rtype);
    return Tcw34;
  }

  cv::Mat getTwcMatCV(int rtype) {
    cv::Mat TwcCV, TwcCVR;
//    cv::eigen2cv(Tcw.inverse().matrix(), TwcCV);
//    TwcCV.convertTo(TwcCVR, rtype);
    return TwcCVR;
  }

  template<typename T>
  cv::Matx<T, 1, 3> getAngleAxisWcMatxCV() {
    T p[3], R[9];
    Eigen::Matrix3d r_mat = Tcw.so3().matrix();
    R[0] = r_mat(0, 0);
    R[1] = r_mat(1, 0);
    R[2] = r_mat(2, 0);
    R[3] = r_mat(0, 1);
    R[4] = r_mat(1, 1);
    R[5] = r_mat(2, 1);
    R[6] = r_mat(0, 2);
    R[7] = r_mat(1, 2);
    R[8] = r_mat(2, 2);
    ceres::RotationMatrixToAngleAxis<T>(R, p);
    cv::Matx<T, 1, 3> angleAxisCV(p[0], p[1], p[2]);
    return angleAxisCV;
  };

  template<typename T>
  void setTcw(cv::Matx<T, 2, 3> angleAxisAndTranslation) {
    double angleAxis[3] =
        {angleAxisAndTranslation(0, 0), angleAxisAndTranslation(0, 1), angleAxisAndTranslation(0, 2)};
    double quaternion[4];
    ceres::AngleAxisToQuaternion(angleAxis, quaternion);
    Eigen::Quaterniond q(quaternion[0], quaternion[1], quaternion[2], quaternion[3]);
    q.normalize();

    Tcw.so3() = SO3(q.toRotationMatrix());
    Tcw.translation() = Vec3 (angleAxisAndTranslation(1, 0),
                                 angleAxisAndTranslation(1, 1),
                                 angleAxisAndTranslation(1, 2));
  }




};

}

#endif
