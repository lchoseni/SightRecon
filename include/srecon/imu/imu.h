#include <opencv2/opencv.hpp>
#include <vector>

#include "../common_include.h"
#include "../frame.h"

using namespace std;
using namespace Eigen;

namespace srecon {

class IMUInterval;

class IMU {
 private:
  vector<IMUInterval> imu_intervals;

 public:
  IMU(/* args */);
  // Each interval accumulated q should be same as frame rotation.
  bool computeGyroBias(vector<Frame> &image_frames, Vector3d &delta_bias_g);
  // Calibrate the direction of g, recover s and get velocity of each frame.
  bool alignVisual(vector<Frame> &image_frames, size_t start, size_t end, Vector3d &g);

  void computeExtraRotation(vector<Frame> &image_frames, size_t start, size_t end);
  void perpendicularBasis(Vector3d &g, MatrixXd &basis);
  void gravityBasis(Vector3d &gravity);
  void refineGravity(vector<Frame> &image_frames, size_t start, size_t end, Vector3d &g);
  ~IMU();
};

class IMUInterval {
 private:
  Vector3d bias_gyro, bias_a;
  // indicate how many timestamps in this time interval.

 public:
  int count;
  vector<Vector3d> acce_as, gyro_ws;
  vector<double> each_dt;
  double dt;
  vector<cv::Mat> Fs, Gs;
  Matrix<double, 15, 15> jacobian, covariance;

  // Quaternion from gravity to current pose.
  vector<Eigen::Quaterniond> each_inte_q;
  Vector3d inte_p, inte_v;
  Quaterniond inte_q;
  IMUInterval(int count_) : count(count_) {
    acce_as = vector<Eigen::Vector3d>(count);
    gyro_ws = vector<Eigen::Vector3d>(count);
    Fs = vector<cv::Mat>(count);
    Gs = vector<cv::Mat>(count);
  }
  // Compute Integration between two timestamps.
  void midPointIntegration(int start, int end, float &a, float &w);
  void computeJacobian(Vector3d &mid_gyro, Vector3d &acc_start,
                                  Vector3d &acc_end, Vector3d &bias_a, Quaterniond &pose_start, Quaterniond &pose_end, double &dt);
};


}  // namespace srecon