

#include "common_include.h"

namespace srecon {

class FeatureTrack {
 public:
  int computePyrimid(cv::Mat &frame, vector<cv::Mat> &img);
  int trackByLK(cv::Mat frame1, cv::Mat frame2, vector<cv::KeyPoint> key_points,
                int win_size);
};
}  // namespace srecon