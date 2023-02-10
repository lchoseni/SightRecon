#include "srecon/initial/mono_initial.h"

#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/highgui/highgui.hpp>

#include "srecon/config.h"
#include "srecon/utils/utils.h"

namespace srecon {

MonoInitial::MonoInitial(ImageDataset::Ptr dataset, Map::Ptr &map,
                         cv::Ptr<cv::GFTTDetector> detecor)
    : Initial(detecor) {
  this->map = map;
  this->dataset = dataset;
  camera = dataset->GetCamera(0);
}

MonoInitial::~MonoInitial() {}

bool MonoInitial::computeRT(Frame::Ptr &frame1, Frame::Ptr &frame2,
                            vector<cv::KeyPoint> &kps1,
                            vector<cv::KeyPoint> &kps2,
                            vector<cv::Point2f> &kps1_2d,
                            vector<cv::Point2f> &kps2_2d,
                            vector<cv::Point2f> &fkps1_2d,
                            vector<cv::Point2f> &fkps2_2d, cv::Mat &cv_R,
                            cv::Mat &cv_T, cv::Mat &inlinerMask) {
  if (frame1 == NULL) {
    frame1 = dataset->GetNextFrame();
    if (frame1 == NULL) return -1;
  }
  if (kps1.size() == 0) {
    // detecor->detectAndCompute(frame1->img, cv::noArray(), kps1, desc1);
    detecor->detect(frame1->img, kps1);
    for (auto kp : kps1) {
      kps1_2d.push_back(kp.pt);
      kps2_2d.push_back(kp.pt);
    }
  };
  if (frame2 == NULL) {
    frame2 = dataset->GetNextFrame();
    if (frame2 == NULL) return -1;
  }


  vector<uchar> status;
  cv::Mat err;
  int win_size = Config::Get<int>(Config::opt_win_size);
  cv::calcOpticalFlowPyrLK(
      frame1->img, frame2->img, kps1_2d, kps2_2d, status, err,
      cv::Size(win_size, win_size), 3,
      cv::TermCriteria(cv::TermCriteria::COUNT + cv::TermCriteria::EPS, 30,
                       0.01),
      cv::OPTFLOW_USE_INITIAL_FLOW);
  int num_good_pts = 0;
  kps1.clear();
  for (size_t i = 0; i < status.size(); ++i) {
    if (status[i]) {
      fkps1_2d.push_back(kps1_2d[i]);
      fkps2_2d.push_back(kps2_2d[i]);
      kps1.push_back(cv::KeyPoint(kps1_2d[i], 7));
      kps2.push_back(cv::KeyPoint(kps2_2d[i], 7));
      num_good_pts++;
    }
  }

  if (num_good_pts < Config::Get<int>(Config::match_threshold)) {
    return false;
  }

  // cv::Mat out_img;
  // cv::drawMatches(frame1->img, kps1, frame2->img, kps2, matches, out_img);
  // cv::imshow("a", out_img);
  // cv::waitKey(0);

  cv::Mat K;
  cv::eigen2cv(camera->K(), K);

  cv::Mat E = cv::findEssentialMat(fkps1_2d, fkps2_2d, K, cv::RANSAC, 0.999,
                                   1.0, inlinerMask);
  LOG(INFO) << "Essential is\n" << E << endl;
  int nPointsFindEssentialMat = countNonZero(inlinerMask);
  DLOG(INFO) << "findEssentialMat: \n\t" << nPointsFindEssentialMat
       << " valid points, "
       << (float)nPointsFindEssentialMat * 100 / fkps1_2d.size() << "% of "
       << fkps1_2d.size() << " points are used" << endl;
  int inliners =
      cv::recoverPose(E, fkps1_2d, fkps2_2d, K, cv_R, cv_T,
                      Config::Get<int>(Config::dis_threshold));
  LOG(INFO) << "Find " << inliners << " inliners." << endl;
  if (inliners < Config::Get<int>(Config::inliners_threshold)) {
    return false;
  }
  cv::cv2eigen(cv_R, frame2->R);
  cv::cv2eigen(cv_T, frame2->T);
  return true;
}

int MonoInitial::init(Frame::Ptr &result_frame1, Frame::Ptr &result_frame2,
                      double scale) {
  Frame::Ptr frame1, frame2;
  vector<cv::KeyPoint> kps1, kps2;
  vector<cv::Point2f> kps1_2d, kps2_2d, fkps1_2d, fkps2_2d;
  while (1) {
    frame1 = frame2;
    kps1.clear();
    kps2.clear();
    kps1_2d.clear();
    kps2_2d.clear();
    fkps1_2d.clear();
    fkps2_2d.clear();

    cv::Mat K, cv_R, cv_T, inlinerMask;
    if (!computeRT(frame1, frame2, kps1, kps2, kps1_2d, kps2_2d, fkps1_2d,
                   fkps2_2d, cv_R, cv_T, inlinerMask)) {
      continue;
    }

    cv::eigen2cv(camera->K(), K);
    frame1->R = Eigen::Matrix3d::Identity();
    frame1->T = Eigen::Vector3d::Zero();

    // Triangulate points
    cv::Mat pts_3d, R1, T1;
    R1 = (cv::Mat_<double>(3, 3));
    T1 = (cv::Mat_<double>(3, 1) << 0.0, 0.0, 0.0);
    cv::setIdentity(R1);

    triangulatePts(R1, T1, cv_R, cv_T, K, fkps1_2d, fkps2_2d, pts_3d);
    LOG(INFO) << "R is\n" << cv_R << "\nT is\n" << cv_T << endl;
    // Add mappoint and features.
    vector<double> depths;
    for (int i = 0; i < pts_3d.cols; i++) {
      if (!inlinerMask.empty() && !inlinerMask.at<uint8_t>(i, 0)) continue;
      cv::Mat pt_3d = pts_3d.col(i) / pts_3d.col(i).at<float>(3, 0);

      if (pt_3d.at<float>(2, 0) <= 0) {
        continue;
      }
      depths.push_back(pt_3d.at<float>(2, 0));
      Feature::Ptr fea1(new Feature(frame1, kps1[i]));
      Feature::Ptr fea2(new Feature(frame2, kps2[i]));
      frame1->features.push_back(fea1);
      frame2->features.push_back(fea2);
      MapPoint::Ptr mp(new MapPoint(fea1, fea2, pt_3d.at<float>(0, 0),
                                    pt_3d.at<float>(1, 0),
                                    pt_3d.at<float>(2, 0)));
      fea1->setMapPoint(mp);
      fea2->setMapPoint(mp);
    }
    sort(depths.begin(), depths.end());
    scale = depths[depths.size() / 2];

    result_frame1 = frame1;
    result_frame2 = frame2;
    return 0;
  }
}
}  // namespace srecon