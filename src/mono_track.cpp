#include "srecon/mono_track.h"

#include "srecon/config.h"
#include "srecon/utils/utils.h"

namespace srecon {

MonoTrack::MonoTrack(Initial *init, ImageDataset::Ptr dataset, Map::Ptr &map,
                     cv::Ptr<cv::GFTTDetector> detecor) {
  this->initializer = init;
  this->dataset = dataset;
  this->map = map;
  this->detecor = detecor;
}

bool MonoTrack::remove3dInliners(vector<MapPoint::Ptr> map_points,
                                 cv::Mat inliners) {}

bool MonoTrack::track(Frame::Ptr &frame1, Frame::Ptr &frame2) {
  // add more features
  vector<cv::KeyPoint> kps;
  frame1->detectFeature(detecor, kps);

  for (auto &kp : kps) {
    Feature::Ptr fea(new Feature(frame1, kp));
    frame1->features.push_back(fea);
  }
  cout << "\tframe1 feature size " << frame1->features.size() << endl;
  // convert features to point2d for tracking
  vector<cv::Point2f> kps1_2d, kps2_2d;
  for (Feature::Ptr &fea : frame1->features) {
    kps1_2d.push_back(cv::Point2f(fea->y, fea->x));
    kps2_2d.push_back(cv::Point2f(fea->y, fea->x));
  }
  // use optical flow method to track points
  vector<uchar> status;
  cv::Mat error;
  int win_size = Config::Get<int>(Config::opt_win_size);
  cv::calcOpticalFlowPyrLK(
      frame1->img, frame2->img, kps1_2d, kps2_2d, status, error,
      cv::Size(win_size, win_size), 3,
      cv::TermCriteria(cv::TermCriteria::COUNT + cv::TermCriteria::EPS, 30,
                       0.01),
      cv::OPTFLOW_USE_INITIAL_FLOW);

  int num_good_pts = 0;
  vector<cv::KeyPoint> no_mp_kps1, no_mp_kps2, all_kps1, all_kps2;
  vector<cv::Point2f> no_mp_kps1_2d, no_mp_kps2_2d;
  vector<Feature::Ptr> no_mp_feat1, no_mp_feat2;
  // Get all corresponding features which has mappoint.
  for (size_t i = 0; i < status.size(); ++i) {
    if (status[i]) {
      cv::KeyPoint kp1(kps1_2d[i], 7);
      cv::KeyPoint kp2(kps2_2d[i], 7);
      all_kps1.push_back(kp1);
      all_kps2.push_back(kp2);
      Feature::Ptr feat(new Feature(frame2, kp2));
      auto mp = frame1->features[i]->getMapPoint();
      if (mp) {
        feat->setMapPoint(mp);
        mp->features.push_back(feat);
      } else {
        no_mp_kps1.push_back(kp1);
        no_mp_kps2.push_back(kp2);
        no_mp_kps1_2d.push_back(kps1_2d[i]);
        no_mp_kps2_2d.push_back(kps2_2d[i]);
        no_mp_feat1.push_back(frame1->features[i]);
        no_mp_feat2.push_back(feat);
      }
      frame2->features.push_back(feat);
      num_good_pts++;
    }
  }

  LOG(INFO) << "\tFind " << num_good_pts << " good points!" << endl;

  if (num_good_pts < Config::Get<int>(Config::match_threshold)) {
    return false;
  }

  // solve by pnp
  // Get corresponding features and mappoints
  vector<cv::Point2d> pts2d;
  vector<cv::Point3d> pts3d;
  vector<MapPoint::Ptr> mp_for_pnp;
  for (Feature::Ptr &fea : frame2->features) {
    auto mp = fea->getMapPoint();
    if (mp) {
      mp_for_pnp.push_back(mp);
      pts2d.push_back(cv::Point2d(fea->y, fea->x));
      pts3d.push_back(cv::Point3d(mp->w_pos(0), mp->w_pos(1), mp->w_pos(2)));
    }
  }

  Camera::Ptr camera = dataset->GetCamera(0);
  cv::Mat K, cv_r_R, cv_R, cv_T, f1_R, f1_T, inliners_3d;
  cv::eigen2cv(camera->K(), K);

  if (!cv::solvePnPRansac(pts3d, pts2d, K, cv::Mat(), cv_r_R, cv_T, false, 100,
                          8.0, 0.99, inliners_3d)) {
    return false;
  }
  // filter inliners after pnp
  // remove map point outliners.
  vector<cv::Point2d> fpts2d;
  vector<cv::Point3d> fpts3d;
  for (int i = 0; i < inliners_3d.cols; i++) {
    if (!inliners_3d.empty() && !inliners_3d.at<uint8_t>(i, 0)) {
      mp_for_pnp[i]->reset();
    }
  }

  cv::Rodrigues(cv_r_R, cv_R);
  Eigen::Matrix3d eR;
  Eigen::Vector3d eT;
  cv::eigen2cv(frame1->R, f1_R);
  cv::eigen2cv(frame1->T, f1_T);
  cv::cv2eigen(cv_R, eR);
  cv::cv2eigen(cv_T, eT);
  frame2->R = eR;
  frame2->T = eT;
  // Find essential to recover
  cv::Mat inliners_2d;
  findEssentialMat(kps1_2d, kps2_2d, K, cv::RANSAC, 0.999, 1.0, inliners_2d);
  // triangulate
  cv::Mat pts_3d;
  if (inliners_2d.cols < Config::Get<int>(Config::inliners_threshold)) {
    triangulatePts(f1_R, f1_T, cv_R, cv_T, K, no_mp_kps1_2d, no_mp_kps2_2d,
                   pts_3d);
    for (int i = 0; i < pts_3d.cols; i++) {
      if (!inliners_2d.empty() && !inliners_2d.at<uint8_t>(i, 0)) continue;
      cv::Mat pt_3d = pts_3d.col(i) / pts_3d.col(i).at<float>(3, 0);
      Feature::Ptr fea1 = no_mp_feat1[i];
      Feature::Ptr fea2 = no_mp_feat2[i];
      MapPoint::Ptr mp(new MapPoint(fea1, fea2, pt_3d.at<float>(0, 0),
                                    pt_3d.at<float>(1, 0),
                                    pt_3d.at<float>(2, 0)));
      fea1->setMapPoint(mp);
      fea2->setMapPoint(mp);
    }
  }

  return true;
}

}  // namespace srecon
