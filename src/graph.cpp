#include <opencv2/core.hpp>
#include <time.h>
#include <math.h>
#include <memory>
#include <random>
#include <iostream>
#include <unistd.h>
#include <Eigen/Core>
#include <Eigen/Dense>

#include "SSLAM/graph.h"
#include "SSLAM/common_include.h"
#include "SSLAM/camera.h"
#include "SSLAM/frame.h"
#include "SSLAM/dataset.h"
#include "SSLAM/utils/utils.h"
#include "SSLAM/map_point.h"
#include "SSLAM/map.h"
#include "SSLAM/BA.h"

using namespace cv;

namespace sslam {

Graph::Graph(Dataset *dataset, shared_ptr<Frame> ref_img) : dataset_(dataset), ref_img_(ref_img) {
  ref_height_ = ref_img_->left_img_.rows;
  ref_width_ = ref_img_->left_img_.cols;
  depth = cv::Mat(ref_height_, ref_width_, CV_64F);
  mask = cv::Mat(ref_height_, ref_width_, CV_64F);
  int di[3] = {ref_height_, ref_width_, 3};
  normals = cv::Mat(3, di, CV_64F, cv::Scalar(0));
  depth_max = 15;
  depth_min = 0;
  Eigen::Matrix<double, 2, 2> tran_prob;
  tran_prob << 0.999, 0.001, 0.001, 0.999;
  hmm = new Hmm(tran_prob);


  // start_row = 225, end_row = 250, start_col = 0, end_col = ref_width_;
  start_row = 0, end_row = ref_height_, start_col = 0, end_col = ref_width_;
  init_start_row = start_row, init_end_row = end_row, init_start_col = start_col, init_end_col = end_col;
  cout << ref_img->left_img_.rows << " x " << ref_img->left_img_.cols << endl;
  rotate = 0;
  g_map_ = std::make_shared<GMap>();
  pose_in_dataset = false;
  simple = false;
  apply_ba = false;

  ncc_norm_factor_ = 2.0 / (sqrt(2.0 * 3.1415926) * 0.6 *
      erf(2.0 / (0.6 * 1.414213562)));
  ncc_norm_factor_ = 1;

  std::random_device theta_rd;
  std::random_device pie_rd;
  theta_gen = std::mt19937(theta_rd());
  pie_gen = std::mt19937(pie_rd());

  rand_theta_dist = uniform_real_distribution<double>(0, 360);
  rand_pie_dist = uniform_real_distribution<double>(0, 60);
  perturb_theta_dist = uniform_real_distribution<double>(-90, 90);
  perturb_pie_dist = uniform_real_distribution<double>(-15, 15);

  cv::Mat K;
  cv::eigen2cv(ref_img->cam_->K(), K);
  ref_inv_K[0] = 1 / K.at<double>(0, 0);
  ref_inv_K[1] = -K.at<double>(0, 2) * ref_inv_K[0];
  ref_inv_K[2] = 1 / K.at<double>(1, 1);
  ref_inv_K[3] = -K.at<double>(1, 2) * ref_inv_K[2];
  cout << "K is " << ref_img->GetCamera()->K() << endl;
  cout << "normal size is " << normals.size << endl;
}

Graph::~Graph() {
  delete hmm;
}

void Graph::InitialRandomDepth() {
  std::random_device rd;
  std::mt19937 gen(rd());
  uniform_real_distribution<double> uni_dist(depth_min, depth_max);
  for (int row = 0; row < ref_height_; row++) {
    for (int col = 0; col < ref_width_; ++col) {
      depth.at<double>(row, col) = (double) uni_dist(gen);
      mask.at<double>(row, col) = 0;
    }
  }
}

void Graph::GenerateRandomNormal(double *rand_normal) {

  double theta = rand_theta_dist(theta_gen);
  double pie = rand_pie_dist(pie_gen);
  double sum = 0;
  rand_normal[0] = cos(theta) * sin(pie);
  rand_normal[1] = sin(theta) * sin(pie);
  rand_normal[2] = cos(pie);
  sum += rand_normal[0] * rand_normal[0];
  sum += rand_normal[1] * rand_normal[1];
  sum += rand_normal[2] * rand_normal[2];
  sum = sqrt(sum);

  rand_normal[0] /= sum;
  rand_normal[1] /= sum;
  rand_normal[2] /= sum;


}

void Graph::InitialRandomNormal(const cv::Mat &K) {

  double rand_normal[3];
  for (int row = 0; row < ref_height_; row++) {
    for (int col = 0; col < ref_width_; ++col) {

      GenerateRandomNormal(rand_normal);

      normals.at<double>(row, col, 0) = rand_normal[0];
      normals.at<double>(row, col, 1) = rand_normal[1];
      normals.at<double>(row, col, 2) = rand_normal[2];

    }
  }
}

void Graph::PerturbNormal(double normal[3]) {
  double theta = perturb_theta_dist(gen);
  double pie = perturb_pie_dist(gen);

  double ori_pie = acos(normal[2]);
  double ori_theta = atan(normal[1] / normal[0]);

  normal[0] += cos(ori_theta + theta) * sin(ori_pie + pie);
  normal[1] += sin(ori_theta + theta) * sin(ori_pie + pie);
  normal[2] += cos(ori_pie + pie);
}

void FilterMatches(vector<DMatch> &matches, vector<DMatch> &good_matches) {
  double min_dist = 10000, max_dist = 0;

  if (matches.empty()) {
    return;
  }

  for (int i = 0; i < matches.size(); i++) {
    double dist = matches[i].distance;
    if (dist < min_dist) min_dist = dist;
    if (dist > max_dist) max_dist = dist;
  }

  for (int i = 0; i < matches.size(); i++) {
    if (matches[i].distance <= max(0.0, 4 * min_dist)) {
      good_matches.push_back(matches[i]);
    }
  }
}

bool Graph::ComputeRAndTOfTwoImgs(shared_ptr<Frame> &frame1,
                                  shared_ptr<Frame> &frame2,
                                  cv::Mat &R_,
                                  cv::Mat &t_,
                                  cv::Mat &points4D_,
                                  cv::Mat &inlinerMask_,
                                  vector<cv::DMatch> &matches_) {

  if (frame1 == nullptr || frame2 == nullptr) {
    return false;
  }

  cv::Mat descriptors_1, descriptors_2;
  cv::Ptr<cv::Feature2D> detector = cv::SIFT::create();
  cv::Ptr<cv::DescriptorMatcher> matcher = cv::DescriptorMatcher::create("BruteForce");

  if (init) {
    front_end_->DetectFeatures(frame1);
    detector->compute(frame1->left_img_, frame1->left_key_points_, descriptors_1);
    descriptors_1.copyTo(frame1->descriptors);
  } else {
    descriptors_1 = frame1->descriptors;
  }
  front_end_->DetectFeatures(frame2);
  detector->compute(frame2->left_img_, frame2->left_key_points_, descriptors_2);
  descriptors_2.copyTo(frame2->descriptors);

  vector<cv::DMatch> matches, good_matches;
  matcher->match(descriptors_1, descriptors_2, matches);
  FilterMatches(matches, good_matches);

  std::vector<cv::Point2d> pts1, pts2;
  std::vector<cv::KeyPoint> ky_pts1, ky_pts2;
  for (auto &match: good_matches) {
    pts1.push_back(frame1->left_key_points_[match.queryIdx].pt);
    pts2.push_back(frame2->left_key_points_[match.trainIdx].pt);
  }

  if (pts1.size() < 8 || pts2.size() < 8) {
    return false;
  }
  id_to_H[frame1->id_][frame2->id_] = cv::findHomography(pts1, pts2, RANSAC, 3);

  cv::Mat K1, K2;

  cv::eigen2cv(frame1->GetCamera()->K(), K1);
  cv::eigen2cv(frame2->GetCamera()->K(), K2);
  if (init) {
    cv::Mat inlineMask, points4D;
    cv::Mat ess = cv::findEssentialMat(pts1, pts2, frame1->cam_->getFocalLength(),
                                       frame1->cam_->getPrincipalPoint(),
                                       RANSAC, 0.999, 1.0, inlineMask);
    cout << "Essential matrix is " << endl << ess / ess.at<double>(2, 2) << endl;
    int nPointsFindEssentialMat = countNonZero(inlineMask);
    cout << "findEssentialMat: \n\t" << nPointsFindEssentialMat << " valid points, " <<
         (double) nPointsFindEssentialMat * 100 / pts1.size()
         << "% of " << pts1.size() << " points are used" << endl;
    if (id_to_H.count(frame1->id_) <= 0) {
      id_to_H.insert(make_pair(frame1->id_, map<unsigned int, cv::Mat>()));
    }
    if (id_to_H[frame1->id_].count(frame2->id_) <= 0) {
      id_to_H[frame1->id_].insert(make_pair(frame2->id_, cv::Mat()));
    }

    cv::Mat R, t;
    if (cv::recoverPose(ess, pts1, pts2, K1, R, t, 100, inlineMask,
                        points4D) <= 0) {
      return false;
    }
    if (inlineMask.rows < 30) {
      return false;
    }
    R_ = R;
    t_ = t;
    points4D.copyTo(points4D_);
    inlineMask.copyTo(inlinerMask_);
    matches_ = good_matches;
    init = !apply_ba;

    Eigen::Matrix3d eigenR2;
    cv2eigen(R, eigenR2);
    frame2->Tcw = SE3(
        eigenR2,
        Vec3(t.at<double>(0, 0), t.at<double>(1, 0), t.at<double>(2, 0))
    );

    return true;
  } else {

    cv::Mat descriptors;
    vector<Point3d> points3d;
    for (MapPoint::Ptr &point: g_map_->mapPoints) {
      if (frame1->isInFrame(point->pos, frame1->Tcw)) {
        descriptors.push_back(point->descriptor);
        points3d.push_back(Point3d(point->pos(0), point->pos(1), point->pos(2)));
      }
    }

    matches.clear(), good_matches.clear();
    matcher->match(descriptors, frame2->descriptors, matches, cv::noArray());

    FilterMatches(matches, good_matches);
    if (good_matches.empty()) {
      return false;
    }

    // solve pnp
    vector<Point2f> points2DPnP;
    vector<Point3f> points3DPnP;
    for (auto match: good_matches) {
      points2DPnP.push_back(frame2->left_key_points_[match.trainIdx].pt);
      points3DPnP.push_back(points3d[match.queryIdx]);
    }

    Mat r, t, indexInline;
    solvePnPRansac(points3DPnP, points2DPnP, K1,
                   cv::noArray(), r, t, false, 100, 8.0, 0.99,
                   indexInline);
    Mat R;
    cv::Rodrigues(r, R);
    Eigen::Matrix3d eigenR2;
    cv2eigen(R, eigenR2);
    frame2->Tcw = SE3(
        SO3(eigenR2),
        Eigen::Vector3d(t.at<double>(0, 0), t.at<double>(1, 0), t.at<double>(2, 0))
    );

    matches.clear();
    good_matches.clear();
    matcher->match(frame1->descriptors, frame2->descriptors, matches, noArray());
    FilterMatches(matches, good_matches);

    if (good_matches.empty() || good_matches.size() < 10) {
      return false;
    }

    vector<Point2f> matchPoints1, matchPoints2;
    for (auto match: good_matches) {
      matchPoints1.push_back(frame1->left_key_points_[match.queryIdx].pt);
      matchPoints2.push_back(frame2->left_key_points_[match.trainIdx].pt);
    }
    Mat inlineMask;
    findEssentialMat(matchPoints1, matchPoints2,
                     frame1->cam_->getFocalLength(),
                     frame2->cam_->getPrincipalPoint(),
                     RANSAC, 0.999, 1.0, inlineMask);

    int nPointsFindEssentialMat = countNonZero(inlineMask);
    cout << "findEssentialMat: \n\t" << nPointsFindEssentialMat << " valid points, " <<
         (double) nPointsFindEssentialMat * 100 / matchPoints1.size()
         << "% of " << matchPoints1.size() << " points are used" << endl;
    vector<Point2f> matchPointsNorm1, matchPointsNorm2;
    matchPointsNorm1.reserve(good_matches.size());
    matchPointsNorm2.reserve(good_matches.size());
    for (auto &match: good_matches) {
      matchPointsNorm1.push_back(frame1->cam_->pixel2normal(frame1->left_key_points_[match.queryIdx].pt));
      matchPointsNorm2.push_back(frame2->cam_->pixel2normal(frame2->left_key_points_[match.trainIdx].pt));

    }
    Mat points4d;
    triangulatePoints(frame1->getTcw34MatCV(CV_64F), frame2->getTcw34MatCV(CV_64F),
                      matchPointsNorm1, matchPointsNorm2, points4d);
    R_ = R;
    t_ = t;
    points4d.copyTo(points4D_);
    inlineMask.copyTo(inlinerMask_);
    matches_ = good_matches;

    return true;
  }

}

double PerturbDepth(const double depth,
                    std::mt19937 gen) {

  const double depth_min = 0.75 * depth;
  const double depth_max = 1.25 * depth;
  uniform_real_distribution<double> uni_dist(depth_min, depth_max);
  return uni_dist(gen) ;
}

void Graph::AddMapPoint(Frame::Ptr &frame1, Frame::Ptr &frame2, RELA_RT &rt) {
  for (int i = 0; i < rt.points4D.cols; i++) {
    MapPoint::Ptr mapPoint;

    if (!rt.inlineMask.empty() && !rt.inlineMask.at<uint8_t>(i, 0))
      continue;

    //获取描述子
    cv::Mat descriptor = frame2->descriptors.row(rt.matches[i].trainIdx);
    if (frame1->inlinePoints.find(rt.matches[i].queryIdx) != frame1->inlinePoints.end()) {
      mapPoint = frame1->inlinePoints[rt.matches[i].queryIdx];
      mapPoint->descriptor = descriptor;
      mapPoint->addObservedFrame(
          frame2, frame2->left_key_points_[rt.matches[i].trainIdx].pt);
      //记录当前帧加入地图的mapPoint和特征点下标
      frame2->inlinePoints[rt.matches[i].trainIdx] = mapPoint;

    } else {

      // convert to homogenous coordinate
      Mat x = rt.points4D.col(i);

      Vec3b rgb;
      if (frame2->left_img_.type() == CV_8UC3) {
        rgb = frame2->left_img_.at<Vec3b>(frame2->left_key_points_[rt.matches[i].trainIdx].pt);
        swap(rgb[0], rgb[2]);
      } else if (frame2->left_img_.type() == CV_8UC1) {
//        cvtColor(frame2->left_img_.at<uint8_t>(frame2->left_key_points_[rt.matches[i].trainIdx].pt),
//                 rgb,
//                 COLOR_BGR2GRAY);
        rgb[0] = 255;
        rgb[1] = 255;
        rgb[2] = 255;
      }

      if (x.type() == CV_32FC1) {
        x /= x.at<float>(3, 0); // 归一化
        mapPoint = MapPoint::Ptr(new MapPoint(Vec3(x.at<float>(0, 0),
                                                   x.at<float>(1, 0),
                                                   x.at<float>(2, 0)),
                                              descriptor, rgb
        ));
      } else if (x.type() == CV_64FC1) {
        x /= x.at<double>(3, 0);
        mapPoint = MapPoint::Ptr(new MapPoint(Vec3(x.at<double>(0, 0),
                                                   x.at<double>(1, 0),
                                                   x.at<double>(2, 0)),
                                              descriptor, rgb
        ));
      }

      frame2->inlinePoints[rt.matches[i].trainIdx] = mapPoint;

      mapPoint->addObservedFrame(frame1, frame1->left_key_points_[rt.matches[i].queryIdx].pt);
      mapPoint->addObservedFrame(frame2, frame2->left_key_points_[rt.matches[i].trainIdx].pt);

      g_map_->addMapPoint(mapPoint);

    }
  }
}

void Graph::ComputeAllRAndT() {

  shared_ptr<Frame> frame = nullptr;
  while ((frame = dataset_->GetNextFrame(-1)) != nullptr) {
      readed_frames.push_back(frame);
  }
  frames.clear();
  g_map_->addFrame(ref_img_);

  for (shared_ptr<Frame> qualified_frame: readed_frames) {
    cv::Mat R, t, points4D, inlinerMask;
    vector<cv::DMatch> matches;
    cout << endl << endl << "Compute R T between " << ref_img_->id_ << " and " << qualified_frame->id_ << endl;
    if (!ComputeRAndTOfTwoImgs(ref_img_, qualified_frame, R, t, points4D, inlinerMask, matches)) {
      cout << "Can not get R and T from " << qualified_frame->id_ << endl;
      continue;
    }
    frames.push_back(qualified_frame);



    cout << "R and t is " << R / R.at<double>(2, 2) << endl << t << endl;
//    cout << "data from file is " << id_to_trans[ref_img_->id_][frame->id_].so3().matrix() << endl<< id_to_trans[ref_img_->id_][frame->id_].translation().matrix() << endl;
    RELA_RT rela_rt;

    cv::Mat R_i, C_i, R_j, C_j;
    cv::eigen2cv(ref_img_->R_c_w, R_i);
    cv::eigen2cv(ref_img_->C_c_w, C_i);
    cv::eigen2cv(qualified_frame->R_c_w, R_j);
    cv::eigen2cv(qualified_frame->C_c_w, C_j);
    rela_rt.R_i = R_i;
    rela_rt.C_i = C_i;
    rela_rt.R_j = R_j;
    rela_rt.C_j = C_j;
//    cout << R_j << R_i <<endl;
    rela_rt.R = R;
    rela_rt.T = t;
    rela_rt.points4D = points4D;
    rela_rt.inlineMask = inlinerMask;
    rela_rt.matches = matches;
    if (id_to_RTs_.count(ref_img_->id_) <= 0) {
      map<unsigned int, RELA_RT> map;
      map.insert(make_pair(qualified_frame->id_, rela_rt));
      id_to_RTs_.insert(make_pair(ref_img_->id_, map));
    } else {
      id_to_RTs_[ref_img_->id_].insert(make_pair(qualified_frame->id_, rela_rt));
    }

    if (apply_ba) {
      AddMapPoint(ref_img_, qualified_frame, rela_rt);
      g_map_->addFrame(qualified_frame);
      BA ba;
      ba(g_map_);
    }

  }

  if (apply_ba) {
//    g_map_->visInCloudViewer();
  }

//  cout << "========ref img R and T is=========" << endl << ref_img_->Tcw.rotationMatrix() << endl << ref_img_->Tcw.translation() << endl;
//
//  for (auto frame: frames) {
//    cout << "==========" << frame->Tcw.rotationMatrix() / frame->Tcw.rotationMatrix().data()[8] << endl << frame->Tcw.translation() << endl;
//  }
}

double Graph::ComputeNCC(Frame &ref,
                         Frame &src,
                         int row_pix,
                         int col_pix,
                         int win_size,
                         double depth_pix,
                         double normal[3],
                         cv::Mat K_src,
                         cv::Mat K_ref) {
  double max_cost = 2.0;
  RELA_RT rela_rt = id_to_RTs_[ref.id_][src.id_];
  cv::Mat H(3, 3, CV_64F);
  double depths[14] = {0.1, 0.2, 0.5, 0.9, 1, 2, 5, 10, 20, 100, 500, 1000, 10000, 100000};
  double normals[5][3] = {{1, 1, 1,}, {1, -1, 1}, {-1, 1, 1}, {-1, -1, 1}, {0, 0, 1}};

  cv::Mat R, T;
//  cv::eigen2cv(src.Tcw.rotationMatrix(), R);
//  cv::eigen2cv(src.Tcw.translation(), T);

  R = rela_rt.R_j.t() * rela_rt.R_i;
  T = rela_rt.R_j.t() * (-rela_rt.C_j + rela_rt.C_i);
  double n[3]= {0, 0, -1};
  ComputeHomography(K_src,
                    K_ref,
                    R, T,
                    depth_pix,
                    H, row_pix, col_pix, normal);


  // Compute the image coordinate after homography warping.
  // If it's outside the boarder, ignore it and return the minimum NCC.
  int half_win = win_size / 2;
  cv::Mat ref_coor = cv::Mat(3, 1, CV_64F);
  double src_col, src_row;
  double refs[win_size * win_size], srcs[win_size * win_size];
  int idx = 0;

  cv::Mat p(3, 1, CV_64F);
  cv::Mat K_inv = K_ref.inv();
  for (int win_row = -half_win; win_row <= half_win; ++win_row) {
    for (int win_col = -half_win; win_col <= half_win; ++win_col) {
      ref_coor.at<double>(0, 0) = (double) (col_pix + win_col);
      ref_coor.at<double>(1, 0) = (double) (row_pix + win_row);
      ref_coor.at<double>(2, 0) = 1.0;
      cv::Mat src_coor = H * ref_coor;
//       p.at<double>(0, 0) = (double ) (col_pix +win_col);
//       p.at<double>(1, 0) = (double ) (row_pix +win_row);
//       p.at<double>(2, 0) = 1.0;
//      cv::Mat src_coor = (K_ref * (R * depth_pix * K_inv * p + T));
      src_col = src_coor.at<double>(0, 0) / src_coor.at<double>(2, 0);
      src_row = src_coor.at<double>(1, 0) / src_coor.at<double>(2, 0);
      if ((src_row + win_row >= 0 && src_row + win_row <= ref_height_ && src_col + win_col >= 0
          && src_col + win_col <= ref_width_)
          && (row_pix + win_row >= 0 && row_pix + win_row <= ref_height_ && col_pix + win_col >= 0
              && col_pix + win_col <= ref_width_)) {
        srcs[idx] = src.left_img_.at<uchar>((int) src_row, (int) src_col);
        refs[idx] = ref.left_img_.at<uchar>(row_pix + win_row, col_pix + win_col);
      } else {
        refs[idx] = 255;
        srcs[idx] = 0;
      }

      idx++;
    }
  }

  double src_mean = 0, ref_mean = 0;
  for (int i = 0; i < win_size * win_size; ++i) {
    src_mean += srcs[i];
    ref_mean += refs[i];
  }

  src_mean /= win_size * win_size;
  ref_mean /= win_size * win_size;

  double var = 0.0, var_ref = 0.0, var_src = 0.0;
  for (int i = 0; i < win_size * win_size; ++i) {
    var += (srcs[i] - src_mean) * (refs[i] - ref_mean);

    var_src += (srcs[i] - src_mean) * (srcs[i] - src_mean);
    var_ref += (refs[i] - ref_mean) * (refs[i] - ref_mean);
  }

  double ncc = var / sqrt(var_src * var_ref + 1e-10);

//   if (row_pix == (start_row + end_row) / 2 && col_pix == (start_col + end_col) / 2) {
//
//     cv::Mat a;
//     cv::Mat b;
//     cv::warpPerspective(ref.left_img_, a, H, cv::Size());
//     cv::warpPerspective(ref.left_img_, b, id_to_H[ref_img_->id_][src.id_], cv::Size());
//     cv::imshow("ref", ref.left_img_);
//     cv::imshow("after computed H", a);
//     cv::imshow("real H", b);
//     cv::imshow("src", src.left_img_);
//     cv::waitKey(5);
//   }
  if (var_ref < 1e-5 || var_src < 1e-5) {
//    cout << "hehe" << endl;
    return max_cost;
  }
  return 1.0 - ncc;
}

void Graph::ComputeHomography(const cv::Mat &K_src,
                              const cv::Mat &K_ref,
                              cv::Mat &src_R, cv::Mat &src_T,
                              double &depth_pix,
                              cv::Mat &H,
                              int row, int col,
                              double normal[3]) {
  double arr_n[3][1] = {{normal[0]}, {normal[1]}, {normal[2]}};
  double arr_p[3][1] = {{(double) col}, {(double) row}, {1}};
  cv::Mat n(3, 1, CV_64F, arr_n);
  cv::Mat p(3, 1, CV_64F, &arr_p);
//  cv::Mat cal_H = K_src * (rt.R_j * rt.R_i.t() - rt.R_j * (rt.C_i - rt.C_j) * n.t() / depth_pix) * K_ref.inv();
//  cv::Mat cal_H = K_src * (rt.R  - rt.T * n.t() / depth_pix) * K_ref.inv();
//cout << n << d << p << endl;
//  cv::Mat cal_H = pose_in_dataset ?
//      K_src * (rt.R_i * rt.R_j.inv() + rt.R_i * (rt.C_j - rt.C_i) * n.t() / (n.t() * depth_pix * K_ref.inv() * p))
//          * K_ref.inv() : K_src * (rt.R  + rt.T * n.t() / depth_pix) * K_ref.inv() ;
//  cv::Mat cal_H = K_src * (src_R - src_T * n.t() / depth_pix) * K_ref.inv();
  cv::Mat cal_H = K_src * (src_R  + src_T * n.t() / (n.t() * depth_pix * K_ref.inv() * p)) * K_ref.inv() ;


//  cout << (rt.R_j * (rt.C_i - rt.C_j) * n.t()).size() << (n.t() * depth_pix * K_ref.inv() * p).size() << endl;
//  cout << ((rt.R_j * (rt.C_i - rt.C_j) * n.t())/ (n.t() * depth_pix * K_ref.inv() * p)).size() << endl;
//  cout << "========"<< "R_j is " << endl << rt.R_j <<endl << "R_i is " << rt.R_i <<endl << rt.R_j * rt.R_i.inv() << endl;
  cal_H.copyTo(H);

}

void Graph::ComputeAllNCC(int win_size) {
  cv::Mat H(3, 3, CV_64F), K_ref(3, 3, CV_64F), K_src(3, 3, CV_64F);
  cv::eigen2cv(ref_img_->GetCamera()->K(), K_src);
  cv::eigen2cv(ref_img_->GetCamera()->K(), K_ref);
  double one_minus_ncc;
  double normal[3];
  for (int idx = 0; idx < frames.size(); idx++) {
    out_bound_pix = 0;

    cout << "Pre-Compute all ncc at image" << frames[idx]->id_ << endl;
    for (int row = init_start_row; row < init_end_row; ++row) {

//    cout << "Pre-Compute all ncc at row " << row << endl;
      for (int col = init_start_col; col < init_end_col; ++col) {
        normal[0] = normals.at<double>(row, col, 0);
        normal[1] = normals.at<double>(row, col, 1);
        normal[2] = normals.at<double>(row, col, 2);
        one_minus_ncc = ComputeNCC(*ref_img_,
                                   *frames[idx],
                                   row,
                                   col, win_size,
                                   depth.at<double>(row, col),
                                   normal,
                                   K_src, K_ref);

        if (id_to_NCC.count(ref_img_->id_) <= 0) {
          id_to_NCC.insert(make_pair(ref_img_->id_, map<unsigned int, cv::Mat>()));
        }
        if (id_to_NCC[ref_img_->id_].count(frames[idx]->id_) <= 0) {
          cv::Mat src_m = cv::Mat(ref_height_, ref_width_, CV_64F);
          src_m.at<double>(row, col) = one_minus_ncc;
          id_to_NCC[ref_img_->id_].insert(make_pair(frames[idx]->id_, src_m));
        } else {
          id_to_NCC[ref_img_->id_][frames[idx]->id_].at<double>(row, col) = one_minus_ncc;
        }

      }
    }
    cout << "pixel partly counted is " << out_bound_pix << endl;
  }
}



void Graph::Propagate() {
  int win_size = 7;
  ComputeAllNCC(win_size);
  cv::Mat K_ref(3, 3, CV_64F), K_src(3, 3, CV_64F);
  cv::eigen2cv(ref_img_->GetCamera()->K(), K_src);
  cv::eigen2cv(ref_img_->GetCamera()->K(), K_ref);


  // Compute Back and Forward message Firstly.
  // And the emission probability at specific pixel to get q(Z_l^m)
  // After compute each q(Z_l^m)
  // We use accept-reject sampling to choose some images and find
  // Compute three depth hypothese and choose the smallest one.
  map<unsigned int, map<unsigned int, vector<double>>> id_back_msg;
  map<unsigned int, map<unsigned int, vector<double>>> id_forward_msg;
  if (rotate % 4 == 0) {

    for (int row = start_row; row < end_row; row++) {

      // Compute the backward message first.
      // Then compute each q(Z_l^m)


      cout << "Calculate at row " << row << endl;
      if (!simple) {
        for (int idx = 0; idx < frames.size(); idx++) {
          if (id_back_msg.count(idx) <= 0) {
            id_back_msg.insert(make_pair(idx, map<unsigned, vector<double>>()));
          }

          if (id_back_msg[idx].count(row) <= 0) {

            vector<double> temp_BackMsg;
            double later = 1;
            temp_BackMsg.push_back(later);
            for (int back_col = end_col - 2; back_col >= start_col; --back_col) {
              double em = ComputeEmissionProb(id_to_NCC[ref_img_->id_][frames[idx]->id_].at<double>(row, back_col));

              later = hmm->ComputeBackwardMessage(row,
                                                  back_col,
                                                  em,
                                                  0.5,
                                                  later,
                                                  hmm->transition_prob_(0, 1));
              temp_BackMsg.push_back(later);

            }
              id_back_msg[idx].insert(make_pair(row, temp_BackMsg));
          }
        }
      }
      for (int col = start_col; col < end_col; col++) {
        ComputeSelectionProb(row, col, id_back_msg, id_forward_msg, win_size, K_ref, K_src);

      }

    }
  } else if (rotate % 4 == 1) {

    for (int col = start_col; col < end_col; col++) {
      // Compute the backward message first.
      // Then compute each q(Z_l^m)
      if (!simple) {
        for (int idx = 0; idx < frames.size(); idx++) {
          if (id_back_msg.count(idx) <= 0) {
            id_back_msg.insert(make_pair(idx, map<unsigned, vector<double>>()));
          }
          if (id_back_msg[idx].count(col) <= 0) {

            vector<double> temp_BackMsg;
            double later = 1;
            temp_BackMsg.push_back(later);
            for (int back_row = end_row - 2; back_row >= start_row; --back_row) {
              double em = ComputeEmissionProb(id_to_NCC[ref_img_->id_][frames[idx]->id_].at<double>(back_row, col));

              later = hmm->ComputeBackwardMessage(back_row,
                                                  col,
                                                  em,
                                                  0.5,
                                                  later,
                                                  hmm->transition_prob_(0, 1));
              temp_BackMsg.push_back(later);
            }
            id_back_msg[idx].insert(make_pair(col, temp_BackMsg));

          }
        }
      }
      cout << "Calculate at col " << col << endl;
      for (int row = start_row; row < end_row; row++) {
        ComputeSelectionProb(row, col, id_back_msg, id_forward_msg, win_size, K_ref, K_src);

      }

    }
  } else if (rotate % 4 == 2) {
    for (int row = start_row; row < end_row; row++) {

      // Compute the backward message first.
      // Then compute each q(Z_l^m)
      if (!simple) {
        for (int idx = 0; idx < frames.size(); idx++) {
          if (id_back_msg[idx].count(row) <= 0) {

            vector<double> temp_BackMsg;
            double later = 1;
            temp_BackMsg.push_back(later);
            for (int back_col = end_col + 2; back_col <= start_col; ++back_col) {
              double em = ComputeEmissionProb(id_to_NCC[ref_img_->id_][frames[idx]->id_].at<double>(row, back_col));

              later = hmm->ComputeBackwardMessage(row,
                                                  back_col,
                                                  em,
                                                  0.5,
                                                  later,
                                                  hmm->transition_prob_(0, 1));
              temp_BackMsg.push_back(later);
            }
            id_back_msg[idx].insert(make_pair(row, temp_BackMsg));

          }
        }
      }
      cout << "Calculate at row " << row << endl;
      for (int col = start_col; col > end_col; col--) {
        ComputeSelectionProb(row, col, id_back_msg, id_forward_msg, win_size, K_ref, K_src);

      }

    }
  } else {
    for (int col = start_col; col < end_col; col++) {
      // Compute the backward message first.
      // Then compute each q(Z_l^m)
      if (!simple) {
        for (int idx = 0; idx < frames.size(); idx++) {

          if (id_back_msg[idx].count(col) <= 0) {

            vector<double> temp_BackMsg;
            double later = 1;
            temp_BackMsg.push_back(later);
            for (int back_row = end_row + 2; back_row <= start_row; ++back_row) {
              double em = ComputeEmissionProb(id_to_NCC[ref_img_->id_][frames[idx]->id_].at<double>(back_row, col));

              later = hmm->ComputeBackwardMessage(back_row,
                                                  col,
                                                  em,
                                                  0.5,
                                                  later,
                                                  hmm->transition_prob_(0, 1));
              temp_BackMsg.push_back(later);
            }
            id_back_msg[idx].insert(make_pair(col, temp_BackMsg));

          }
        }
      }
      cout << "Calculate at col " << col << endl;
      for (int row = start_row; row > end_row; row--) {
        ComputeSelectionProb(row, col, id_back_msg, id_forward_msg, win_size, K_ref, K_src);
      }

    }
  }

  ConvertToDepthMap();

}

void Graph::ComputeSelectionProb(int row,
                                 int col,
                                 map<unsigned int, map<unsigned int, vector<double>>> &id_back_msg,
                                 map<unsigned int, map<unsigned int, vector<double>>> &id_forward_msg,
                                 int win_size,
                                 const cv::Mat K_ref, const cv::Mat &K_src) {

  std::random_device rd;
  std::mt19937 gen(rd());
  uniform_int_distribution<int> uni_dist(0, 100);
  uniform_real_distribution<double> rand_dist(depth_min, depth_max);
  vector<double> all_frame_selection_prob;
  // The first one is current depth hypothesis,
  // The second one is the last pixel depth hypothesis,
  // The last one is a random depth.

  double sum_of_ncc_diff_depth[3] = {0.0};
  if (!simple) {
    for (int idx = 0; idx < frames.size(); idx++) {

      if (rotate % 4 == 0) {

        if (id_forward_msg.count(idx) <= 0) {
          id_forward_msg.insert(make_pair(idx, map<unsigned, vector<double>>()));
        }
        if (id_forward_msg[idx].count(row) <= 0) {
          id_forward_msg[idx].insert(make_pair(row, vector<double>()));

        }
        double prev = 0.999;

        if (col - start_col >= 1) {
          double em = ComputeEmissionProb(id_to_NCC[ref_img_->id_][frames[idx]->id_].at<double>(row, col));
          prev = hmm->ComputeForwardMessage(row,
                                            col,
                                            em,
                                            0.5,
                                            id_forward_msg[idx][row].at(col - start_col - 1),
                                            hmm->transition_prob_(0, 1));
        }
        id_forward_msg[idx][row].push_back(prev);


        const double zn0 = (1.0 - prev) * (1.0f - id_back_msg[idx][row][end_col - 1 - col]);
        const double zn1 = prev * id_back_msg[idx][row][end_col - 1 - col];
        double q_l_m = zn1 / (zn0 + zn1);

        all_frame_selection_prob.push_back(q_l_m);

      } else if (rotate % 4 == 1) {

        if (id_forward_msg.count(idx) <= 0) {
          id_forward_msg.insert(make_pair(idx, map<unsigned, vector<double>>()));
        }
        if (id_forward_msg[idx].count(col) <= 0) {
          id_forward_msg[idx].insert(make_pair(col, vector<double>()));

        }
        double prev = 0.999;
        if (row - start_row >= 1) {
          double em = ComputeEmissionProb(id_to_NCC[ref_img_->id_][frames[idx]->id_].at<double>(row, col));
          prev = hmm->ComputeForwardMessage(row,
                                            col,
                                            em,
                                            0.5,
                                            id_forward_msg[idx][col].at(row - start_row - 1),
                                            hmm->transition_prob_(0, 1));
        }
        id_forward_msg[idx][col].push_back(prev);

        const double zn0 = (1.0 - prev) * (1.0f - id_back_msg[idx][col][end_row - 1 - row]);
        const double zn1 = prev * id_back_msg[idx][col][end_row - 1 - row];
        double q_l_m = zn1 / (zn0 + zn1);

        all_frame_selection_prob.push_back(q_l_m);
      } else if (rotate % 4 == 2) {

        if (id_forward_msg.count(idx) <= 0) {
          id_forward_msg.insert(make_pair(idx, map<unsigned, vector<double>>()));
        }
        if (id_forward_msg[idx].count(row) <= 0) {
          id_forward_msg[idx].insert(make_pair(row, vector<double>()));

        }
        double prev = 0.999;
        if (start_col - col >= 1) {
          double em = ComputeEmissionProb(id_to_NCC[ref_img_->id_][frames[idx]->id_].at<double>(row, col));
          prev = hmm->ComputeForwardMessage(row,
                                            col,
                                            em,
                                            0.5,
                                            id_forward_msg[idx][row].at(start_col - col - 1),
                                            hmm->transition_prob_(0, 1));
        }
        id_forward_msg[idx][row].push_back(prev);


        const double zn0 = (1.0 - prev) * (1.0f - id_back_msg[idx][row][start_col - col]);
        const double zn1 = prev * id_back_msg[idx][row][start_col - col];
        double q_l_m = zn1 / (zn0 + zn1);

        all_frame_selection_prob.push_back(q_l_m);
      } else {

        if (id_forward_msg.count(idx) <= 0) {
          id_forward_msg.insert(make_pair(idx, map<unsigned, vector<double>>()));
        }
        if (id_forward_msg[idx].count(col) <= 0) {
          id_forward_msg[idx].insert(make_pair(col, vector<double>()));

        }
        double prev = 0.999;
        if (start_row - row >= 1) {
          double em = ComputeEmissionProb(id_to_NCC[ref_img_->id_][frames[idx]->id_].at<double>(row, col));
          prev = hmm->ComputeForwardMessage(row,
                                            col,
                                            em,
                                            0.5,
                                            id_forward_msg[idx][col].at(start_row - row - 1),
                                            hmm->transition_prob_(0, 1));
        }
        id_forward_msg[idx][col].push_back(prev);

        const double zn0 = (1.0 - prev) * (1.0f - id_back_msg[idx][col][start_row - row]);
        const double zn1 = prev * id_back_msg[idx][col][start_row - row];
        double q_l_m = zn1 / (zn0 + zn1);

        all_frame_selection_prob.push_back(q_l_m);
      }

    }
    int count = 0;
    double min_prob = ComputeEmissionProb(1 - 0.2);
    for (double prob: all_frame_selection_prob) {
      if (prob > min_prob){
        count++;
      }
    }
    if (count <= 2){
      mask.at<double>(row, col) = 1;
    } else {
      mask.at<double>(row, col) = 0;
    }

    // Create a new distribution.
    // And then select the new subset to calculate the sum of ncc.
    Sampling(all_frame_selection_prob);
  }




  int row_cal_depth = row, col_cal_depth = col;
  switch (rotate % 4) {
    case 0:col_cal_depth = col_cal_depth > start_col ? col_cal_depth - 1 : col_cal_depth;
      break;
    case 1:row_cal_depth = row_cal_depth > 0 ? row_cal_depth - 1 : row_cal_depth;
      break;
    case 2:col_cal_depth = col_cal_depth < start_col ? col_cal_depth + 1 : col_cal_depth;
      break;
    case 3:row_cal_depth = row_cal_depth < start_row ? row_cal_depth + 1 : row_cal_depth;
  }

  double prev_normal[3], perturb_cur_normal[3], perturb_depth, prev_depth;
  perturb_depth = PerturbDepth(depth.at<double>(row, col), gen);
  prev_depth = depth.at<double>(row_cal_depth, col_cal_depth);
  prev_normal[0] = normals.at<double>(row_cal_depth, col_cal_depth, 0);
  prev_normal[1] = normals.at<double>(row_cal_depth, col_cal_depth, 1);
  prev_normal[2] = normals.at<double>(row_cal_depth, col_cal_depth, 2);
  perturb_cur_normal[0] = normals.at<double>(row_cal_depth, col_cal_depth, 0);
  perturb_cur_normal[1] = normals.at<double>(row_cal_depth, col_cal_depth, 1);
  perturb_cur_normal[2] = normals.at<double>(row_cal_depth, col_cal_depth, 2);
  PerturbNormal(perturb_cur_normal);

  for (int sample_idx = 0; sample_idx < 15; ++sample_idx) {
    double random = uni_dist(gen) * 0.01;

    for (int idx = 0; idx < frames.size(); idx++) {
      double prob;

      if (simple) {
        prob = 1.0;
      } else {
        prob = all_frame_selection_prob[idx];
      }


      // If accept this frame, then sum the ncc value at different depth.
      if (prob > random) {
        sum_of_ncc_diff_depth[0] += id_to_NCC[ref_img_->id_][frames[idx]->id_].at<double>(row, col);
        sum_of_ncc_diff_depth[1] +=
            ComputeNCC(*ref_img_, *frames[idx], row, col, win_size, prev_depth, prev_normal, K_ref, K_src);
        sum_of_ncc_diff_depth[2] +=
            ComputeNCC(*ref_img_, *frames[idx], row, col, win_size, perturb_depth, perturb_cur_normal, K_ref, K_src);
        if (!simple) {
          break;
        }
      }
    }
    if (simple) {
      break;
    }
  }
  double minimum = 9999;
  int minimum_idx = 0;
  for (int idx = 0; idx < 3; ++idx) {
    if (minimum > sum_of_ncc_diff_depth[idx]) {
      minimum_idx = idx;
      minimum = sum_of_ncc_diff_depth[idx];
    }
  }
  // Update depth
//  cout << "Update depth at " << row << " x " << col << endl;
  switch (minimum_idx) {
    case 1:
      if (col > 0) {
//        cout << "Choose prev depth" << endl;;
        depth.at<double>(row, col) = prev_depth;
        normals.at<double>(row, col, 0) = prev_normal[0];
        normals.at<double>(row, col, 1) = prev_normal[1];
        normals.at<double>(row, col, 2) = prev_normal[2];
      }
      break;
    case 2:depth.at<double>(row, col) = perturb_depth;
      normals.at<double>(row, col, 0) = perturb_cur_normal[0];
      normals.at<double>(row, col, 1) = perturb_cur_normal[1];
      normals.at<double>(row, col, 2) = perturb_cur_normal[2];
//      cout << "Choose random depth" << endl;
      break;
    default:break;

  };
}

void Graph::Sampling(vector<double> &all_prob) {
  double sum = 0.0;

  for (double prob: all_prob) {
    sum += prob;
  }

  double cum_prob = 0.0;
  for (double & idx : all_prob) {
    const double prob = idx  / sum;
    cum_prob += prob;
    idx = cum_prob;
  }


//  const float inv_prob_sum = 1.0f / prob_sum;
//
//  float cum_prob = 0.0f;
//  for (int i = 0; i < num_probs; ++i) {
//    const float prob = probs[i] * inv_prob_sum;
//    cum_prob += prob;
//    probs[i] = cum_prob;
//  }
}

void Graph::ConvertToDepthMap() {
  double min, max;
  cv::Scalar_<double> mean;
  cv::Mat map(ref_height_, ref_width_, CV_64F);
  depth.copyTo(map);
  cv::minMaxLoc(map, &min, &max);
  map /= max;
  cv::imshow("mask1", map);

  map *= 255;

  for (int row = 0; row < ref_height_; ++row) {
    for (int col = 0; col < ref_width_; ++col) {
      if(mask.at<double>(row, col) == 1){
        map.at<double>(row, col) = 0;
      }
    }
  }

  cv::imshow("mask", mask);
  cv::waitKey(1);


  // map.convertTo(map, CV_16U);



  stringstream ss, output;
  ss << "depth-d" << depth_max << "-" << depth_min << "-r" << rotate << "-i" << iterations
     << (pose_in_dataset ? "-pd" : "-pc") << (simple ? "-nohmm" : "-hmm") << "-" << ref_img_->id_ << ".jpg";
  output << "depth-d" << depth_max << "-" << depth_min << "-r" << rotate << "-i" << iterations
             << (pose_in_dataset ? "-pd" : "-pc") << (simple ? "-nohmm" : "-hmm") << "-" << ref_img_->id_ << ".txt";
  iterations++;
  cv::imwrite(ss.str(), map);

  ofstream os;
  os.open(output.str());
  for (int row = 0; row < ref_height_; ++row) {
    for (int col = 0; col < ref_width_; ++col) {
      if(mask.at<double>(row, col) == 1){
        os << 0 << " ";
      } else {
        os << to_string(depth.at<double>(row, col)) << " ";
      }
    }
    os << "\n";
  }
  os.close();

}

void Graph::Rotate() {
  rotate++;
  switch (rotate % 4) {
    case 0:start_row = init_start_row;
      start_col = init_start_col;
      end_row = init_end_row;
      end_col = init_end_col;
      break;
    case 1:start_row = init_start_row;
      start_col = init_start_col;
      end_row = init_end_row;
      end_col = init_end_col;
      break;
    case 2:start_row = init_start_row;
      end_row = init_end_row;
      start_col = init_end_col - 1;
      end_col = init_start_col - 1;
      break;
    case 3:start_row = init_end_row - 1;
      end_row = init_start_row - 1;
      start_col = init_start_col;
      end_col = init_end_col;
      break;
    default:break;
  }
}

double ComputeNCCCostNormFactor(
    const double ncc_sigma) {
  // A = sqrt(2pi)*sigma/2*erf(sqrt(2)/sigma)
  // erf(x) = 2/sqrt(pi) * integral from 0 to x of exp(-t^2) dt
  return 2.0 / (sqrt(2.0 * 3.1415926) * 0.6 *
      erf(2.0 / (0.6 * 1.414213562)));
}

double Graph::ComputeEmissionProb(double ncc) {

  return exp(ncc * ncc * (-0.5 / (0.6 * 0.6))) * ncc_norm_factor_ ;

}

}