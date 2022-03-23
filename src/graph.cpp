#include <opencv2/core.hpp>
#include <time.h>
#include <math.h>
#include <random>
#include <unistd.h>

#include "SSLAM/graph.h"
#include "SSLAM/common_include.h"
#include "SSLAM/camera.h"
#include "SSLAM/frame.h"
#include "SSLAM/dataset.h"
#include "SSLAM/utils/utils.h"

namespace sslam {

Graph::Graph(Dataset *dataset, shared_ptr<Frame> ref_img) : dataset_(dataset), ref_img_(ref_img) {
  ref_height_ = ref_img_->left_img_.rows;
  ref_width_ = ref_img_->left_img_.cols;
  depth = cv::Mat(ref_height_, ref_width_, CV_64F);
  depth_max = 500.0;
  depth_min = 0.0;
  Eigen::Matrix<double, 2, 2> tran_prob;
  tran_prob << 0.999, 0.001, 0.001, 0.999;
  hmm = new Hmm(tran_prob);
  start_row = 0, end_row = ref_height_, start_col = 0, end_col = ref_width_;

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
    }
  }
}

bool Graph::ComputeRAndTOfTwoImgs(shared_ptr<Frame> &frame1, shared_ptr<Frame> &frame2, cv::Mat &R_, cv::Mat &t_) {

  if (frame1 == nullptr || frame2 == nullptr) {
    return false;
  }

  front_end_->DetectFeatures(frame1);
  front_end_->DetectFeatures(frame2);

  cv::Mat descriptors_1, descriptors_2;
  cv::Ptr<cv::FeatureDetector> detector = cv::ORB::create();
  cv::Ptr<cv::DescriptorExtractor> descriptor = cv::ORB::create();
  cv::Ptr<cv::DescriptorMatcher> matcher = cv::DescriptorMatcher::create("BruteForce-Hamming");

  detector->detect(frame1->left_img_, frame1->left_key_points_);
  detector->detect(frame2->left_img_, frame2->left_key_points_);

  descriptor->compute(frame1->left_img_, frame1->left_key_points_, descriptors_1);
  descriptor->compute(frame2->left_img_, frame2->left_key_points_, descriptors_2);

  vector<cv::DMatch> matches;
  matcher->match(descriptors_1, descriptors_2, matches);

  auto min_max = minmax_element(matches.begin(), matches.end(),
                                [](const cv::DMatch &m1, const cv::DMatch &m2) { return m1.distance < m2.distance; });

  double min_dist = min_max.first->distance;
  double max_dist = min_max.second->distance;

  std::vector<cv::DMatch> good_matches;
  for (int i = 0; i < descriptors_1.rows; i++) {
    if (matches[i].distance <= max(2 * min_dist, 1.0)) {
      good_matches.push_back(matches[i]);
    }
  }

//   cv::Mat img_match;
//   cv::Mat img_goodmatch;
//   drawMatches(frame1->left_img_, frame1->left_key_points_, frame2->left_img_, frame2->left_key_points_, matches, img_match);
//   drawMatches(frame1->left_img_, frame1->left_key_points_, frame2->left_img_, frame2->left_key_points_, good_matches, img_goodmatch);
//   imshow("all matches", img_match);
//   imshow("good matches", img_goodmatch);

  std::vector<cv::Point2f> pts1, pts2;
  std::vector<cv::KeyPoint> ky_pts1, ky_pts2;
  for (auto &match: good_matches) {
    ky_pts1.push_back(frame1->left_key_points_[match.queryIdx]);
    ky_pts2.push_back(frame2->left_key_points_[match.trainIdx]);
  }

  cv::KeyPoint::convert(ky_pts1, pts1);
  cv::KeyPoint::convert(ky_pts2, pts2);
  // std::cout << pts1.size() << " " << pts2.size() << std::endl;

  if (pts1.size() < 8 || pts2.size() < 8) {
    return false;
  }

  cv::Mat K1, K2;

  cv::eigen2cv(frame1->GetCamera()->K(), K1);
  cv::eigen2cv(frame2->GetCamera()->K(), K2);

  cv::Mat ess = cv::findEssentialMat(pts1, pts2, K1);
  // cout << "Essential matrix is " << ess << endl;

  cv::Mat R, t;
  if (cv::recoverPose(ess, pts1, pts2, R, t) <= 0) {
    return false;
  }
//  cout << "R is " << R << ", and T is " << t << endl;
  R_ = R;
  t_ = t;
  return true;
}

void Graph::ComputeAllRAndT() {

  shared_ptr<Frame> frame = nullptr;
  while ((frame = dataset_->GetNextFrame()) != nullptr) {
    if (ref_img_ == nullptr) {
      ref_img_ = frame;
      continue;
    }
    cv::Mat R, t;
    if (!ComputeRAndTOfTwoImgs(ref_img_, frame, R, t)) {
      cout << "Can not get R and T from " << frame->id_ << endl;
      continue;
    }
    frames.push_back(frame);
    cout << R << t << endl;
    RELA_RT rela_rt;
    rela_rt.R = R;
    rela_rt.T = t;
    if (id_to_RTs_.count(ref_img_->id_) == 0) {
      map<unsigned int, RELA_RT> map;
      map.insert(make_pair(frame->id_, rela_rt));
      id_to_RTs_.insert(make_pair(ref_img_->id_, map));
    } else {
      id_to_RTs_[ref_img_->id_].insert(make_pair(frame->id_, rela_rt));
    }
  }
}

double Graph::ComputeNCC(Frame &ref, Frame &src, int row_pix, int col_pix, int win_size, double depth) {
  double max_cost = 2.0;
  RELA_RT rela_rt = id_to_RTs_[ref.id_][src.id_];
  cv::Mat H(3, 3, CV_64F), K_ref(3, 3, CV_64F), K_src(3, 3, CV_64F);
  cv::eigen2cv(src.GetCamera()->K(), K_src);
  cv::eigen2cv(ref.GetCamera()->K(), K_ref);


//  cout << "Start compute Homography matrix." << endl;
  ComputeHomography(K_src,
                    K_ref,
                    rela_rt.R,
                    rela_rt.T,
                    depth,
                    H);
  // Compute the image coordinate after homography warping.
  // If it's outside the boarder, ignore it and return the minimum NCC.
  double ref_coor_homo[3][1] = {{(double)row_pix}, {(double)col_pix}, {1.0}};
  cv::Mat ref_coor = cv::Mat(3, 1, CV_64F);
//  cout << ref_coor<< " " << K_ref << " " << K_src<<endl;

  ref_coor.at<double>(0, 0) = (double ) row_pix;
  ref_coor.at<double>(0, 1) = (double ) col_pix;
  ref_coor.at<double>(0, 2) = 1.0;
  cv::Mat src_coor = H * ref_coor;
  double src_row = src_coor.at<double>(0, 0) / src_coor.at<double>(0, 2);
  double src_col = src_coor.at<double>(0, 1) / src_coor.at<double>(0, 2);
  double ref_row = ref_coor.at<double>(0, 0) / ref_coor.at<double>(0, 2);
  double ref_col = ref_coor.at<double>(0, 1) / ref_coor.at<double>(0, 2);

  // Calculate the ncc value of the matched patch.
  int half_win = win_size / 2;
  double sum_ref = 0.0, sum_src = 0.0, sum_sqre_ref = 0.0, sum_sqre_src = 0.0;
  for (int win_row = -half_win; win_row <= half_win; ++win_row) {
    for (int win_col = -half_win; win_col <= half_win; ++win_col) {
      if ((src_row + win_row >= 0 && src_row + win_row <= ref_width_ && src_col + win_col >= 0
          && src_col + win_col <= ref_height_)
          && (ref_row + win_row >= 0 && ref_row + win_row <= ref_width_ && ref_col + win_col >= 0
              && ref_col + win_col <= ref_height_)) {
        sum_src += src.left_img_.at<uchar>(src_col + win_col, src_row + win_row);
        sum_sqre_src += pow(src.left_img_.at<uchar>(src_col + win_col, src_row + win_row), 2);

        sum_ref += ref.left_img_.at<uchar>(ref_col + win_col, ref_row + win_row);
        sum_sqre_ref += pow(ref.left_img_.at<uchar>(ref_col + win_col, ref_row + win_row), 2);
      }
    }
  }
  double ncc = sum_src * sum_ref / (sum_sqre_ref * sum_sqre_src);

  return max(0.0, min(max_cost, 1.0 - ncc));
}

void Graph::ComputeHomography(const cv::Mat &K_src,
                              const cv::Mat &K_ref,
                              const cv::Mat &R,
                              const cv::Mat &T,
                              double &depth,
                              cv::Mat &H) {
  double arr_n[3][1] = {{0}, {0}, {1.0}};
  cv::Mat n(3, 1, CV_64F, &arr_n);
  cv::Mat cal_H = K_src * (R - T * n.t() / depth) * K_ref.inv();

  H.at<double>(0, 0) = cal_H.at<double>(0, 0);
  H.at<double>(0, 1) = cal_H.at<double>(0, 1);
  H.at<double>(0, 2) = cal_H.at<double>(0, 2);

  H.at<double>(1, 0) = cal_H.at<double>(1, 0);
  H.at<double>(1, 1) = cal_H.at<double>(1, 1);
  H.at<double>(1, 2) = cal_H.at<double>(1, 2);

  H.at<double>(2, 0) = cal_H.at<double>(2, 0);
  H.at<double>(2, 1) = cal_H.at<double>(2, 1);
  H.at<double>(2, 2) = cal_H.at<double>(2, 2);
}

void Graph::ComputeAllNCC(int win_size) {
  for (int idx = 0; idx < frames.size(); idx++) {

    cout << "Compute all ncc at image" << frames[idx]->id_ << endl;
    for (int row = start_row; row < end_row; ++row) {
      for (int col = start_col; col < end_col; ++col) {
        double one_minus_ncc = ComputeNCC(*ref_img_,
                                          *frames[idx],
                                          row,
                                          col, win_size,
                                          depth.at<double>(row, col));
        if (one_minus_ncc < 0.5){
          cout << "Good!!!!!!!!!!!!!!!!!!!!!!!" << endl;
        }
//        cout << "1 - ncc is " << one_minus_ncc << endl;
        if (id_to_NCC.count(ref_img_->id_) <= 0) {
          id_to_NCC.insert(make_pair(ref_img_->id_, map<unsigned int, cv::Mat>()));
          cv::Mat src_m = cv::Mat(ref_height_, ref_width_, CV_64F);
          src_m.at<double>(row, col) = one_minus_ncc;
          id_to_NCC[ref_img_->id_].insert(make_pair(frames[idx]->id_, src_m));
        } else {
          if (id_to_NCC[ref_img_->id_].count(frames[idx]->id_) <= 0) {
            cv::Mat src_m = cv::Mat(ref_height_, ref_width_, CV_64F);
            src_m.at<double>(row, col) = one_minus_ncc;
            id_to_NCC[ref_img_->id_].insert(make_pair(frames[idx]->id_, src_m));
          } else {
            id_to_NCC[ref_img_->id_][frames[idx]->id_].at<double>(row, col) = one_minus_ncc;
          }
        }
      }
    }
  }
}

void Graph::Propagate() {
  int win_size = 5;
  ComputeAllNCC(win_size);
  std::random_device rd;
  std::mt19937 gen(rd());
  uniform_int_distribution<int> uni_dist(0, 100);
  uniform_real_distribution<double> rand_dist(depth_min, depth_max);
  map<unsigned int, map<unsigned int, vector<double>>> id_row_back_msg;

  // Compute Back and Forward message Firstly.
  // And the emission probability at specific pixel to get q(Z_l^m)
  // After compute each q(Z_l^m)
  // We use accept-reject sampling to choose some images and find
  // Compute three depth hypothese and choose the smallest one.

//  int start_row = 600, end_row = 900, start_col = 300, end_col = 600;
  for (int row = start_row; row < end_row; row++) {

    // Compute the backward message first.
    // Then compute each q(Z_l^m)

    cout << "Calculate at " << row << endl;
    for (int col = start_col; col < end_col; ++col) {
//      cout << "Calculate at " << row << ", " << col << endl;

      vector<double> all_frame_selection_prob;
      double later = 1;
      vector<double> *BackMsg;
      // The first one is current depth hypothesis,
      // The second one is the last pixel depth hypothesis,
      // The last one is a random depth.

      double sum_of_ncc_diff_depth[3] = {0.0};
      for (int idx = 0; idx < frames.size(); idx++) {
        if (id_row_back_msg.count(idx) <= 0) {
          id_row_back_msg.insert(make_pair(idx, map<unsigned, vector<double>>()));

        }
        if (id_row_back_msg[idx].count(row) <= 0) {

          vector<double> temp_BackMsg;

          for (int back_col = end_col - 1; back_col >= start_col; --back_col) {
//          double em = ComputeEmissionProb(id_to_NCC[ref_img_->id_][frames[idx]->id_].at<double>(row, back_col));

            double em = ComputeEmissionProb(id_to_NCC[ref_img_->id_][frames[idx]->id_].at<double>(row, back_col));
            // cout << "em of image " <<  frames[idx]->id_ << " is " << em << endl;

            later = hmm->ComputeBackwardMessage(row,
                                                back_col,
                                                end_col - 1,
                                                em,
                                                0.5,
                                                later,
                                                hmm->transition_prob_(0, 1));
            temp_BackMsg.push_back(later);
          }
          id_row_back_msg[idx].insert(make_pair(row, temp_BackMsg));

        }
        BackMsg = &id_row_back_msg[idx][row];

        later = 0.5;
        double em = ComputeEmissionProb(id_to_NCC[ref_img_->id_][frames[idx]->id_].at<double>(row, col));
        later = hmm->ComputeForwardMessage(row, col, end_col, em, 0.5, later, hmm->transition_prob_(0, 1));

        double q_l_m = later * (*BackMsg)[end_col - 1 - col];
//        cout << "all prob is " << all_frame_selection_prob[0] << " " << all_frame_selection_prob[1] << endl;

        all_frame_selection_prob.push_back(q_l_m);
        // cout << "all prob is " << all_frame_selection_prob[0] << " " << all_frame_selection_prob[1] << endl;

      }

      // Create a new distribution.
      // And then select the new subset to calculate the sum of ncc.
      Sampling(all_frame_selection_prob);

      double random_depth = rand_dist(gen);
      for (int sample_idx = 0; sample_idx < 15; ++sample_idx) {
        double random = uni_dist(gen) * 0.01;

        for (int idx = 0; idx < frames.size(); idx++) {
          const float prob = all_frame_selection_prob[frames[idx]->id_];

          // If accept this frame, then sum the ncc value at different depth.
          // curr
          if (prob > random) {
            sum_of_ncc_diff_depth[0] += id_to_NCC[ref_img_->id_][frames[idx]->id_].at<double>(row, col);
            if (col > 0) {

              sum_of_ncc_diff_depth[1] +=
                  ComputeNCC(*ref_img_, *frames[idx], row, col, win_size, depth.at<double>(row, col - 1));

            } else {
              // prev
              sum_of_ncc_diff_depth[1] += id_to_NCC[ref_img_->id_][frames[idx]->id_].at<double>(row, col);

            }

            sum_of_ncc_diff_depth[2] += ComputeNCC(*ref_img_, *frames[idx], row, col, win_size, random_depth);

            break;
          }
        }
      }
      double minimum = 9999;
      int minimum_idx = -1;
      for (int idx = 0; idx < 3; ++idx) {
        if (minimum > sum_of_ncc_diff_depth[idx]) {
          minimum_idx = idx;
          minimum = sum_of_ncc_diff_depth[idx];
        }
      }

      // Update depth
      switch (minimum_idx) {
        case 1:
          if (col > 0) {
            depth.at<double>(row, col) = depth.at<double>(row, col - 1);
          }
          break;
        case 2:depth.at<double>(row, col) = random_depth;
          break;
        default:break;

      };

    }

  }
  ConvertToDepthMap();
}

void Graph::Sampling(vector<double> &all_prob) {
  double sum = 0.0;

  for (double prob: all_prob) {
    sum += prob;
  }

  for (int idx = 0; idx < all_prob.size(); ++idx) {
    all_prob[idx] /= sum;
  }
}

void Graph::ConvertToDepthMap() {
  double min, max;
  cv::Mat map;
  cv::minMaxLoc(depth, &min, &max);
  cout << "max and min is " << max << " " << min << endl;
  map = depth / max;
  map *= 255;
  // cout << map;
  cv::imwrite("depth.jpg", map);

}

double Graph::ComputeEmissionProb(double ncc) {

  return exp(ncc * ncc * (-0.5 / (0.6 * 0.6))) * 0.6;

}

}