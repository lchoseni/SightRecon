#include <opencv2/core.hpp>
#include <time.h>
#include <math.h>
#include <random>
#include <iostream>
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
//  start_row = 500, end_row = 600, start_col = 500, end_col = 600;
  //  start_row = 500, end_row = 1500, start_col = 500, end_col = 1500;
// start_row = 500, end_row = 1500, start_col = 2200, end_col = 2500;
  start_row = 0, end_row = ref_height_, start_col = 0, end_col = ref_width_;
  cout << end_row << " x " << end_col << endl;

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
    if (matches[i].distance <= max(2 * min_dist, 5.0)) {
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

  if (pts1.size() < 8 || pts2.size() < 8) {
    return false;
  }

  cv::Mat K1, K2;

  cv::eigen2cv(frame1->GetCamera()->K(), K1);
  cv::eigen2cv(frame2->GetCamera()->K(), K2);
  cv::Mat ess = cv::findEssentialMat(pts1, pts2, K1);
  if (id_to_H.count(ref_img_->id_) <= 0){
    id_to_H.insert(make_pair(ref_img_->id_, map<unsigned int, cv::Mat>()));
  }
  if (id_to_H[ref_img_->id_].count(frame2->id_) <= 0){
    id_to_H[ref_img_->id_].insert(make_pair(frame2->id_, cv::Mat()));
  }

  id_to_H[ref_img_->id_][frame2->id_] = cv::findHomography(pts1, pts2);
  cout << "H is " << id_to_H[ref_img_->id_][frame2->id_] << endl;
  cv::Mat R, t;
  if (cv::recoverPose(ess, pts1, pts2, R, t) <= 0) {
    return false;
  }
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
    if (id_to_RTs_.count(ref_img_->id_) <= 0) {
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
//  cout << rela_rt.R << rela_rt.T << endl;
  ComputeHomography(K_src,
                    K_ref,
                    rela_rt.R,
                    rela_rt.T,
                    depth,
                    H, row_pix, col_pix);

//  if (src.id_ == 4) {
//    cv::imshow("1", ref.left_img_);
//    cv::Mat a;
//    cv::Mat b;
//    cv::warpPerspective(ref.left_img_, a, H, cv::Size());
//    cv::warpPerspective(ref.left_img_, b, id_to_H[ref_img_->id_][src.id_], cv::Size());
//    cv::imshow("2", a);
//    cv::imshow("real H", b);
//    cv::imshow("3", src.left_img_);
//    cv::waitKey(0);
//  }

  // Compute the image coordinate after homography warping.
  // If it's outside the boarder, ignore it and return the minimum NCC.
  double ref_coor_homo[3][1] = {{(double) row_pix}, {(double) col_pix}, {1.0}};
  cv::Mat ref_coor = cv::Mat(3, 1, CV_64F);

  ref_coor.at<double>(0, 0) = (double) row_pix;
  ref_coor.at<double>(0, 1) = (double) col_pix;
  ref_coor.at<double>(0, 2) = 1.0;
  cv::Mat src_coor = H * ref_coor;
  if (row_pix == 100 && col_pix == 100){
    cv::Mat sd = id_to_H[ref_img_->id_][src.id_] * ref_coor;
    sd /= sd.at<double>(0, 2);
    cout<< ref_coor.t() << " " << sd.t() << " " << (src_coor / src_coor.at<double>(0, 2)).t() << " " << (rela_rt.R * ref_coor).t() << endl;
//    cv::imshow("1", ref.left_img_);
//    cv::Mat a;
//    cv::Mat b;
//    cv::warpPerspective(ref.left_img_, a, H, cv::Size());
//    cv::warpPerspective(ref.left_img_, b, id_to_H[ref_img_->id_][src.id_], cv::Size());
//    cv::imshow("2", a);
//    cv::imshow("real H", b);
//    cv::imshow("3", src.left_img_);
//    cv::waitKey(0);
  }
  double src_row = src_coor.at<double>(0, 0) / src_coor.at<double>(0, 2);
  double src_col = src_coor.at<double>(0, 1) / src_coor.at<double>(0, 2);
  double ref_row = ref_coor.at<double>(0, 0) / ref_coor.at<double>(0, 2);
  double ref_col = ref_coor.at<double>(0, 1) / ref_coor.at<double>(0, 2);

  // Calculate the ncc value of the matched patch.
  int half_win = win_size / 2;
  int count = 0;
  double sum_ref = 0.0, sum_src = 0.0, sum_sqre_ref = 0.0, sum_sqre_src = 0.0;
  for (int win_row = -half_win; win_row <= half_win; ++win_row) {
    for (int win_col = -half_win; win_col <= half_win; ++win_col) {
      if ((src_row + win_row >= 0 && src_row + win_row <= ref_width_ && src_col + win_col >= 0
          && src_col + win_col <= ref_height_)
          && (ref_row + win_row >= 0 && ref_row + win_row <= ref_width_ && ref_col + win_col >= 0
              && ref_col + win_col <= ref_height_)) {

        count++;

        sum_src += src.left_img_.at<uchar>(src_row + win_row, src_col + win_col);
//        sum_sqre_src += pow(src.left_img_.at<uchar>(src_row + win_row, src_col + win_col), 2);

        sum_ref += ref.left_img_.at<uchar>(ref_row + win_row, ref_col + win_col);
//        sum_sqre_ref += pow(ref.left_img_.at<uchar>(ref_row + win_row, ref_col + win_col), 2);
      }
    }
  }

  double ref_mean = sum_ref / count;
  double src_mean = sum_src / count;
  double var = 0.0, var_ref = 0.0, var_src = 0.0;
  for (int win_row = -half_win; win_row <= half_win; ++win_row) {
    for (int win_col = -half_win; win_col <= half_win; ++win_col) {
      if ((src_row + win_row >= 0 && src_row + win_row <= ref_width_ && src_col + win_col >= 0
          && src_col + win_col <= ref_height_)
          && (ref_row + win_row >= 0 && ref_row + win_row <= ref_width_ && ref_col + win_col >= 0
              && ref_col + win_col <= ref_height_)) {
        var += (src.left_img_.at<uchar>(src_row + win_row, src_col + win_col) - src_mean)
            * (ref.left_img_.at<uchar>(ref_row + win_row, ref_col + win_col) - ref_mean);

        var_src += pow(src.left_img_.at<uchar>(src_row + win_row, src_col + win_col) - src_mean, 2);
        var_ref += pow(ref.left_img_.at<uchar>(ref_row + win_row, ref_col + win_col) - ref_mean, 2);
      }
    }
  }

  double ncc = var / sqrt(var_src * var_ref);
//  if (col_pix > 400) {
//    cout << "col " << col_pix << " ncc " << ncc << "var "<< var << " " << var_src << " " << var_ref << endl;
//  }
  if (var_ref < 1e-5 || var_src < 1e-5) {
    return max_cost;
  }
  return max(0.0, min(max_cost, 1.0 - ncc));
}

void Graph::ComputeHomography(const cv::Mat &K_src,
                              const cv::Mat &K_ref,
                              const cv::Mat &R,
                              const cv::Mat &T,
                              double &depth,
                              cv::Mat &H,
                              int row, int col) {
  double arr_n[3][1] = {{0}, {0}, {1.0}};
  double arr_d[3][1] = {{1}, {1}, {depth}};
  double arr_p[3][1] = {{(double) col}, {(double) row}, {1}};
  cv::Mat n(3, 1, CV_64F, &arr_n);
  cv::Mat d(3, 1, CV_64F, &arr_d);
  cv::Mat p(3, 1, CV_64F, &arr_p);
//  cv::Mat cal_H = K_src * (R - T * n.t() / (n.t() * depth * K_src.inv() * p)) * K_ref.inv();
  cv::Mat cal_H = K_src * (R - T * n.t() / depth) * K_ref.inv();

  
//  cout << n << " " << d << " " << n.t() *d << endl;
  cal_H.copyTo(H);

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

  // Compute Back and Forward message Firstly.
  // And the emission probability at specific pixel to get q(Z_l^m)
  // After compute each q(Z_l^m)
  // We use accept-reject sampling to choose some images and find
  // Compute three depth hypothese and choose the smallest one.

  for (int row = start_row; row < end_row; row++) {

    // Compute the backward message first.
    // Then compute each q(Z_l^m)

    map<unsigned int, map<unsigned int, vector<double>>> id_row_back_msg;
    map<unsigned int, map<unsigned int, vector<double>>> id_row_forward_msg;
    cout << "Calculate at " << row << endl;
    for (int col = start_col; col < end_col; ++col) {
      vector<double> all_frame_selection_prob;
      vector<double> *BackMsg;
      // The first one is current depth hypothesis,
      // The second one is the last pixel depth hypothesis,
      // The last one is a random depth.

      double sum_of_ncc_diff_depth[3] = {0.0};
//      for (int idx = 0; idx < frames.size(); idx++) {
//        if (id_row_back_msg.count(idx) <= 0) {
//          id_row_back_msg.insert(make_pair(idx, map<unsigned, vector<double>>()));
//
//        }
//        if (id_row_back_msg[idx].count(row) <= 0) {
//
//          vector<double> temp_BackMsg;
//          double later = 1;
//
//          for (int back_col = end_col - 1; back_col >= start_col; --back_col) {
//            double em = ComputeEmissionProb(id_to_NCC[ref_img_->id_][frames[idx]->id_].at<double>(row, back_col));
//
//            later = hmm->ComputeBackwardMessage(row,
//                                                back_col,
//                                                end_col - 1,
//                                                em,
//                                                0.5,
//                                                later,
//                                                hmm->transition_prob_(0, 1));
//            temp_BackMsg.push_back(later);
//          }
//          id_row_back_msg[idx].insert(make_pair(row, temp_BackMsg));
//          //          for (size_t i = 0; i < temp_BackMsg.size(); i++) {
//          //            cout << temp_BackMsg[i] << ' ';
//          //          }
//          //          cout << endl;
//        }
//        BackMsg = &id_row_back_msg[idx][row];
//
//        if (id_row_forward_msg.count(idx) <= 0) {
//          id_row_forward_msg.insert(make_pair(idx, map<unsigned, vector<double>>()));
//        }
//        if (id_row_back_msg[idx].count(row) <= 0) {
//          vector<double> temp_BackMsg;
//          id_row_forward_msg[idx].insert(make_pair(row, temp_BackMsg));
//
//        }
//        double prev = 0.5;
//        if (col - start_col >= 1) {
//          double em = ComputeEmissionProb(id_to_NCC[ref_img_->id_][frames[idx]->id_].at<double>(row, col));
//          prev = hmm->ComputeForwardMessage(row,
//                                            col,
//                                            end_col,
//                                            em,
//                                            0.5,
//                                            id_row_forward_msg[idx][row].at(col - start_col - 1),
//                                            hmm->transition_prob_(0, 1));
//        }
//        id_row_forward_msg[idx][row].push_back(prev);
//
//        const float zn0 = (1.0f - prev) * (1.0f - (*BackMsg)[end_col - 1 - col]);
//        const float zn1 = prev * (*BackMsg)[end_col - 1 - col];
//        double q_l_m = zn1 / (zn0 + zn1);
//
//        all_frame_selection_prob.push_back(q_l_m);
//
//      }
//
//      // Create a new distribution.
//      // And then select the new subset to calculate the sum of ncc.
//      Sampling(all_frame_selection_prob);
//      //      for (size_t i = 0; i < all_frame_selection_prob.size(); i++) {
//      //        cout << all_frame_selection_prob[i] << ' ';
//      //      }
//      //      cout << endl;
//
      double random_depth = rand_dist(gen);
//      for (int sample_idx = 0; sample_idx < 15; ++sample_idx) {
        double random = uni_dist(gen) * 0.01;
//
        for (int idx = 0; idx < frames.size(); idx++) {
//          const float prob = all_frame_selection_prob[idx];

          // If accept this frame, then sum the ncc value at different depth.
          // curr
//          if (prob > random) {
            sum_of_ncc_diff_depth[0] += id_to_NCC[ref_img_->id_][frames[idx]->id_].at<double>(row, col);
            if (col > 0) {

              sum_of_ncc_diff_depth[1] +=
                  ComputeNCC(*ref_img_, *frames[idx], row, col, win_size, depth.at<double>(row, col - 1));

            } else {
              // prev
              sum_of_ncc_diff_depth[1] += id_to_NCC[ref_img_->id_][frames[idx]->id_].at<double>(row, col);

            }

            sum_of_ncc_diff_depth[2] += ComputeNCC(*ref_img_, *frames[idx], row, col, win_size, random_depth);

//          }
//        }
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
//      cout << "Update depth at " << row << " x " << col << endl;
      switch (minimum_idx) {
        case 1:
          if (col > 0) {
//            cout << "Choose prev depth" << endl;;
            depth.at<double>(row, col) = depth.at<double>(row, col - 1);
          }
          break;
        case 2:depth.at<double>(row, col) = random_depth;
//          cout << "Choose random depth" << endl;
          break;
        default:break;

      };

    }

  }
  cv::imwrite("depth2.jpg", depth);
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
  cv::Mat map(ref_height_, ref_width_, CV_64F);
  cv::Mat mat;
  cv::minMaxLoc(depth, &min, &max);
  map = depth / max;
  map *= 255;

  stringstream ss;
  ss << "depth-" << rotate << ".jpg";
  cv::imwrite(ss.str(), map);
  cv::imwrite("depth1.jpg", depth);

}

void Graph::Rotate() {
  depth = depth.t();
  int temp = ref_width_;
  ref_width_ = ref_height_;
  ref_height_ = temp;
  int temp_row = start_row, temp_end_row = end_row;
  start_row = start_col;
  end_row = end_col;
  start_col = temp_row;
  end_col = temp_end_row;
  auto it = id_to_NCC.begin();
  while (it != id_to_NCC.end()) {
    auto second_it = it->second.begin();
    while (second_it != it->second.end()) {
      cv::rotate(second_it->second, second_it->second, cv::ROTATE_90_CLOCKWISE);
      second_it++;
    }
    it++;
  }

  for (const shared_ptr<Frame> &frame: frames) {
    frame->left_img_ = frame->left_img_.t();
  }
  rotate++;
}

double Graph::ComputeEmissionProb(double ncc) {

  return exp(ncc * ncc * (-0.5 / (0.6 * 0.6))) * 0.6;

}

}