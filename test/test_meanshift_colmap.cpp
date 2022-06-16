//
// Created by yeren on 2022/5/4.
//

//
// Created by lchoseni on 2022/3/17.
//

#include "SSLAM/graph.h"
#include "SSLAM/dataset.h"
#include "SSLAM/ColmapDataset.h"
#include "SSLAM/front_end.h"
#include "SSLAM/config.h"
#include "SSLAM/frame.h"
#include "SSLAM/hmm.h"
#include "SSLAM/fusion.h"
#include "SSLAM/mean_shift.h"
#include "unistd.h"

int main() {
  sslam::Config::SetConfigFile(sslam::Config::config_file_);

  sslam::ColmapDataset dataset;

  sslam::ColmapDataset::GetCamera(0);
  sslam::ColmapDataset::GetCamera(1);
  sslam::FrontEnd front_end;

  sslam::Graph graph = sslam::Graph(&dataset, dataset.GetNextFrame(0));


  graph.InitialRandomDepth();
  cv::Mat K;
  cv::eigen2cv(graph.ref_img_->cam_->K(), K);
  graph.InitialRandomNormal(K);

  graph.ComputeAllRAndT();

  std::ifstream is;

  // Load sources depth data
  for (shared_ptr<sslam::Frame> &frame: graph.frames) {

    frame->depth = cv::Mat(frame->left_img_.rows, frame->left_img_.cols, CV_64F);
    stringstream ss;
    if (frame->id_ < 10){

      ss << "/home/yeren/self/images/converted_depth_000" << frame->id_  << ".jpg.geometric.bin.txt";
    } else{
      ss << "/home/yeren/self/images/converted_depth_00" << frame->id_  << ".jpg.geometric.bin.txt";
    }
    cout << "Load depth of " << ss.str() << endl;
    if (access(ss.str().c_str(), F_OK) == -1) {
      cout << "Cannot access " << ss.str() <<endl;
      return -1;
    }
    std::ifstream fin(ss.str());
    double temp_val;
    for (int row = 0; row < graph.ref_img_->left_img_.rows; ++row) {
      for (int col = 0; col < graph.ref_img_->left_img_.cols; ++col) {
        fin >> temp_val;
        frame->depth.at<double>(row, col) = temp_val;
      }
    }
    cout << "R is " << (frame->R_c_w.transpose() * graph.ref_img_->R_c_w) / (frame->R_c_w.transpose() * graph.ref_img_->R_c_w)(2, 2) << endl;
    // ref_img_->R_c_w.transpose() * (ref_img_->R_c_w * -ref_img_->C_c_w + frame->R_c_w * frame->C_c_w)
    cout << "T is " << (frame->R_c_w.transpose() * (frame->R_c_w * -frame->C_c_w + graph.ref_img_->R_c_w * graph.ref_img_->C_c_w)) / ((frame->R_c_w.transpose() * (frame->R_c_w * -frame->C_c_w + graph.ref_img_->R_c_w * graph.ref_img_->C_c_w))).norm()  << endl;

    // cout << "T is " << graph.ref_img_->C_c_w - frame->C_c_w << endl;
    fin.close();
  }
  // Load reference depth data.
  graph.ref_img_->depth = cv::Mat(graph.ref_img_->left_img_.rows, graph.ref_img_->left_img_.cols, CV_64F);
  stringstream ss;
  ss << "/home/yeren/self/images/converted_depth_000" << graph.ref_img_->id_ << ".jpg.geometric.bin.txt";
  cout << "Load depth of " << ss.str() << endl;

  std::ifstream fin(ss.str());
  double temp_val;
  for (int row = 0; row < graph.ref_img_->left_img_.rows; ++row) {
    for (int col = 0; col < graph.ref_img_->left_img_.cols; ++col) {
      fin >> temp_val;

      graph.ref_img_->depth.at<double>(row, col) = temp_val;
    }
  }
  fin.close();


  // Start fusing.
  sslam::Fusion fusion;
//  vector<shared_ptr<sslam::Frame>> temp_frames;
//  temp_frames.push_back(graph.frames[0]);
//  // temp_frames.push_back(graph.frames[3]);
//
//  fusion.frames_ = temp_frames;
  fusion.frames_ = vector<sslam::Frame::Ptr>(graph.frames);
  fusion.ref_img_ = graph.ref_img_;

  fusion.fuse();

  Vec3 mean;
  cv::Mat final_depth(graph.ref_img_->left_img_.rows, graph.ref_img_->left_img_.cols, CV_64F);
  for (int row = 0; row < graph.ref_img_->left_img_.rows; ++row) {
    cout << "Mean shift at row " << row << endl;
    for (int col = 0; col < graph.ref_img_->left_img_.cols; ++col) {
      sslam::MeanShift mean_shift(1, 1, &fusion.map_row_map_col_pts[row][col]);
      if (mean_shift.Shift() == -1) {
        continue;
      }
      mean_shift.Label();
      mean = mean_shift.GetMean();

      if (mean(2) == 0){
        final_depth.at<double>(row, col) = graph.ref_img_->depth.at<double>(row, col);
      } else {
        final_depth.at<double>(row, col) = mean(2);
      }
    }
  }

  double max, min;
  cv::minMaxLoc(final_depth, &min, &max);
  final_depth /= max / 1.5;
  final_depth *= 255;
  cv::imwrite("fused_unfiltered_colmap_depth.jpg", final_depth);



  // Filter geometric error pixel by meanshift

  double threshold = 0.01;
  for(int row =0; row < graph.ref_img_->left_img_.rows; row ++){
    for(int col = 0; col < graph.ref_img_->left_img_.cols; col ++){
      vector<vector<double>> filitered_pts = vector<vector<double>>();
      for (vector<double> pt: fusion.map_row_map_col_pts[row][col]){
        if(abs(final_depth.at<double>(row, col) - pt[2]) < threshold){
          filitered_pts.push_back(pt);
        }
      }
      fusion.map_row_map_col_pts[row][col] = filitered_pts;
    }
  }

  for (int row = 0; row < graph.ref_img_->left_img_.rows; ++row) {
    cout << "Mean shift at row " << row << endl;
    for (int col = 0; col < graph.ref_img_->left_img_.cols; ++col) {
      sslam::MeanShift mean_shift(1, 1, &fusion.map_row_map_col_pts[row][col]);
      if (mean_shift.Shift() == -1) {
        continue;
      }
      mean_shift.Label();
      mean = mean_shift.GetMean();

      if (mean(2) == 0){
        final_depth.at<double>(row, col) = graph.ref_img_->depth.at<double>(row, col);
      } else {
        final_depth.at<double>(row, col) = mean(2);
      }
    }
  }

  cv::minMaxLoc(final_depth, &min, &max);
  final_depth /= max / 1.5;
  final_depth *= 255;
  stringstream ss1;
  ss1 << "fused_unfiltered_colmap_depth_threshold_" << threshold << ".jpg";
  cv::imwrite(ss1.str(), final_depth);



//  ofstream os;
//  os.open("pcl.ply");
//  unsigned long pts_count = 0;
//  for (int row = 0; row < graph.ref_img_->left_img_.rows; ++row) {
//    for (int col = 0; col < graph.ref_img_->left_img_.cols; ++col) {
//      if(fusion.map_row_map_col_pts[row][col].empty()) {
//        pts_count ++;
//      } else{
//        pts_count += fusion.map_row_map_col_pts[row][col].size();
//      }
//    }
//  }
//  os << "ply\n"
//        "format ascii 1.0\n"
//        "element vertex " << pts_count << "\n" << "property float x\n" << "property float y\n" << "property float z\n"
//     << "property uchar red\n" << "property uchar green\n"
//     << "property uchar blue\n"
//     << "element face 0\n"
//     << "end_header" << endl;
//
//  cv::Mat ref_img_color;
//  ref_img_color = cv::imread("/home/yeren/self/images/0000.jpg");
//  cv::resize(ref_img_color, ref_img_color, cv::Size(), 1, 1, cv::INTER_NEAREST);
//  for (int row = 0; row < graph.ref_img_->left_img_.rows; ++row) {
//    for (int col = 0; col < graph.ref_img_->left_img_.cols; ++col) {
//      cv::Vec3b color = ref_img_color.at<cv::Vec3b>(row, col);
//      if(fusion.map_row_map_col_pts[row][col].empty()) {
//        os << 0 << " " << 0 << " "
//           << 0 << " " << 0 << " "
//           << 0 << " " << 0 << endl;
//      }
//      for (vector<double> pt: fusion.map_row_map_col_pts[row][col]){
//      if(!fusion.map_row_map_col_pts[row][col].empty()) {
//        os << pt[0] << " " <<pt[1] << " "
//           << pt[2] << " " << (unsigned short) color(2) << " "
//           << (unsigned short) color(1) << " " << (unsigned short) color(0) << endl;
//      } else {
//        os << 0 << " " << 0 << " "
//           << 0 << " " << 0 << " "
//           << 0 << " " << 0 << endl;
//      }
//      }
//    }
//    os << "\n";
//  }
//  os.close();




//  // Test Mean shift
//  vector<vector<double>> pts;
//  vector<double> p1, p2, p3;
//  p1.push_back(0);
//  p1.push_back(0);
//  p1.push_back(0);
//  p2.push_back(10);
//  p2.push_back(10);
//  p2.push_back(10);
//  p3.push_back(11);
//  p3.push_back(11);
//  p3.push_back(11);
//
//  pts.push_back(p1);
//  pts.push_back(p2);
//  pts.push_back(p3);
//  sslam::MeanShift mean_shift(1, 1, &pts);
//  if( -1 == mean_shift.Shift()){
//    cout << "Mean shift fails." << endl;
//  }
//  mean_shift.Label();
//  for (int i = 0; i < mean_shift.labels_.size(); ++i) {
//    cout << "Label of " << i <<"-th point is "<< mean_shift.labels_[i] << endl;
//  }



}