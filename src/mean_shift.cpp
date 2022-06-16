
#include "SSLAM/mean_shift.h"
#include <random>

#define EPSILON 0.01
#define CLUSTER_EPSILON 0.1

namespace sslam {

MeanShift::MeanShift(double bandwidth, double threshold, vector<vector<double>> *pts)
    : bandwidth_(bandwidth), threshold_(threshold), points_(pts) {
  labels_ = vector<int>(pts->size(), -1);
  final_points_ = vector<vector<double>>();
  for (vector<double> &point: *points_) {
    vector<double> p;
    p.push_back(point[0]);
    p.push_back(point[1]);
    p.push_back(point[2]);

    final_points_.push_back(p);
  }
}

int MeanShift::RandomChoose() {
  if (points_->empty()) {
    return -1;
  }
  std::random_device rd;
  std::mt19937 gen = std::mt19937(rd());

  uniform_real_distribution<double> dist = uniform_real_distribution<double>(0, points_->size());
  int idx = (int) dist(gen);
  return idx;
}

int MeanShift::Shift() {

  if (points_->empty()) {
    return -1;
  }
  for (int idx = 0; idx < points_->size(); idx++) {

    double z_sum = 0;
    double weight_sum = 0;
//    final_points_.emplace_back();
//    final_points_[idx][0] == (*points_)[idx][0];
//    final_points_[idx][1] == (*points_)[idx][1];
//    final_points_[idx][2] == (*points_)[idx][2];

    bool stop = false;
    while (!stop) {
      for (int ref = 0; ref < points_->size(); ref++) {
        double tmp_distance = Distance(final_points_[idx], (*points_)[ref]);
        double weight = GaussianKernel(tmp_distance, bandwidth_);
        z_sum += (*points_)[ref][2] * weight;
        weight_sum += weight;
      }
      vector<double> shift_vector;
      shift_vector.push_back(0);
      shift_vector.push_back(0);
      shift_vector.push_back(z_sum / weight_sum);

      if (Distance(final_points_[idx], shift_vector) < EPSILON) {
        stop = true;
      }
      final_points_[idx][0] = shift_vector[0];
      final_points_[idx][1] = shift_vector[1];
      final_points_[idx][2] = shift_vector[2];
    }
  }

}

int MeanShift::Label() {
  if(final_points_.empty()){
    return -1;

  }

  int label = -1;
  for (int idx = 0; idx < final_points_.size(); ++idx) {
    if (labels_[idx] != -1){
      continue;
    }
    label++;
    for (int ref = 0; ref < final_points_.size() ; ref ++){
      if(Distance(final_points_[idx], final_points_[ref]) < CLUSTER_EPSILON){
        labels_[ref] = label;
      }
    }
  }

}

// Pick the largest cluster and compute the mean.
Vec3 MeanShift::GetMean(){
  std::sort(labels_.begin(), labels_.end());
  vector<int> count;
  int cur_label = -1;
  for( int & label : labels_){
    if (label != cur_label){
      cur_label = label;
      count.push_back(0);
    }
    count[label]++;
  }

//  if(count.size() > 1){
//    cout << "Good！" <<endl;
//  }
  int max_idx = -1, max = -1;
  for(int idx_count = 0; idx_count < count.size(); idx_count++){
    if (max < count[idx_count]){
      max = count[idx_count];
      max_idx = idx_count;
    }
  }

  if (max == 1){
    return Vec3(0, 0, 0);
  }

  double x =0, y =0, z=0;
  for(int label_idx = 0; label_idx < labels_.size(); label_idx ++){
    if (labels_[label_idx] ==max_idx) {
      x += (*points_)[label_idx][0];
      y += (*points_)[label_idx][1];
      z += (*points_)[label_idx][2];
    }
  }

  x /= max;
  y /=max;
  z/= max;

  return Vec3(x, y, z);
}




double MeanShift::Distance(vector<double> &p1, vector<double> &p2) {
  return abs(p1[2] - p2[2]);
}

double MeanShift::GaussianKernel(double distance, double bandwidth) {
  return exp(-0.5 * (distance * distance) / (bandwidth * bandwidth));
}



}
