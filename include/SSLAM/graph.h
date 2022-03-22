#ifndef GRAPH_H
#define GRAPH_H

#include "frame.h"
#include "common_include.h"
#include "front_end.h"
#include "dataset.h"
#include "hmm.h"

namespace sslam {

typedef struct {
  cv::Mat R;
  cv::Mat T;
} RELA_RT;

class Graph {
 private:
  map<unsigned int, map<unsigned int, RELA_RT>> id_to_RTs_;
  map<unsigned int, map<unsigned int, cv::Mat>> id_to_NCC;
  vector<shared_ptr<Frame>> frames;
  cv::Mat depth;
  Dataset *dataset_;
  int ref_width_, ref_height_;
  double depth_max, depth_min;
  shared_ptr<Hmm> hmm;

 public:

  Graph(Dataset *dataset, shared_ptr<Frame> ref_img);
  shared_ptr<FrontEnd> front_end_;
  shared_ptr<Frame> ref_img_;


  //  void AddFrame(Frame &frame);
  bool ComputeRAndTOfTwoImgs(shared_ptr<Frame> frame1, shared_ptr<Frame> frame2, cv::Mat &R_, cv::Mat &t_);
  void ComputeAllRAndT();
  double ComputeNCC(Frame &ref, Frame &src, int row_pix, int col_pix, int win_size, double depth);
  void InitialRandomDepth();
  void ComputeHomography(const cv::Mat &K_src,
                         const cv::Mat &K_ref,
                         const cv::Mat &R,
                         const cv::Mat &T,
                         double &depth,
                         cv::Mat &H);
  void ComputeAllNCC(int win_size);
  void Propagate();
  void Sampling(vector<double> &all_prob);
  double ComputeEmissionProb(double ncc);
  shared_ptr<Frame> GetFrame(int idx);

//        ~Graph();
};

}

#endif