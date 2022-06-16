#include "SSLAM/fusion.h"


namespace sslam{




int Fusion::fuse() {
  // 1. Reproject all pixels into camera coordinate.
  // 2. Convert points to world coordinate.
  // 3. Back project points into reference image.
  // 4. Apply mean shift to each pixel.

  int height = ref_img_->depth.rows;
  int width = ref_img_->depth.cols;


  // Reproject source image pixel to world coordinate of reference image.
  for (Frame::Ptr &frame: frames_) {
    Mat33 K_inv = frame->cam_->K().inverse().matrix();
    Mat33 K = frame->cam_->K();
    cout << "Project " << frame->id_ << " to reference image " << ref_img_->id_ << "." << endl;

    // R = rela_rt.R_j.t() * rela_rt.R_i;
    // T = rela_rt.R_j.t() * (-rela_rt.C_j + rela_rt.C_i);
    // -R^t * T
    Mat33 R = ref_img_->R_c_w.transpose() * frame->R_c_w;
    Vec3 T = ref_img_->R_c_w.transpose() * (-ref_img_->R_c_w * ref_img_->C_c_w + frame->R_c_w * frame->C_c_w);
    // Vec3 T = ref_img_->R_c_w.transpose() * (-ref_img_->C_c_w + frame->C_c_w);
    // Vec3 T = ref_img_->C_c_w - frame->C_c_w;

    for (int row = 0; row < height; ++row) {
      for (int col = 0; col < width; ++col) {
        if (frame->depth.at<double>(row, col) == 0){
          continue;
        }
        Vec3 world = PixelToWorld(row, col, R, T, K_inv, frame->depth.at<double>(row, col));

        Vec3 pixel = WorldToPixel(world, K);

        if (pixel(1) > height || pixel(1) < 0 || pixel(0) > width || pixel(0) < 0){
          continue;
        }
        if (map_row_map_col_pts.find((int)pixel(1)) == map_row_map_col_pts.end()){
          map_row_map_col_pts.insert(make_pair((int)pixel(1), std::map<int, vector<vector<double>>>()));
        }
        if(map_row_map_col_pts[(int)pixel(1)].find((int)pixel(0)) == map_row_map_col_pts[(int)pixel(1)].end()){
          map_row_map_col_pts[(int)pixel(1)].insert(make_pair((int)pixel(0), vector<vector<double>>()));
        }
        vector<double> point;
        point.push_back(world(0));
        point.push_back(world(1));
        point.push_back(world(2));
        map_row_map_col_pts[(int)pixel(1)][(int)pixel(0)].push_back(point);
      }
    }
  }
  Mat33 K_inv = ref_img_->cam_->K().inverse().matrix();
  Mat33 K = ref_img_->cam_->K();
  cout << "Project " << ref_img_->id_ << " to reference image " << ref_img_->id_ << "." << endl;
  Mat33 R;
  R << 1, 0, 0, 0, 1, 0, 0, 0, 1;
  Vec3 T(0, 0, 0);
  for (int row = 0; row < height; ++row) {
    for (int col = 0; col < width; ++col) {
      Vec3 world = PixelToWorld(row, col, R, T, K_inv, ref_img_->depth.at<double>(row, col));
      if (map_row_map_col_pts.find(row) == map_row_map_col_pts.end()) {
        map_row_map_col_pts.insert(make_pair(row, std::map<int, vector<vector<double>>>()));
      }
      if (map_row_map_col_pts[ row].find(col) == map_row_map_col_pts[row].end()) {
        map_row_map_col_pts[row].insert(make_pair(col, vector<vector<double>>()));
      }
      vector<double> point;
      point.push_back(world(0));
      point.push_back(world(1));
      point.push_back(world(2));
      map_row_map_col_pts[row][col].push_back(point);
    }

  }


}

Vec3 Fusion::PixelToWorld(int &row, int &col, Mat33 &R, Vec3 &T, Mat33 &K_inv, double &depth) {
  double cam[3];
  double world[3];
  cam[0] = K_inv(0, 0) * col + K_inv(0, 1) * row + K_inv(0, 2);
  cam[1] = K_inv(1, 0) * col + K_inv(1, 1) * row + K_inv(1, 2);
  cam[2] = K_inv(2, 0) * col + K_inv(2, 1) * row + K_inv(2, 2);

  cam[0] /= cam[2];
  cam[1] /= cam[2];
  cam[2] /= cam[2];
  cam[0] *= depth;
  cam[1] *= depth;
  cam[2] *= depth;


  world[0] = R(0, 0) * cam[0] + R(0, 1) * cam[1] + R(0, 2) * cam[2];
  world[1] = R(1, 0) * cam[0] + R(1, 1) * cam[1] + R(1, 2) * cam[2];
  world[2] = R(2, 0) * cam[0] + R(2, 1) * cam[1] + R(2, 2) * cam[2];

  world[0] += T(0);
  world[1] += T(1);
  world[2] += T(2);

  return Vec3(world[0], world[1], world[2]);
}

Vec3 Fusion::WorldToPixel(Vec3 world, Mat33 &K){


  double pix[3];

  pix[0] = K(0, 0) * world(0) + K(0, 1) * world(1) + K(0, 2) * world(2);
  pix[1] = K(1, 0) * world(0) + K(1, 1) * world(1) + K(1, 2) * world(2);
  pix[2] = K(2, 0) * world(0) + K(2, 1) * world(1) + K(2, 2) * world(2);

  pix[0] /= pix[2];
  pix[1] /= pix[2];
  pix[2] /= pix[2];

  return Vec3(pix[0], pix[1], pix[2]);

}



}