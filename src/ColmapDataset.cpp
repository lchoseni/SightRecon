#include <boost/format.hpp>
#include "SSLAM/ColmapDataset.h"

namespace sslam {


shared_ptr<Frame> ColmapDataset::GetNextFrame(int idx) {

  if (cur_img_index == ref_img_index) {
    cur_img_index++;
  }

  boost::format data_fmt("%simages/%04d.jpg");
  boost::format scale_fmt("%simages/scale/%04d.jpg");

  boost::format ext_m_fmt("%stest/images.txt");
  cv::Mat left, right;

  if (idx != -1) {
    ref_img_index = idx;
    if (access((data_fmt % dataset_dir % idx).str().c_str(), F_OK) == -1) {
      return nullptr;
    }
    cout << (data_fmt % dataset_dir % idx).str().c_str() << endl;
    left = cv::imread((data_fmt % dataset_dir % idx).str(), CV_LOAD_IMAGE_GRAYSCALE);
    right = cv::imread((data_fmt % dataset_dir % idx).str(), CV_LOAD_IMAGE_GRAYSCALE);

  } else {
    if (access((data_fmt % dataset_dir % cur_img_index).str().c_str(), F_OK) == -1) {
      return nullptr;
    }
    cout << (data_fmt % dataset_dir % cur_img_index).str().c_str() << endl;
    left = cv::imread((data_fmt % dataset_dir % cur_img_index).str(), CV_LOAD_IMAGE_GRAYSCALE);
    right = cv::imread((data_fmt % dataset_dir % cur_img_index).str(), CV_LOAD_IMAGE_GRAYSCALE);

  }
//  cur_img_index += 4;
  if (cur_img_index > 10) {
    return nullptr;
  }

  cv::Mat resized_left, resized_right;

  cv::resize(left, resized_left, cv::Size(), scale, scale, cv::INTER_NEAREST);
  cv::resize(right, resized_right, cv::Size(), scale, scale, cv::INTER_NEAREST);

//  cv::imwrite((data_fmt % dataset_dir % cur_img_index).str().c_str(), resized_left);
//  cout << (ext_m_fmt % dataset_dir % cur_img_index).str().c_str() << endl;
//  stringstream  ss;
//  ss << (scale_fmt % dataset_dir % cur_img_index);
//  cv::imwrite(ss.str(), resized_left);

  std::ifstream fin;

  fin = std::ifstream((ext_m_fmt % dataset_dir).str());

  if (!fin) {
    return nullptr;
  }

  std::string line;
  int line_number = 0;
  int image_id;
  double quaternion[4];
  double t[3];
  while (std::getline(fin, line)) {

    line_number++;
    if (line_number <= 4) {
      continue;
    }
    if (line_number % 2 == 0) {
      continue;
    }
    string space_delimiter = " ";
    vector<string> words{};

    size_t pos = 0;
    while ((pos = line.find(space_delimiter)) != string::npos) {
      words.push_back(line.substr(0, pos));
      line.erase(0, pos + space_delimiter.length());
    }
    image_id = stoi(words[0]) - 1;
    if (image_id != cur_img_index) {
      continue;
    }
    for (const auto &str: words) {
      cout << str << " ";
    }
    cout << endl;
    quaternion[0] = stod(words[1]);
    quaternion[1] = stod(words[2]);
    quaternion[2] = stod(words[3]);
    quaternion[3] = stod(words[4]);
    t[0] = stod(words[5]);
    t[1] = stod(words[6]);
    t[2] = stod(words[7]);
    printf("%s\n", line.c_str());

  }
  fin.close();
  Eigen::Quaterniond q(quaternion[0], quaternion[1], quaternion[2], quaternion[3]);
  q.normalize();

  Frame *new_frame;
  if (idx != -1) {
    new_frame = new Frame(idx);
  } else {
    new_frame = new Frame(cur_img_index);
  }
  new_frame->left_img_ = resized_left;
  new_frame->right_img_ = resized_right;
  new_frame->SetCamera(left_camera_);
//  new_frame->Tcw = SE3(*K, *t);
  new_frame->R_c_w = q.toRotationMatrix().transpose();
  new_frame->C_c_w = Vec3(-t[0], -t[1], -t[2]);
  if (idx == -1) {
    cur_img_index++;
  }
  return shared_ptr<Frame>(new_frame);
}

std::shared_ptr<Camera> ColmapDataset::GetCamera(int id) {
  if (left_camera_ == nullptr) {
    std::vector<std::shared_ptr<Eigen::Matrix<double, 3, 3>>> Ks;
    std::vector<std::shared_ptr<Vec3>> ts;


    left_camera_ = std::shared_ptr<Camera>(
        new Camera(2758.3792686780812, 2753.4040475603097, 1520.6900000000001, 1006.8099999999999, SE3(SO3(), Vec3(0,0,0))));
    right_camera_ = std::shared_ptr<Camera>(
        new Camera(2758.3792686780812, 2753.4040475603097, 1520.6900000000001, 1006.8099999999999, SE3(SO3(), Vec3(0,0,0))));

  }
  switch (id) {
    case 0:return left_camera_;
    case 1:return right_camera_;
    default:return nullptr;
  }

}
}