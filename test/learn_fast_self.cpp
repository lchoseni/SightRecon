#include <algorithm>
#include <chrono>
#include <cstddef>
#include <ctime>
#include <dirent.h>
#include <iostream>
#include <list>
#include <map>
#include <opencv2/core/core.hpp>
#include <opencv2/features2d.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <random>
#include <utility>
#include <vector>

#include "generate_cpp_self.h"
#include "node.h"

using namespace std;

int radius = 3;
vector<pair<int, int>> pixels = {
    make_pair(-3, 0), make_pair(-3, 1),  make_pair(-2, 2),  make_pair(-1, 3),
    make_pair(0, 3),  make_pair(1, 3),   make_pair(2, 2),   make_pair(3, 1),
    make_pair(3, 0),  make_pair(3, -1),  make_pair(2, -2),  make_pair(1, -3),
    make_pair(0, -3), make_pair(-1, -3), make_pair(-2, -2), make_pair(-3, -1)};
int threshold = 25;
int continous_pixels = 9;

vector<int> max_idxs;
int node_count = 0;

// 0:not corner, 1: darker corner, 2: brighter corner
Feature *fast(cv::Mat &image, int row, int col) {

  int center = image.at<int>(row, col);
  int center_upper_threshold = center + threshold;
  int center_lower_threshold = center - threshold;

  // Collect all pixels on the circle into vectoe and
  // check whether there are 12 continuous pixels darker or lighter.
  array<short, feature_size> around_pixels;
  array<short, feature_size> around_pixels_cmp;
  for (size_t i = 0; i < pixels.size(); i++) {
    around_pixels[i] =
        image.at<int>(row + pixels[i].first, col + pixels[i].second);
    if (around_pixels[i] > center_upper_threshold)
      around_pixels_cmp[i] = 2;
    else if (around_pixels[i] < center_lower_threshold)
      around_pixels_cmp[i] = 1;
    else
      around_pixels_cmp[i] = 0;
  }

  short pixel_1 = around_pixels[0];
  short pixel_4 = around_pixels[4];
  short pixel_8 = around_pixels[8];
  short pixel_12 = around_pixels[12];
  short darker = 0, lighter = 0;
  if (pixel_1 >= center_upper_threshold)
    lighter += 1;
  else if (pixel_1 <= center_lower_threshold)
    darker += 1;
  if (pixel_8 >= center_upper_threshold)
    lighter += 1;
  else if (pixel_8 <= center_lower_threshold)
    darker += 1;

  if (!(darker == 2 || lighter == 2))
    return new Feature(0, around_pixels_cmp);

  if (pixel_4 >= center_upper_threshold)
    lighter += 1;
  else if (pixel_4 <= center_lower_threshold)
    darker += 1;
  if (pixel_12 >= center_upper_threshold)
    lighter += 1;
  else if (pixel_12 <= center_lower_threshold)
    darker += 1;

  if (!(darker >= 3 || lighter >= 3))
    return new Feature(0, around_pixels_cmp);

  // Use double pointer to find the continuous subset.
  std::size_t start = 0, end = 0, forward = 0, count = 0;

  while (forward < pixels.size()) {
    // The end moves forward if start pixel and end pixel are samely darker of
    // lighter
    if ((around_pixels[start] <= center_lower_threshold &&
         around_pixels[end] <= center_lower_threshold) ||
        (around_pixels[start] >= center_upper_threshold &&
         around_pixels[end] >= center_upper_threshold)) {
      end++;
      if (end >= around_pixels.size())
        end = end % around_pixels.size();

      count++;
      if (count >= continous_pixels) {
        // darker
        if (around_pixels[start] <= center_lower_threshold)
          return new Feature(1, around_pixels_cmp);
        // brighter
        if (around_pixels[start] >= center_upper_threshold)
          return new Feature(2, around_pixels_cmp);
      }
    } else {
      if (start == end)
        end++;
      if (end >= start)
        forward += end - start;
      else
        forward += end - start + around_pixels.size();
      if (end >= around_pixels.size())
        end = end % around_pixels.size();

      start = end;
      count = 0;
    }
  }

  return new Feature(0, around_pixels_cmp);
}

list<Feature *> detectCorners(vector<cv::Mat> &images) {
  // Check every pixel and find if there are 12 continuous darker or lighter
  // pixels and store its pixel values each.
  list<Feature *> all_corners;
  cout << "Has " << images.size() << " images......" << endl;
  for (size_t idx = 0; idx < images.size(); idx++) {
    if (idx % 10 == 0) {
      cout << "Detecting image " << idx << "......" << endl;
    }
    cv::Mat image = images[idx];
    int rows = images[idx].rows;
    int cols = images[idx].cols;
    for (int row = 0; row < rows; row++) {
      for (int col = 0; col < cols; col++) {
        if (row <= radius || abs(row - rows) <= radius || col <= radius ||
            abs(cols - col) <= radius)
          continue;
        all_corners.emplace_back(fast(image, row, col));
      }
    }
  }
  cout << "Finishing detecting." << endl;
  return all_corners;
}

double computeEntropy(Dis &dis) {

  if (dis.is_a_corner == 0 && dis.not_a_corner == 0)
    return 0;
  else if (dis.not_a_corner == 0)
    return (-(double)dis.is_a_corner / dis.number *
            log2((double)dis.is_a_corner / dis.number));
  else if (dis.is_a_corner == 0)
    return (-(double)dis.not_a_corner / dis.number *
            log2(dis.not_a_corner / dis.number));
  return (-(double)dis.is_a_corner / dis.number *
              log2((double)dis.is_a_corner / dis.number) -
          (double)dis.not_a_corner / dis.number *
              log2((double)dis.not_a_corner / dis.number));
}

// corners value is [positive(negtive, neither), [value at each pixel]]
void build(Node &node) {
  // print('data_ size is ', sys.getsizeof(node.data_))
  // test if the node has the same result
  size_t darker = 0, neither = 0, brighter = 0;
  {
    for (auto corner = node.data_->begin(); corner != node.data_->end();
         corner++) {
      if ((*corner)->result_ == 0)
        neither += 1;
      else if ((*corner)->result_ == 1)
        darker += 1;
      else
        brighter += 1;
    }
    if (darker == node.data_->size()) {
      node.result_ = 1;
      return;
    } else if (brighter == node.data_->size()) {
      node.result_ = 2;
      return;
    } else if (neither == node.data_->size()) {
      node.result_ = 0;
      return;
    } else if (node.depth_ == feature_size) {
      node.result_ = 0;
      return;
    }
  }
  node_count++;
  // find max entropy in all features
  // iterate all features.
  // for one value of one feature, count its result and calculate its entropy.
  // add all entropy of different value
  vector<double> all_entropy;
  vector<map<short, Dis>> each_feature_separaed_value_count;
  for (int i = 0; i < feature_size; i++) {
    if (node.available_feature_[i] == 0) {
      all_entropy.push_back(-1.f);
      continue;
    }
    map<short, Dis> featureDis;
    for (auto fea = node.data_->begin(); fea != node.data_->end(); fea++) {
      featureDis[(*fea)->around_pixels_cmp_[i]].number++;
      if ((*fea)->result_ == 0)
        featureDis[(*fea)->around_pixels_cmp_[i]].not_a_corner++;
      else
        featureDis[(*fea)->around_pixels_cmp_[i]].is_a_corner++;
    }
    each_feature_separaed_value_count.push_back(featureDis);
    vector<double> separated_value_entropy;
    vector<int> each_total;
    double total = 0;
    for (map<short, Dis>::iterator separated_value = featureDis.begin();
         separated_value != featureDis.end(); separated_value++) {
      separated_value_entropy.push_back(
          computeEntropy(separated_value->second));
      each_total.push_back(separated_value->second.number);
      total += separated_value->second.number;
    }
    double weighted_entropy_sum = 0;
    for (size_t i = 0; i < separated_value_entropy.size(); i++) {
      weighted_entropy_sum +=
          (each_total[i] / total) * separated_value_entropy[i];
    }

    all_entropy.push_back(weighted_entropy_sum);
  }
  // print('data_ size after cleaning... ', sys.getsizeof((node.data_)))
  double max_entropy = -1;
  int max_idx = -1;
  for (size_t i = 0; i < all_entropy.size(); i++) {
    if (max_entropy == -1) {
      max_entropy = all_entropy[i];
      max_idx = i;
      continue;
    }
    if (all_entropy[i] > max_entropy) {
      max_entropy = all_entropy[i];
      max_idx = i;
    }
  }

  node.feature_index_ = static_cast<short>(max_idx);
  if (node.feature_index_ == -1) {
    cerr << "error" << endl;
  }
  max_idxs.push_back(max_idx);
  array<short, feature_size> available_features = node.available_feature_;
  available_features[max_idx] = 0;
  map<short, list<Feature *>> min_feature_separated_value_corner;
  while (!node.data_->empty()) {

    if (min_feature_separated_value_corner.count(
            (*node.data_->begin())->around_pixels_cmp_[max_idx]) == 0) {
      min_feature_separated_value_corner.insert(
          make_pair((*node.data_->begin())->around_pixels_cmp_[max_idx],
                    list<Feature *>()));
    }
    list<Feature *> *list =
        &min_feature_separated_value_corner[(*node.data_->begin())
                                                ->around_pixels_cmp_[max_idx]];
    list->splice(list->begin(), *node.data_, node.data_->begin());
  }
  // node.data_->clear();
  // node.data_.shrink_to_fit();
  for (auto &corners_each_key : min_feature_separated_value_corner) {
    Node child =
        Node(corners_each_key.second, -1, available_features, node.depth_ + 1);
    build(child);
    node.children_.insert(make_pair(corners_each_key.first, child));
  }
}

vector<cv::Mat> readAllImages(string dir_path, bool random = false,
                              size_t size = -1) {

  DIR *dir;
  struct dirent *dp;

  if (NULL == (dir = opendir(dir_path.c_str()))) {
    cerr << "Can not open dir " << dir_path << endl;
  }
  vector<string> image_paths;
  vector<string> image_endings{".png",  ".jpg", ".jpeg",
                               ".tiff", ".bmp", ".gif"};
  int max = 0;
  while (NULL != (dp = readdir(dir))) {
    string image_name(dp->d_name);
    bool is_image = false;
    for (auto &ending : image_endings) {
      if (image_name.length() >= ending.length()) {
        if (0 == image_name.compare(image_name.length() - ending.length(),
                                    ending.length(), ending)) {
          is_image = true;
        }
      }
    }
    if (is_image) {
      image_paths.push_back(dir_path + string(dp->d_name));
      max++;
    }
  };
  vector<string> final_image_path;
  if (image_paths.size() < size) {
    final_image_path = image_paths;
  } else {
    if (!random)
      final_image_path.insert(final_image_path.begin(), image_paths.begin(),
                              image_paths.begin() + size);
    else {
      std::random_device dev;
      std::mt19937 rng(dev());
      std::uniform_int_distribution<std::mt19937::result_type> dist6(
          0, image_paths.size());

      for (size_t i = 0; i < size; i++) {
        int random_int = dist6(rng);
        if (std::find(final_image_path.begin(), final_image_path.end(),
                      image_paths[random_int]) == final_image_path.end()) {
          final_image_path.push_back(image_paths[random_int]);
        } else {
          i--;
        }
      }
    }
  }
  vector<cv::Mat> images;
  for (auto &image_path : final_image_path) {
    cv::Mat image = cv::imread(image_path, cv::IMREAD_GRAYSCALE);
    image.convertTo(image, CV_32S);
    images.push_back(image);
  }
  return images;
}

int main(int argc, char **argv) {
  int images_size = 10;
  // auto images =
  // readAllImages("/home/yeren/rgbd_dataset_freiburg1_desk/data/",
  //                             true, images_size);
  auto images = readAllImages("/home/yeren/dataset/sequences/05/image_0/", true,
                              images_size);
  // auto images =
  // readAllImages("/home/yeren/Simple-SLAM/test/", true, images_size);
  list<Feature *> corners = detectCorners(images);

  array<short, feature_size> available_feature;
  available_feature.fill(1);
  Node root = Node(corners, -1, available_feature, 0);

  cout << "Building Tree......" << endl;
  build(root);
  cout << "Finish Building Tree......" << endl;
  cout << "Have " << node_count << " nodes. " << endl;

  //   for (int i = 0; i < max_idxs.size(); i++) {
  //     cout << max_idxs[i] << " ";
  //     if (i % 50 == 0)
  //       cout << endl;
  //   }
  cout << "size is " << max_idxs.size() << endl;

  string dir_path = "/home/yeren/OneDrive/C C++ Projects/Simple-SLAM/test/";
  cv::Mat test_image = readAllImages(dir_path)[0];
  // cv::Mat test_image =
  //     cv::imread("/home/yeren/dataset/sequences/05/image_0/000000.png",
  //                cv::IMREAD_GRAYSCALE);
  test_image.convertTo(test_image, CV_32S);

  int circle = 0, count = 0;
  vector<pair<int, int>> detected_corners;
  for (int row = 0; row < test_image.rows; row++) {
    for (int col = 0; col < test_image.cols; col++) {
      if (row <= radius || abs(test_image.rows - row) <= radius ||
          col <= radius || abs(test_image.cols - col) <= radius)
        continue;
      count++;
      int center = test_image.at<int>(row, col);
      int center_upper_threshold = center + threshold;
      int center_lower_threshold = center - threshold;

      // Collect all pixels on the circle into vectoe and
      // check whether there are 12 continuous pixels darker or lighter.
      array<short, feature_size> around_pixels_cmp;
      for (size_t i = 0; i < pixels.size(); i++) {
        int val =
            test_image.at<int>(row + pixels[i].first, col + pixels[i].second);
        if (val > center_upper_threshold)
          around_pixels_cmp[i] = 2;
        else if (val < center_lower_threshold)
          around_pixels_cmp[i] = 1;
        else
          around_pixels_cmp[i] = 0;
      }
      Node *node = &root;
      while (node->result_ == -1) {
        if (node->children_.count(around_pixels_cmp[node->feature_index_]) != 0)
          node = &node->children_[around_pixels_cmp[node->feature_index_]];
        else
          break;
      }
      if (node->result_ == 1) {
        circle++;
        detected_corners.push_back(make_pair(row, col));
      }
    }
  }
  cout << "Find " << circle << " circles." << endl;
  for (pair<int, int> &corner : detected_corners) {
    cv::Point2f pt(corner.second, corner.first);
    cv::circle(test_image, pt, 1, cv::Scalar(0, 0, 255));
  }

  auto cur = std::chrono::system_clock::now();

  std::time_t cur_time = std::chrono::system_clock::to_time_t(cur);

  stringstream ss;
  ss << string(std::ctime(&cur_time)) << "_" << images_size << ".jpg";

  cv::imwrite(ss.str(), test_image);

  generate_cpp(root);
}