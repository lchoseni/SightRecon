#include <gtest/gtest.h>
#include <opencv2/features2d.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <utility>

#include "srecon/feature/extractor.h"
#include "srecon/feature/orb_extractor.h"
#include "fast9.cpp"

bool fast(vector<uchar> pixels) {
  // Use double pointer to find the continuous subset .
  size_t start = 0, end = 0, forward = 0;
  int count = 0, center_lower_threshold = 0, center_upper_threshold = 2;
  bool continuous = false;
  while (forward < pixels.size()) {
    // printf("%lu, %lu, %d, %lu\n ", start, end, count, forward);

    // The end moves forward if start pixel and end pixel are samely darker of
    // lighter
    if ((pixels[start] <= center_lower_threshold &&
         pixels[end] <= center_lower_threshold) ||
        (pixels[start] >= center_upper_threshold &&
         pixels[end] >= center_upper_threshold)) {
      end++;
      if (end >= pixels.size())

        end = end % pixels.size();

      count++;
      if (count >= 12) {
        continuous = true;
        break;
      }
    } else {
      if (start == end)
        end++;
      if (end >= start)
        forward += end - start;
      else
        forward += end - start + pixels.size();
      if (end >= pixels.size())

        end = end % pixels.size();

      start = end;
      count = 0;
    }
  }
  return continuous;
}

TEST(ORBExtractorTest, fastTest) {
  {
    vector<uchar> pixels{0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 1, 1, 1};
    ASSERT_FALSE(fast(pixels));
  }
  {
    vector<uchar> pixels{0, 0, 0, 0, 0, 0, 1, 1, 0, 0, 0, 0, 0, 1, 1, 1};
    ASSERT_FALSE(fast(pixels));
  }
  {
    vector<uchar> pixels{1, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 1, 1, 1};
    ASSERT_FALSE(fast(pixels));
  }
  {
    vector<uchar> pixels{0, 0, 0, 0, 0, 0, 1, 0, 1, 0, 0, 0, 0, 1, 1, 1};
    ASSERT_FALSE(fast(pixels));
  }
  {
    vector<uchar> pixels{0, 0, 0, 0, 0, 0, 1, 0, 0, 1, 0, 0, 0, 1, 1, 1};
    ASSERT_FALSE(fast(pixels));
  }
  {
    vector<uchar> pixels{0, 0, 0, 0, 0, 0, 1, 0, 0, 1, 0, 0, 0, 1, 1, 1};
    ASSERT_FALSE(fast(pixels));
  }
}

TEST(ORBExtractorTest, extractFeature) {
  // srecon::FeatureExtractor *extractor = new srecon::ORBExtractor(15, 35);

  // cv::Mat img = cv::imread("/home/yeren/Simple-SLAM/test/1.png",
  // cv::IMREAD_GRAYSCALE); img.convertTo(img, CV_32S); cv::Mat img1 =
  // cv::imread("/home/yeren/Simple-SLAM/test/1.png", cv::IMREAD_GRAYSCALE);

  // ASSERT_TRUE(img.rows > 0 && img.cols > 0);
  // vector<cv::KeyPoint> kps;
  // extractor->extract(img, kps);

  // ASSERT_TRUE(kps.size() > 0);

  // cout << "Find " << kps.size() << " key points." << endl;

  // for (auto &kp : kps)
  // {
  //     cv::circle(img, kp.pt,
  //                2,
  //                cv::Scalar(0, 0, 255));
  // }

  // if (!cv::imwrite("/home/yeren/Simple-SLAM/build/test/processed.jpg", img))
  // {
  //     cout << "Write processed.jpg failed!" << endl;
  // }
  // vector<cv::KeyPoint> kps1;

  // cv::FAST(img1, kps1, 50);
  // for (auto &kp : kps1)
  // {
  //     cv::circle(img1, kp.pt,
  //                2,
  //                cv::Scalar(0, 0, 255));
  // }

  // cv::imwrite("processed1.jpg", img1);

  // cv::Mat img2 = cv::imread("/home/yeren/Simple-SLAM/test/1.png",
  // cv::IMREAD_GRAYSCALE);
  cv::Mat img2 =
      cv::imread("/home/yeren/dataset/sequences/05/image_0/000000.png",
                 cv::IMREAD_GRAYSCALE);
  img2.convertTo(img2, CV_32S);
  cv::resize(img2, img2, cv::Size(), 1, 1, cv::INTER_NEAREST);

  vector<pair<int, int>> corners;
  // fast_corner_detect(img2, corners, 25);
  for (int row = 0; row < img2.rows; row++) {
    for (int col = 0; col < img2.cols; col++) {
        if(row < 3 || col < 3 || abs(img2.rows - row) < 3 || abs(img2.cols - col) < 3){
            continue;
        }
      if (is_fast_corner(img2, row, col)) {
        corners.push_back(make_pair(row, col));
      }
    }
  }

  for (pair<int, int> &corner : corners) {
    cv::Point2f pt(corner.second, corner.first);
    cv::circle(img2, pt, 2, cv::Scalar(0, 0, 255));
  }
  cv::imwrite("2006_learned_processed.jpg", img2);
  cout << "Find " << corners.size() << " circles." << endl;
}

int main(int argc, char **argv) {
  testing::InitGoogleTest(&argc, argv);

  // Runs all tests using Google Test.
  return RUN_ALL_TESTS();
}