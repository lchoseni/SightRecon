#include "srecon/feature/orb_extractor.h"
#include <opencv2/features2d/features2d.hpp>
#include "opencv2/imgproc.hpp"
#include "opencv2/highgui.hpp"

#include "opencv2/imgcodecs.hpp"

namespace srecon
{

    ORBExtractor::ORBExtractor(int win_size, int threshold) : win_size_(win_size), threshold_(threshold), continous_pixels_(12), adjacent_distance_(4), max_nodes_(25)
    {
        if (win_size % 2 != 1)
        {
            cout << "Window size should be odd." << endl;
        }
        half_win_size_ = win_size / 2;

        // Calculate the orb circle pixel positions and make it ture that all pixels are
        // symmetric horizontally or vertically.
        const int radius = half_win_size_;

        int last_col_max = -1;
        for (int current_row = 0; current_row <= radius; current_row++)
        {
            int current_col = last_col_max + 1;
            while (true)
            {
                if (0 != onCircle(current_col, radius - current_row, radius) || current_col > radius)
                {
                    last_col_max = current_col - 1;
                    circle_positions_of_des_.push_back(last_col_max);
                    break;
                }
                current_col++;
            }
        }

        circle_positions_of_fast_.emplace_back(-3, 0);
        circle_positions_of_fast_.emplace_back(-3, 1);
        circle_positions_of_fast_.emplace_back(-2, 2);
        circle_positions_of_fast_.emplace_back(-1, 3);
        circle_positions_of_fast_.emplace_back(0, 3);
        circle_positions_of_fast_.emplace_back(1, 3);
        circle_positions_of_fast_.emplace_back(2, 2);
        circle_positions_of_fast_.emplace_back(3, 1);
        circle_positions_of_fast_.emplace_back(3, 0);
        circle_positions_of_fast_.emplace_back(3, -1);
        circle_positions_of_fast_.emplace_back(2, -2);
        circle_positions_of_fast_.emplace_back(1, -3);
        circle_positions_of_fast_.emplace_back(0, -3);
        circle_positions_of_fast_.emplace_back(-1, -3);
        circle_positions_of_fast_.emplace_back(-2, -2);
        circle_positions_of_fast_.emplace_back(-3, -1);
    }

    int ORBExtractor::extract(cv::Mat &frame, vector<cv::KeyPoint> &keyPoints)
    {

        //  cv::medianBlur(frame, frame, 3);
        for (int row = 0; row < frame.rows; row++)
        {
            for (int col = 0; col < frame.cols; col++)
            {
                // Ensure the center of ORB feature is not close to boarder.
                if (row < half_win_size_ || frame.rows - row < half_win_size_ || col < half_win_size_ || frame.cols - col < half_win_size_)
                {
                    continue;
                }
                if (0 == fast(frame, row, col))
                {
                    keyPoints.push_back(cv::KeyPoint(col, row, 0));
                }
            }
        }

        // filter key points.
        return distribute(frame, keyPoints);
    }

    // judge whether the pixel is on the circle.
    int ORBExtractor::onCircle(int h_dis, int v_dis, int radius)
    {

        bool up_left_in_circle = sqrt((h_dis - 0.5) * (h_dis - 0.5) + (v_dis + 0.5) * (v_dis + 0.5)) < static_cast<float>(radius);
        bool up_right_in_circle = sqrt((h_dis + 0.5) * (h_dis + 0.5) + (v_dis + 0.5) * (v_dis + 0.5)) < static_cast<float>(radius);
        bool bottom_left_in_circle = sqrt((h_dis - 0.5) * (h_dis - 0.5) + (v_dis - 0.5) * (v_dis - 0.5)) < static_cast<float>(radius);
        bool bottom_right_in_circle = sqrt((h_dis + 0.5) * (h_dis + 0.5) + (v_dis - 0.5) * (v_dis - 0.5)) < static_cast<float>(radius);

        // the pixel is not on circle if there is four corner distance is less than radius or four corner distance
        // is greater than radius
        bool all_corners_in_circle = up_left_in_circle && up_right_in_circle && bottom_left_in_circle && bottom_right_in_circle;
        bool all_corners_out_of_circle = !up_left_in_circle && !up_right_in_circle && !bottom_left_in_circle && !bottom_right_in_circle;
        if (all_corners_in_circle || all_corners_out_of_circle)
        {
            return 1;
        }
        else
        {
            return 0;
        }
    }

    int ORBExtractor::fast(cv::Mat &frame, int row, int col)
    {

        int darker = 0, lighter = 0;
        // circle_positions_of_fast_[0, 4, 8, 12]

        // First detect pixel on the circle at horizontal and vertical direction
        // And then find whether there has three pixels darker or lighter than other one pixel.

        int pixel_1 = frame.at<int>(row + circle_positions_of_fast_[0].first, col + circle_positions_of_fast_[0].second);
        int pixel_4 = frame.at<int>(row + circle_positions_of_fast_[4].first, col + circle_positions_of_fast_[4].second);
        int pixel_8 = frame.at<int>(row + circle_positions_of_fast_[8].first, col + circle_positions_of_fast_[8].second);
        int pixel_12 = frame.at<int>(row + circle_positions_of_fast_[12].first, col + circle_positions_of_fast_[12].second);

        int center = frame.at<int>(row, col);
        auto center_upper_threshold = static_cast<int>(center + threshold_);
        auto center_lower_threshold = static_cast<int>(center  - threshold_);
        if (pixel_1 >= center_upper_threshold)
            lighter++;
        else if (pixel_1 <= center_lower_threshold)
            darker++;

        if (pixel_8 >= center_upper_threshold)
            lighter++;
        else if (pixel_8 <= center_lower_threshold)
            darker++;

        if (!(darker == 2 || lighter == 2))
            return 1;

        if (pixel_4 >= center_upper_threshold)
            lighter++;
        else if (pixel_4 <= center_lower_threshold)
            darker++;
        if (pixel_12 >= center_upper_threshold)
            lighter++;
        else if (pixel_12 <= center_lower_threshold)
            darker++;

        if (!(darker >= 3 || lighter >= 3))
            return 1;

        // Collect all pixels on the circle into vectoe and
        // check whether there are 12 continuous pixels darker or lighter.
        vector<int> pixels(16);

        for (auto &iter : circle_positions_of_fast_)
            pixels.push_back(frame.at<int>(row + iter.first, col + iter.second));

        // Use double pointer to find the continuous subset .
        size_t start = 0, end = 0, forward = 0;
        int count = 0;

        while (forward < pixels.size())
        {
            // The end moves forward if start pixel and end pixel are samely darker of lighter
            if ((pixels[start] <= center_lower_threshold && pixels[end] <= center_lower_threshold) || (pixels[start] >= center_upper_threshold && pixels[end] >= center_upper_threshold))
            {
                end++;
                if (end >= pixels.size())

                    end = end % pixels.size();

                count++;
                if (count >= continous_pixels_)
                {
                    return 0;
                }
            }
            else
            {
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

        return 1;
    }

    int ORBExtractor::score(cv::Mat &frame, int row, int col)
    {
        int center = frame.at<int>(row, col);
        int score = 0;
        for (auto &iter : circle_positions_of_fast_)
        {
            score += abs((static_cast<int>(center) - static_cast<int>(frame.at<int>(row + iter.first, col + iter.second))));
        }
        return score;
    }

    int ORBExtractor::isAdjacent(cv::KeyPoint &p1, cv::KeyPoint &p2)
    {
        return sqrt(pow(p1.pt.x - p2.pt.x, 2) + pow(p1.pt.y - p2.pt.y, 2)) < adjacent_distance_;
    }

    bool compare_response(cv::KeyPoint first, cv::KeyPoint second)
    {
        if (first.response > second.response)
            return true;
        else
            return false;
    }

    int ORBExtractor::distribute(cv::Mat &frame, vector<cv::KeyPoint> &kps)
    {
        // Divide image into several nodes and choose the best keypoint for each node.
        Node whole_img(0, frame.rows, 0, frame.cols);

        for (size_t idx = 0; idx < kps.size(); idx++)
        {
            kps[idx].response = score(frame, kps[idx].pt.y, kps[idx].pt.x);
        }
        sort(kps.begin(), kps.end(), compare_response);
        if (kps.size() > 1000)
        {
            kps.erase(kps.begin() + 1000, kps.end());
        }

        whole_img.kps_ = kps;

        vector<Node> nodes;
        nodes.push_back(whole_img);

        while (nodes.size() < max_nodes_)
        {
            auto n_iter = nodes.begin();
            vector<vector<Node>> sub_nodes;
            bool divided = false;
            while (n_iter != nodes.end())
            {
                vector<Node> divided_nodes;
                if (0 == n_iter->divide(divided_nodes))
                {
                    sub_nodes.push_back(divided_nodes);
                    divided = true;
                }
                n_iter++;
            }
            if(!divided){
                break;
            }

            nodes.clear();
            for (auto iter = sub_nodes.begin(); iter != sub_nodes.end(); iter++)
            {
                nodes.insert(nodes.end(), iter->begin(), iter->end());
            }
        }

        // Pick the best keypoint in each node.
        vector<cv::KeyPoint> filtered_kps;

        for (auto &node : nodes)
        {
            int max_score = -1, score_kp = -1;
            int max_kp = -1;

            for (size_t idx = 0; idx < node.kps_.size(); idx++)
            {
                node.kps_[idx].response = score_kp = score(frame, node.kps_[idx].pt.y, node.kps_[idx].pt.x);

            }
            sort(node.kps_.begin(), node.kps_.end(), compare_response);
            if (!nodes.empty())
            {
                filtered_kps.insert(filtered_kps.end(), node.kps_.begin(), node.kps_.begin() + 10);
            }
        }

        kps.clear();
        kps.insert(kps.begin(), filtered_kps.begin(), filtered_kps.end());

        return 0;
    }

    Node::Node(int start_row, int end_row, int start_col, int end_col) : start_row_(start_row), end_row_(end_row), start_col_(start_col), end_col_(end_col) {}

    Node::~Node() {}

    int Node::divide(vector<Node> &nodes)
    {

        // Distribute all points into sub nodes and then
        // remove the original node.


        if(start_row_ >= end_row_ - 1 || start_col_ >= end_col_ - 1){
            return 1;
        }

        nodes.emplace_back(Node(start_row_, (start_row_ + end_row_) / 2, start_col_, (start_col_ + end_col_) / 2));
        nodes.emplace_back(Node((start_row_ + end_row_) / 2 + 1, end_row_, start_col_, (start_col_ + end_col_) / 2));
        nodes.emplace_back(Node(start_row_, (start_row_ + end_row_) / 2, (start_col_ + end_col_) / 2 + 1, end_col_));
        nodes.emplace_back(Node((start_row_ + end_row_) / 2 + 1, end_row_, (start_col_ + end_col_) / 2 + 1, end_col_));

        if (!kps_.empty())
        {
            for (auto &kp : kps_)
            {
                nodes[2 * (kp.pt.y > ((start_row_ + end_row_) / 2) ? 1 : 0) + (kp.pt.x > ((start_col_ + end_col_) / 2) ? 1
                                                                                                                       : 0)]
                    .kps_.push_back(kp);
            }
        }
        else
        {
            return 1;
        }
        auto iter = nodes.begin();
        while (iter != nodes.end())
        {
            if (iter->kps_.empty())
            {
                iter = nodes.erase(iter);
            }
            else
            {
                iter++;
            }
        }

        return 0;
    }
}