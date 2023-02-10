#ifndef SRECON_ORB_EXTRACTOR_H
#define SRECON_ORB_EXTRACTOR_H

#include "extractor.h"
#include <list>

namespace srecon
{

    class ORBExtractor : public FeatureExtractor
    {
    private:
        int win_size_;
        int half_win_size_;
        float threshold_;
        int continous_pixels_;
        int adjacent_distance_;
        int max_nodes_;
        vector<int> circle_positions_of_des_;
        vector<pair<int, int>> circle_positions_of_fast_;

        int fast(cv::Mat &frame, int row, int col);
        int onCircle(int h_dis, int v_dis, int radius);
        int distribute(cv::Mat &frame, vector<cv::KeyPoint> &kps);
        int score(cv::Mat &frame, int row, int col);
        int isAdjacent(cv::KeyPoint &p1, cv::KeyPoint &p2);

    public:
        ORBExtractor(int win_size, int threshold);
        ~ORBExtractor();

        virtual int extract(cv::Mat &frame, vector<cv::KeyPoint> &keyPoints);
    };

    class Node
    {
    public:
        int start_row_, end_row_, start_col_, end_col_;
        vector<cv::KeyPoint> kps_;

        Node(int start_row, int end_row, int start_col, int end_col);
        ~Node();

        int divide(vector<Node> &nodes);
        int pickBest();
    };

}

#endif