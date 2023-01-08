#include <iostream>
#include <sstream>
#include <fstream>
#include <sstream>
#include <unistd.h>
#include <list>
#include <stdlib.h>
#include <math.h>
#include <algorithm>
#include <string>
#include <dirent.h>
#include "query_tree.h"
#include "generate.cpp"

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/features2d.hpp>
#include <opencv2/imgproc.hpp>
#include <vector>

using namespace std;


extern pair<int, int> dir[17];

vector<int> depths;
vector<int> min_idxs;

// Test a pixel in the image to see if it is a "positive" corner
int is_corner_positive(const cv::Mat &frame, int row, int col, int cb, int N)
{
    int num_for_corner = N;

    int num_consecutive = 0;
    int first_cons = 0;
    int val;
    for (int i = 0; i < 16; i++)
    {
        val = frame.at<int>(row + dir[i].first, col + dir[i].second);
        if (val > cb)
        {
            num_consecutive++;

            if (num_consecutive == num_for_corner)
                return 1;
        }
        else
        {
            if (num_consecutive == i)
                first_cons = i;

            num_consecutive = 0;
        }
    }
    return first_cons + num_consecutive >= num_for_corner;
}

// Test a pixel in the image to see if it is a "negative" corner
int is_corner_negative(const cv::Mat &frame, int row, int col, int c_b, int N)
{
    int num_for_corner = N;
    int num_consecutive = 0;
    int first_cons = 0;
    int val;
    for (int i = 0; i < 16; i++)
    {
        val = frame.at<int>(row + dir[i].first, col + dir[i].second);
        if (val < c_b)
        {
            num_consecutive++;

            if (num_consecutive == num_for_corner)
                return 2;
        }
        else
        {
            if (num_consecutive == i)
                first_cons = i;

            num_consecutive = 0;
        }
    }

    return first_cons + num_consecutive >= num_for_corner;
}

struct Feature
{
    // pos_test stored the results of the test for positivity as booleans
    // stored in the first 16 bits.
    unsigned short pos_test;
    unsigned short neg_test;
    bool is_a_feature;

};

void do_frame(const cv::Mat &frame, list<Feature> &corners, int barrier, int N)
{
    // Process a frame, extracting all potential corners
    int count = 0;
    for (int row = 0; row < frame.rows; row++)
    {
        for (int col = 0; col < frame.cols; col++)
        {
            if(row <= 3 || abs(frame.rows - row) <= 3 || col <=3 || abs(frame.cols - col) <=3)
                continue; 
            if(count == 9)
                int a = 1;
            count++;
            Feature f;

            f.pos_test = 0;
            f.neg_test = 0;
            bool p = is_corner_positive(frame, row, col, frame.at<int>(row, col) + barrier, N);
            bool n = is_corner_negative(frame, row, col, frame.at<int>(row, col) - barrier, N);

            for (int i = 0; i < 16; i++)
            {
                f.pos_test |= (frame.at<int>(row + dir[i].first, col + dir[i].second) > frame.at<int>(row, col) + barrier) << i;
                f.neg_test |= (frame.at<int>(row + dir[i].first, col + dir[i].second) < frame.at<int>(row, col) - barrier) << i;
            }
            f.is_a_feature = p || n;

            corners.push_back(f);
        }
    }
}

struct count_features_if_feature
{
    bool operator()(const Feature &f)
    {
        return f.is_a_feature;
    }
};

struct count_features_if_positive_pixel
{
    count_features_if_positive_pixel(int n)
        : N(n) {}
    int N;

    bool operator()(const Feature &f)
    {
        return f.pos_test & (1 << N);
    }
};

struct count_features_if_negative_pixel
{
    count_features_if_negative_pixel(int n)
        : N(n) {}
    int N;
    bool operator()(const Feature &f)
    {
        return f.neg_test & (1 << N);
    }
};

typedef list<Feature> fs;

bool test_feature(fs::iterator i, int n, bool posi)
{
    return (posi == true && count_features_if_positive_pixel(n)(*i)) ||
           ((posi == false) && count_features_if_negative_pixel(n)(*i));
}

// Find the entropy of a binary distribution given the total number of points
// and the number in class 1
float entropy(int n, int c1)
{
    // n is total number, c1 in num in class 1
    if (n == 0)
        return 0;
    else if (c1 == 0 || c1 == n)
        return 0;
    else
    {
        float p1 = (float)c1 / n;
        float p2 = 1 - p1;

        return -n * (p1 * log(p1) + p2 * log(p2)) / log(2.f);
    }
}

int find_best_split(list<Feature> &corners, int idepth)
{
    // Use entropy to find the best question to ask

    float min_entropy = HUGE_VAL;
    int best_entropy_pos = 0;

    // The pointer to the middle is always cached and therefore the left
    // and right pixels can be accessed very easily. This forces the first question
    // to only access these pixels. Yes, it is a hack, but it might speed things up
    // if no caching is used.
    bool force_first_question = false;

    int total_features = 0;
    for (fs::iterator i = corners.begin(); i != corners.end(); i++)
        if (i->is_a_feature)
            total_features++;

    // float max_ent = entropy(corners.size(), total_features);

    for (int n = 0; n < 16; n++)
    {
        // Compute the resulting distribution of features and non-features after a question
        int num_pos = 0, num_neg = 0, num_neither = 0;
        int fea_pos = 0, fea_neg = 0, fea_neither = 0;
        int count = 0;
        for (fs::iterator i = corners.begin(); i != corners.end(); i++)
        {
            if(count == 10){
                int b = 1;
            }
            count += 1;
            if (test_feature(i, n, 1))
            {
                num_pos++;
                if (i->is_a_feature)
                    fea_pos++;
            }
            else if (test_feature(i, n, 0))
            {
                num_neg++;
                if (i->is_a_feature)
                    fea_neg++;
            }
            else
            {
                num_neither++;
                if (i->is_a_feature)
                    fea_neither++;
            }
        }

        float ent = entropy(num_pos, fea_pos) + entropy(num_neg, fea_neg) + entropy(num_neither, fea_neither);

        // If force_first_question is set, then only consider
        // pixels 4 and 12. (or 5 and 13 if you look at the figure
        // in the paper)
        // The result is that the first question will always
        // use the main pointer.
        bool allow = !force_first_question || (idepth > 0) || (n == 4) || (n == 12);

        // Pick the question which results in the smallest total entropy
        if (ent < min_entropy && allow)
        {
            min_entropy = ent;
            best_entropy_pos = n;
        }
    }

    // For all 16 possible questions, the question causing the largest entropy change has been computed.
    return best_entropy_pos;
}

int build_tree(list<Feature> &corners, int idepth, vector<query_tree> &tree, unsigned long int &questions)
{
    depths.push_back(idepth);
    int size = corners.size();
    int s = find_best_split(corners, idepth);
    min_idxs.push_back(s);

    // Compute the total number of questions asked
    // to completely classify everything in the tree
    questions += size;

    // Based on the best question, split the list in to 3 sublists
    list<Feature> passed_pos, passed_neg, passed_neither;
    while (!corners.empty())
    {
        if (test_feature(corners.begin(), s, 1))
            passed_pos.splice(passed_pos.begin(), corners, corners.begin());
        else if (test_feature(corners.begin(), s, 0))
            passed_neg.splice(passed_neg.begin(), corners, corners.begin());
        else
            passed_neither.splice(passed_neither.begin(), corners, corners.begin());
    }

    // Compute the distribution of corners and non-corners
    int fea_pos = count_if(passed_pos.begin(), passed_pos.end(), count_features_if_feature());
    int num_pos = passed_pos.size();

    int fea_neg = count_if(passed_neg.begin(), passed_neg.end(), count_features_if_feature());
    int num_neg = passed_neg.size();

    int fea_nei = count_if(passed_neither.begin(), passed_neither.end(), count_features_if_feature());
    int num_nei = passed_neither.size();

    // Allocate a node for for the tree
    int current = tree.size();
    tree.push_back(query_tree());

    query_tree q;
    q.pixel = s;
    q.num_of_tests = size;

    // Now go and process the sublists

    if (fea_pos == 0)
        q.positive = query_tree::is_not_a_feature;
    else if (fea_pos == num_pos)
        q.positive = query_tree::is_a_feature;
    else
        q.positive = build_tree(passed_pos, idepth + 1, tree, questions);

    if (fea_neg == 0)
        q.negative = query_tree::is_not_a_feature;
    else if (fea_neg == num_neg)
        q.negative = query_tree::is_a_feature;
    else
        q.negative = build_tree(passed_neg, idepth + 1, tree, questions);

    if (fea_nei == 0)
        q.neither = query_tree::is_not_a_feature;
    else if (fea_nei == num_nei)
        q.neither = query_tree::is_a_feature;
    else
        q.neither = build_tree(passed_neither, idepth + 1, tree, questions);

    tree[current] = q;
    return current;
}

int mmain(int argc, char **argv)
{

    list<Feature> corners;

    int frames = 0;
    
    string dir_path = "/home/yeren/dataset/sequences/05/image_0/";
    // string dir_path = "/home/yeren/Simple-SLAM/test/";
    DIR *dir;
    struct dirent *dp;

    if (NULL == (dir = opendir(dir_path.c_str())))
    {
        cerr << "Can not open dir " << dir_path << endl;
    }

    vector<string> image_paths;
    vector<string> image_endings{".png", ".jpg", ".jpeg", ".tiff", ".bmp", ".gif"};
    int max = 0;
    while (NULL != (dp = readdir(dir)) && max < 20)
    {
        string image_name(dp->d_name);
        bool is_image = false;
        for (auto &ending : image_endings)
        {
            if (image_name.length() >= ending.length())
            {
                if (0 == image_name.compare(image_name.length() - ending.length(), ending.length(), ending))
                {
                    is_image = true;
                }
            }
        }
        if (is_image)
        {
            image_paths.push_back(dir_path + string(dp->d_name));
            max++;
        }
    };

    // Collect feature vectors from all pixels in to a huge list
    int continuous_pixels = 9;
    int threshold = 25;
    for (auto &image_path : image_paths)
    {
        cv::Mat image;
        image = cv::imread(image_path, cv::IMREAD_GRAYSCALE);
        image.convertTo(image, CV_32S);
        do_frame(image, corners, threshold, continuous_pixels);
    }

    unsigned long cs = corners.size();

    cerr << endl
         << "Number of frames:   " << frames << endl
         << "Potential features: " << corners.size() << endl
         << "Real features:      " << count_if(corners.begin(), corners.end(), count_features_if_feature()) << endl
         << endl;

    // Output a header
    ostringstream comments;
    comments << "Automatically generated code\n"
                "Parameters:\n"
                "splat_subtree = "
             << "1" << endl
             << "2" << endl
             << "Data:\n"
             << "Number of frames:    " << frames << endl
             << "Potential features:  " << corners.size() << endl
             << "Real features:       " << count_if(corners.begin(), corners.end(), count_features_if_feature()) << endl;

    unsigned long int questions = 0;
    vector<query_tree> tree;
    build_tree(corners, 0, tree, questions);
    cerr << "Have " << tree.size() << " nodes." << endl;
    // comments << "Questions per pixel: " << 1.0 * questions / cs << endl;
    cerr << "Questions per pixel: " << 1.0 * questions / cs << endl;

    // if (*lang == "c++")
    //     generate_cpp(tree, comments.str());
    // else if (*lang == "c")
    //     generate_c(tree, comments.str());
    // else if (*lang == "matlab")
    //     generate_matlab(tree, comments.str());
    // else if (*lang == "none")
    //     ;
    // else
    // {
    //     cerr << "Unknown language: \"" << *lang << "\"\n";
    //     return 1;
    // }
    generate_cpp(tree, comments.str());
    // int count = 0;
    // for(int i =0; i < depths.size(); i++){
    //     cout << depths[i] << " ";
    //     count++;
    //     if (count % 50 == 0) {
    //         cout << endl;
    //     }
    // }
    // cout << endl << count << endl;
    // count = 0;
    // cout << "min_idxs is " << endl;
    // for(int i =0; i < min_idxs.size(); i++){
    //     cout << min_idxs[i] << " ";
    //     count++;
    //     if (count % 50 == 0) {
    //         cout << endl;
    //     }
    // }
    // cout << endl << count << endl;
    return 0;
}

int main(int argc, char **argv)
{

    mmain(argc, argv);

    return 0;
}
