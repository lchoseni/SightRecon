#ifndef NODE_H
#define NODE_H

#include <list>
#include <map>
#include <vector>

using namespace std;

const int feature_size = 16;

struct Feature {
  short result_;
  array<short, feature_size> around_pixels_cmp_;

  Feature(short result, array<short, feature_size> &around_pixels_cmp)
      : result_(result), around_pixels_cmp_(around_pixels_cmp) {}

  Feature() {}
};

struct Dis {
  unsigned int number = 0;
  unsigned int is_a_corner = 0;
  unsigned int not_a_corner = 0;
};

class Node {
public:
  list<Feature *> *data_;
  int feature_index_;
  array<short, feature_size> available_feature_;
  int depth_;
  short result_;
  map<short, Node> children_;

  Node(list<Feature *> &data, short feature_index,
       array<short, feature_size> &available_feature, unsigned int depth)
      : data_(&data), feature_index_(feature_index),
        available_feature_(available_feature), depth_(depth), result_(-1) {}

  Node() {}
};


#endif