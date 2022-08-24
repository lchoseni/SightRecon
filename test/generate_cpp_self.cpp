#include <iostream>
#include <sstream>
#include <string>
#include <vector>

#include "generate_cpp_self.h"

using namespace std;

namespace g_cpp {
 vector<pair<int, int>> pixels = {
    make_pair(-3, 0), make_pair(-3, 1),  make_pair(-2, 2),  make_pair(-1, 3),
    make_pair(0, 3),  make_pair(1, 3),   make_pair(2, 2),   make_pair(3, 1),
    make_pair(3, 0),  make_pair(3, -1),  make_pair(2, -2),  make_pair(1, -3),
    make_pair(0, -3), make_pair(-1, -3), make_pair(-2, -2), make_pair(-3, -1)};
}

void printIndents(int indents, stringstream &ss) {
  for (int i = 0; i < indents; i++) {
    ss << " ";
  }
}

void printIfStart(int indents, stringstream &ss, string condition, bool first,
                  bool last) {
  printIndents(indents, ss);
  if (first) {
    ss << "if(" << condition << "){\n";
  }
  else if (last) {
    ss << "else{\n";
  }
  if (!last && !first) {
    ss << "else if(" << condition << "){\n";
  }
}

void printIfEnd(int indents, stringstream &ss) {
  printIndents(indents, ss);
  ss << "}\n";
}

void printNode(Node &node, stringstream &ss) {
  // According to the node depth, print indents ahead of it.
  // then check its result to find if there are children available.

  printIndents(node.depth_, ss);
  if (node.result_ == 0){
    printIndents(node.depth_, ss);
    ss << "return false;\n";
  }
  else if (node.result_ == 1 || node.result_ == 2){
    printIndents(node.depth_, ss);
    ss << "return true;\n";
  }
  else {
    // For the first node, print if
    // For the node between the first and last node, print else if
    // For the last node, print else
    string condition, cmp;
    int count = 0, size = node.children_.size();
    for (auto &feature_value_to_node : node.children_) {
      condition = "image.at<int>(row + " + to_string(g_cpp::pixels[
                  feature_value_to_node.second.feature_index_].first) +
                  ", col + " +
                  to_string(g_cpp::pixels[feature_value_to_node.second.feature_index_].second) + ")";
      if (feature_value_to_node.first == 1)
        condition += "< lower_threshold";
      else if (feature_value_to_node.first == 2)
        condition += "> upper_threshold";
      if (feature_value_to_node.first == 0) {
        condition = condition + "<= upper_threshold && " + condition +
                    ">= lower_threshold";
      }
      printIfStart(node.depth_, ss, condition, count == 0, count == size - 1);
      printNode(feature_value_to_node.second, ss);
      printIfEnd(node.depth_, ss);
      count++;
    }
  }
}

void generate_cpp(Node &root) {

  // Print include details
  // function name
  // thresholds
  // tree
  stringstream ss;

  ss << "#include<iostream>\n"
        "#include<vector>\n"
        "#include <opencv2/core/core.hpp>\n\n"
        "using namespace std;\n\n"
        "int radius = 3;\n"
        "int threshold = 25;\n\n"
        "bool is_fast_corner(cv::Mat &image, int row, int col){\n"
        "int center = image.at<int>(row, col);\n"
        "int upper_threshold = center + threshold;\n"
        "int lower_threshold = center - threshold;\n";
  // Start traversing the whole tree
  printNode(root, ss);
  ss << "return false;\n}\n" << endl;
  cout << ss.str() << endl;
}
