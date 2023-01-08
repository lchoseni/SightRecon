#ifndef QUERY_TREE_H
#define QUERY_TREE_H

struct query_tree
{
    static const int is_a_feature = -1;
    static const int is_not_a_feature = -2;

    // Indices to next nodes
    int positive;
    int negative;
    int neither;

    // Which test to perform
    int pixel;
    int num_of_tests;
};

#endif