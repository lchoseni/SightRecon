#ifndef OPTIMIZER_H
#define OPTIMIZER_H

#include "frame.h"
#include "map.h"
namespace srecon {
class Optimizer {
 private:
  /* data */
 public:
  Optimizer(/* args */);
  ~Optimizer();
  bool optimize(Map::Ptr map);
};

}  // namespace srecon

#endif