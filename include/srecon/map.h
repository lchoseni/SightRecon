//
// Created by yeren on 1/25/22.
//

#ifndef SRECON_MAP_H
#define SRECON_MAP_H

#include "common_include.h"
#include "frame.h"
#include "feature.h"
#include "map_point.h"

namespace srecon {

class Map {
 public:
  typedef std::shared_ptr<Map> Ptr;
  std::vector<Frame::Ptr> frames;

};

}  // namespace srecon

#endif  // SRECON_MAP_H
