#ifndef DATASET_H
#define DATASET_H

#include <string>

namespace srecon {
class Dataset {
   protected:
    std::string dataset_dir;
    virtual bool readGroundTruth() = 0;
};

}  // namespace srecon

#endif