#include "srecon/dataset/image_dataset.h"

#include <boost/format.hpp>
#include <iostream>

#include "srecon/config.h"
#include "unistd.h"

namespace srecon {

ImageDataset::ImageDataset()
    : img_index(0) {
}

ImageDataset::~ImageDataset() {}
}  // namespace srecon
