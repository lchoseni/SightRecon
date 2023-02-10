#pragma once

#include "../common_include.h"
#include "../mono_track.h"

namespace srecon {
class ImageProcessor {
   public:
    MonoTrack::Ptr monoTrack;
    ImageDataset::Ptr imageDataset;
    void process();
};
}  // namespace srecon