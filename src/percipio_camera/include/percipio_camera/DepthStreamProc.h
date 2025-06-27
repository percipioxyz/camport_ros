#pragma once

#include <vector>
#include "TYParameter.h"

struct DepthSpkFilterPara {
    int max_speckle_size;
    int max_speckle_diff;
};

int TYDepthSpeckleFilter(TY_IMAGE_DATA& depth , const DepthSpkFilterPara& param);
int TYDepthEnhenceFilter(const std::vector<TY_IMAGE_DATA>& depth_images, TY_IMAGE_DATA& output);