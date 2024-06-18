/*
 * @Author: 姚潘涛
 * @Date: 2024-06-14 10:03:17
 * @LastEditors: 姚潘涛
 * @LastEditTime: 2024-06-18 11:56:16
 * @Description:
 *
 * Copyright (c) 2024 by pandaman, All Rights Reserved.
 */
#ifndef CALIB_GROUND_EXTRACT_IMAGE_H
#define CALIB_GROUND_EXTRACT_IMAGE_H
#include "core/camera_calib.h"
// #include <filesystem>
// #include <iostream>
// #include <opencv2/opencv.hpp>
// #include <unistd.h>

namespace xict_calib {
    void ExtractImage(const std::vector<int>& camera_ids,
                      std::filesystem::path video_path_base,
                      float new_size_factor, float balance);
}
#endif   // CALIB_GROUND_EXTRACT_IMAGE_H
