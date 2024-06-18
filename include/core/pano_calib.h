/*
 * @Author: 姚潘涛
 * @Date: 2024-06-14 10:03:17
 * @LastEditors: 姚潘涛
 * @LastEditTime: 2024-06-18 11:09:03
 * @Description:
 *
 * Copyright (c) 2024 by pandaman, All Rights Reserved.
 */
#ifndef CALIB_GROUND_PANO_CALIB_H
#define CALIB_GROUND_PANO_CALIB_H
#include "core/camera_calib.h"
#include <filesystem>
#include <iostream>
#include <opencv2/opencv.hpp>
#include <opencv2/rgbd/large_kinfu.hpp>

namespace xict_calib {
    bool CalibPanoHomography(const std::vector<int>& camera_idx_vec,
                             std::string root_dir, int tgt_w = 1280,
                             int tgt_h = 720);
    bool TestPanoCalibResult(const std::vector<int>& camera_idx_vec,
                             const std::string& root_dir, int tgt_width,
                             int tgt_height);
}   // namespace xict_calib
#endif   // CALIB_GROUND_PANO_CALIB_H
