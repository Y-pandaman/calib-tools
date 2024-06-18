/*
 * @Author: 姚潘涛
 * @Date: 2024-06-14 10:03:17
 * @LastEditors: 姚潘涛
 * @LastEditTime: 2024-06-18 18:34:28
 * @Description:
 *
 * Copyright (c) 2024 by pandaman, All Rights Reserved.
 */
#pragma once

#ifndef _CAMERA_CALIB_
#define _CAMERA_CALIB_

#include "stage/april_tag.h"
#include "utils/math_utils.h"
#include <boost/filesystem.hpp>
// #include <cassert>
// #include <filesystem>
#include <iostream>
// #include <opencv2/core/types_c.h>
// #include <opencv2/imgcodecs/legacy/constants_c.h>
// #include <opencv2/imgproc/types_c.h>
#include <opencv2/opencv.hpp>
// #include <signal.h>
// #include <stack>
// #include <stdio.h>
#include <string>
#include <sys/stat.h>   // mkdir
#include <unistd.h>     // STDERR_FILENO
#include <vector>

namespace xict_calib {
    std::vector<int> ExtractNoBlurImagesFromVideo(
        std::string images_dir, std::string video_path, int camera_idx = -1,
        std::vector<int> noblur_image_idx_vec = std::vector<int>(),
        float motion_threshold = 8.0f, int local_mini_stride = 5,
        int image_base_count = 0);
    void ExtractImagesFromVideo(std::string images_dir, std::string video_path,
                                int camera_idx);
    void FisheyeCameraCalib(std::string images_dir, int camera_idx);
    void UndistortVideo(std::string video_path, int camera_idx,
                        float new_size_factor, float balance);
    void CalibEachViewToAirView(std::string images_dir,
                                std::string airview_images_dir, int camera_idx,
                                int target_w_, int target_h_);

    void TestStitching(std::vector<cv::Mat> images, std::vector<cv::Mat> masks,
                       std::vector<cv::Mat> Hs, int tgt_w_, int tgt_h_);
}   // namespace xict_calib
#endif   // _CAMERA_CALIB_
