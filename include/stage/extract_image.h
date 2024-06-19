/*
 * @Author: 姚潘涛
 * @Date: 2024-06-14 10:03:17
 * @LastEditors: 姚潘涛
 * @LastEditTime: 2024-06-18 19:38:44
 * @Description:
 *
 * Copyright (c) 2024 by pandaman, All Rights Reserved.
 */
#ifndef CALIB_GROUND_EXTRACT_IMAGE_H
#define CALIB_GROUND_EXTRACT_IMAGE_H
#include "core/camera_calib.h"

namespace xict_calib {
    /**
     * @brief 提取指定摄像头ID的视频图像，并进行去畸变处理。
     *
     * @param camera_ids 摄像头ID的集合。
     * @param video_path_base 视频文件和图像存储的基础路径。
     * @param new_size_factor 新尺寸因子，用于调整图像大小。
     * @param balance 平衡因子，用于调整去畸变过程中的优化平衡。
     */
    void ExtractImage(const std::vector<int>& camera_ids,
                      const std::string& video_path_base,
                      float new_size_factor, float balance);
}
#endif   // CALIB_GROUND_EXTRACT_IMAGE_H
