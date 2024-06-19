/*
 * @Author: 姚潘涛
 * @Date: 2024-06-14 10:03:17
 * @LastEditors: 姚潘涛
 * @LastEditTime: 2024-06-18 19:39:25
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
    /**
     * @brief 根据给定的相机索引序列，对每个相机视图进行矫正并拼接成全景图
     *
     * @param camera_idx_vec 相机索引序列，代表需要矫正的每个相机的编号
     * @param root_dir 所有相机图像和矫正结果的根目录
     * @param tgt_w 目标全景图的宽度
     * @param tgt_h 目标全景图的高度
     * @return true 总是返回true，表示函数执行成功
     */
    bool CalibPanoHomography(const std::vector<int>& camera_idx_vec,
                             std::string root_dir, int tgt_w = 1280,
                             int tgt_h = 720);
    /**
     * @brief 测试全景相机标定结果。
     *
     * 此函数从指定路径加载各相机的图像和掩模，调整它们的尺寸，并执行拼接测试。
     * 它读取每台相机的单应性矩阵，并将所有数据传递给TestStitching函数以进行测试。
     *
     * @param camera_idx_vec 包含相机索引的向量。
     * @param root_dir 存放校准结果的根目录路径。
     * @param tgt_width 图像调整的目标宽度。
     * @param tgt_height 图像调整的目标高度。
     * @return 始终返回false（留作将来使用）。
     */
    bool TestPanoCalibResult(const std::vector<int>& camera_idx_vec,
                             const std::string& root_dir, int tgt_width,
                             int tgt_height);
}   // namespace xict_calib
#endif   // CALIB_GROUND_PANO_CALIB_H
