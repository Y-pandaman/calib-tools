/*
 * @Author: 姚潘涛
 * @Date: 2024-06-14 10:03:17
 * @LastEditors: 姚潘涛
 * @LastEditTime: 2024-06-18 19:39:34
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
    /**
     * @brief 计算两个图像的差异分数。
     *
     * 该函数通过比较两个图像的每个像素值的差异，并计算这些差异值的平均值，来评估两个图像的相似度。
     * 差异分数越小，表示两个图像越相似；差异分数越大，表示两个图像越不相似。
     *
     * @param img_1 第一个图像，是一个CV_8UC1类型的OpenCV Mat对象。
     * @param img_2 第二个图像，是一个CV_8UC1类型的OpenCV Mat对象。
     * @return 返回两个图像的差异分数，这是一个浮点数，范围在0到255之间。
     */
    float DiffScore(const cv::Mat_<uchar>& img_1, const cv::Mat_<uchar>& img_2);
    /**
     * @brief 从视频中提取不模糊的图像
     *
     * @param images_dir 存储提取图像的目录
     * @param video_path 视频文件的路径
     * @param camera_idx 摄像头的索引，用于图像命名
     * @param noblur_image_idx_vec 存储不模糊图像索引的向量
     * @param motion_threshold 运动阈值，用于判断图像是否模糊
     * @param local_mini_stride 局部最小帧间隔，用于确定存储图像的频率
     * @param image_base_count 图像基础计数，用于图像命名
     * @return 返回更新后的不模糊图像索引向量
     */
    std::vector<int> ExtractNoBlurImagesFromVideo(
        std::string images_dir, std::string video_path, int camera_idx = -1,
        std::vector<int> noblur_image_idx_vec = std::vector<int>(),
        float motion_threshold = 8.0f, int local_mini_stride = 5,
        int image_base_count = 0);
    /**
     * @brief 从视频中提取图像并保存到指定目录。
     *
     * @param images_dir 保存提取图像的目录路径。
     * @param video_path 视频文件的路径。
     * @param camera_idx 摄像头索引，用于在文件名中区分不同摄像头的数据。
     *
     * 此函数首先创建指定的图像保存目录，然后打开视频文件，逐帧读取视频中的彩色图像，
     * 并将每帧图像保存为PNG格式的文件，文件名包含帧编号和摄像头索引。
     */
    void ExtractImagesFromVideo(std::string images_dir, std::string video_path,
                                int camera_idx);
    /**
     * @brief FishEyeCameraCalib 大致功能是进行鱼眼相机的校准。
     *
     * @param images_dir 图像目录，包含用于校准的图像。
     * @param camera_idx 相机索引，用于区分不同的相机。
     */
    void FisheyeCameraCalib(std::string images_dir, int camera_idx);
    /**
     * @brief 视频去畸变处理函数。
     *
     * 功能描述：该函数从指定的YAML文件中读取相机内参信息，计算用于校正畸变的新摄像机矩阵，
     * 初始化映射以进行畸变矫正，并将此映射应用于视频中的每一帧以进行畸变校正处理。
     * 最终输出校正后的视频及对应的掩模图像（首帧）至指定路径。
     *
     * @param video_path 输入视频文件的路径。
     * @param camera_idx
     * 相机的索引编号，用于构建读取/保存相关参数文件的名称。
     * @param new_size_factor 新视频大小相对于原视频的缩放因子。
     * @param balance 用于计算新摄像机矩阵的平衡参数，影响校正区域的大小。
     */
    void UndistortVideo(std::string video_path, int camera_idx,
                        float new_size_factor, float balance);
    /**
     * @brief 对每个视图进行校准，转换到空中视图
     *
     * 本函数的目的是通过检测图像中的AprilTag标记，来建立地面视图和空中视图之间的映射关系。
     * 它首先初始化AprilTag检测器，然后循环处理每对对应的地面视图和空中视图图像。在每对图像中，
     * 它检测AprilTag的位置，并尝试匹配来自两幅图像的标签。一旦找到足够的匹配点，它使用这些点
     * 来计算一个 homography（ homography
     * 是一种用于平面场景中图像之间映射的几何变换矩阵）。 最后，它将这个
     * homography 存储起来，用于将地面视图转换为空中视图。
     *
     * @param images_dir 地面视图图像的目录
     * @param airview_images_dir 空中视图图像的目录
     * @param camera_idx 相机索引，用于标识特定的相机
     * @param target_w_ 空中视图的目标宽度
     * @param target_h_ 空中视图的目标高度
     */
    void CalibEachViewToAirView(std::string images_dir,
                                std::string airview_images_dir, int camera_idx,
                                int target_w_, int target_h_);
    /**
     * @brief 测试图像拼接过程中的融合算法
     *
     * 该函数通过给定一组图像、对应的掩模以及变换矩阵，实现图像的拼接融合。
     * 具体步骤包括：
     * 1. 初始化融合图像和权重图像。
     * 2. 对于每张图像，根据变换矩阵将其变形并融合到融合图像中。
     * 3. 计算每像素的加权平均值，得到最终的融合图像。
     *
     * @param images 输入的图像数组。
     * @param masks 对应的图像掩模数组，用于指定每像素的融合权重。
     * @param Hs 图像之间的变换矩阵数组。
     * @param tgt_w_ 目标图像的宽度。
     * @param tgt_h_ 目标图像的高度。
     */
    void TestStitching(std::vector<cv::Mat> images, std::vector<cv::Mat> masks,
                       std::vector<cv::Mat> Hs, int tgt_w_, int tgt_h_);
}   // namespace xict_calib
#endif   // _CAMERA_CALIB_
