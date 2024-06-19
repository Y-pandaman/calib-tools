/*
 * @Author: 姚潘涛
 * @Date: 2024-06-14 10:03:17
 * @LastEditors: 姚潘涛
 * @LastEditTime: 2024-06-18 19:38:22
 * @Description:
 *
 * Copyright (c) 2024 by pandaman, All Rights Reserved.
 */
#ifndef CALIB_GROUND_UNDISTORTER_H
#define CALIB_GROUND_UNDISTORTER_H

#include <cstring>
#include <opencv2/opencv.hpp>

namespace xict_calib {
    class Undistorter {
    public:
        /**
         * @brief 加载相机内参
         * @param fs_path 相机内参文件的路径
         * @return 是否成功加载内参
         *
         * 本函数用于从指定的文件中加载相机的内参矩阵K、畸变系数D以及图像尺寸。
         * 如果文件无法打开，将打印错误信息并返回false。成功加载后，会生成重映射所需的地图，
         * 并设置标志表示内参已加载。
         */
        bool LoadCameraIntrin(const std::string& fs_path);
        /**
         * @brief 获取用于重新映射的映射矩阵
         * @param new_size_factor 新图像尺寸相对于原始图像尺寸的因子
         * @param balance 平衡参数，用于调整优化的目标
         * @return 如果成功生成映射矩阵，则返回true；如果K或D为空，则返回false
         *
         * 本函数用于计算从原始扭曲图像到新尺寸下无扭曲图像的映射矩阵。
         * 它首先检查相机矩阵K和畸变系数D是否为空，然后根据新的尺寸因子和平衡参数，
         * 计算新的相机矩阵和用于无扭曲图像重新映射的映射矩阵。
         */
        bool GetMapForRemapping(float new_size_factor = 1.4,
                                float balance         = 1.0);
        /**
         * @brief 对图像进行去畸变处理
         *
         * 本函数旨在去除输入图像中的畸变，使用之前初始化的映射矩阵。如果映射矩阵未初始化，
         * 则尝试通过调用getMapForRemapping方法来初始化。只有在成功初始化映射矩阵后，才会
         * 对图像进行去畸变处理。
         *
         * @param input_image 输入图像，即待处理的畸变图像。
         * @param output_image
         * 输出图像，即处理后的去畸变图像。这是引用参数，函数将结果直接写入此参数。
         * @return true 如果成功执行了去畸变操作。
         * @return false
         * 如果映射矩阵初始化失败，或者由于其他原因导致去畸变操作未能执行。
         */
        bool UndistortImage(cv::Mat input_image, cv::Mat& output_image);
        /**
         * @brief 获取畸变矫正的掩码图像。
         *
         * 该函数从内部状态中获取或生成一个畸变矫正所用的掩码图像，并将其复制到输出参数out_mask中。
         * 掩码图像是一幅单通道的8位无符号整数图像，其中的像素值为255表示对应位置在矫正后应该被保留，
         * 像素值为0表示对应位置在矫正后应该被忽略。
         *
         * @param out_mask 输出参数，接收到生成的掩码图像的一个副本。
         * @return 如果掩码图像已初始化并成功获取，则返回true；否则返回false。
         */
        bool GetMask(cv::Mat& out_mask);

    private:
        cv::Mat K, D;
        cv::Size input_image_size;
        cv::Mat new_K, new_D;
        cv::Size new_image_size;
        cv::Mat map1, map2;   // map for remapping
        cv::Mat mask;
        bool mask_generated = false;
        bool map_inited     = false;
    };
}   // namespace xict_calib
#endif   // CALIB_GROUND_UNDISTORTER_H
