/*
 * @Author: 姚潘涛
 * @Date: 2024-06-14 10:03:17
 * @LastEditors: 姚潘涛
 * @LastEditTime: 2024-06-18 19:39:13
 * @Description:
 *
 * Copyright (c) 2024 by pandaman, All Rights Reserved.
 */
#ifndef CALIB_GROUND_VIZPAINTER_H
#define CALIB_GROUND_VIZPAINTER_H

#include <Eigen/Eigen>
#include <opencv2/core/eigen.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/viz/vizcore.hpp>

namespace xict_calib {
    class VizPainter {
    public:
        /**
         * @brief 在给定的窗口中绘制坐标轴。
         * @param matrix 4x4的变换矩阵，用于确定坐标轴的位置和方向。
         * @param window Viz3d窗口对象，用于显示绘制的坐标轴。
         * @param name 坐标轴的名称，用于标识不同的坐标轴系统。
         * @param length 坐标轴的长度。
         * @param width 坐标轴线的宽度。
         */
        static void DrawAxis(const Eigen::Matrix4f& matrix,
                             cv::viz::Viz3d& window, const std::string& name,
                             float length = 200, float width = 4);
        /**
         * @brief 绘制坐标轴
         * @param matrix 输入的OpenCV矩阵，表示一个4x4的变换矩阵
         * @param window OpenCV viz模块的窗口对象，用于显示绘制结果
         * @param name 坐标轴在窗口中的名称，用于标识和操作特定的坐标轴
         * @param length 坐标轴的长度
         * @param width 坐标轴线的宽度
         *
         * 此函数首先将OpenCV矩阵转换为Eigen矩阵，因为OpenCV
         * viz模块的函数使用Eigen矩阵作为参数。
         * 转换完成后，调用另一个重载的drawAxis函数来进行实际的绘制操作。
         * 这里的函数主要是为了提供一个接口，以兼容使用OpenCV矩阵的场景。
         */
        static void DrawAxis(const cv::Mat& matrix, cv::viz::Viz3d& window,
                             const std::string& name, float length = 200,
                             float width = 4);
    };
}   // namespace xict_calib

#endif   // CALIB_GROUND_VIZPAINTER_H
