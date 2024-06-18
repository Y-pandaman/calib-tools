/*
 * @Author: 姚潘涛
 * @Date: 2024-06-14 10:03:17
 * @LastEditors: 姚潘涛
 * @LastEditTime: 2024-06-18 10:04:17
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
        static void DrawAxis(const Eigen::Matrix4f& matrix,
                             cv::viz::Viz3d& window, const std::string& name,
                             float length = 200, float width = 4);
        static void DrawAxis(const cv::Mat& matrix, cv::viz::Viz3d& window,
                             const std::string& name, float length = 200,
                             float width = 4);
    };
}   // namespace xict_calib

#endif   // CALIB_GROUND_VIZPAINTER_H
