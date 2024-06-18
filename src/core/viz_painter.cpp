/*
 * @Author: 姚潘涛
 * @Date: 2024-06-14 10:03:17
 * @LastEditors: 姚潘涛
 * @LastEditTime: 2024-06-18 09:49:04
 * @Description:
 *
 * Copyright (c) 2024 by pandaman, All Rights Reserved.
 */

#include "core/viz_painter.h"

namespace xict_calib {
    /**
     * 在给定的窗口中绘制坐标轴。
     * @param matrix 4x4的变换矩阵，用于确定坐标轴的位置和方向。
     * @param window Viz3d窗口对象，用于显示绘制的坐标轴。
     * @param name 坐标轴的名称，用于标识不同的坐标轴系统。
     * @param length 坐标轴的长度。
     * @param width 坐标轴线的宽度。
     */
    void VizPainter::DrawAxis(const Eigen::Matrix4f& matrix,
                              cv::viz::Viz3d& window, const std::string& name,
                              float length, float width) {
        // 计算坐标轴原点的位置
        cv::Point3f root(matrix.col(3).x(), matrix.col(3).y(),
                         matrix.col(3).z());

        // 绘制x轴，颜色为红色
        cv::viz::WLine x_line(root,
                              cv::Point3f(matrix.col(0).x(), matrix.col(0).y(),
                                          matrix.col(0).z()) *
                                      length +
                                  root,
                              cv::viz::Color::red());
        // 绘制y轴，颜色为绿色
        cv::viz::WLine y_line(root,
                              cv::Point3f(matrix.col(1).x(), matrix.col(1).y(),
                                          matrix.col(1).z()) *
                                      length +
                                  root,
                              cv::viz::Color::green());
        // 绘制z轴，颜色为蓝色
        cv::viz::WLine z_line(root,
                              cv::Point3f(matrix.col(2).x(), matrix.col(2).y(),
                                          matrix.col(2).z()) *
                                      length +
                                  root,
                              cv::viz::Color::blue());

        // 设置坐标轴线的宽度
        x_line.setRenderingProperty(cv::viz::LINE_WIDTH, width);
        y_line.setRenderingProperty(cv::viz::LINE_WIDTH, width);
        z_line.setRenderingProperty(cv::viz::LINE_WIDTH, width);

        // 在窗口中显示坐标轴
        window.showWidget("x_line_" + name, x_line);
        window.showWidget("y_line_" + name, y_line);
        window.showWidget("z_line_" + name, z_line);
    }

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
    void VizPainter::DrawAxis(const cv::Mat& matrix, cv::viz::Viz3d& window,
                              const std::string& name, float length,
                              float width) {
        Eigen::Matrix4f eigen_matrix;
        cv::cv2eigen(matrix, eigen_matrix);
        DrawAxis(eigen_matrix, window, name, length, width);
    }
}   // namespace xict_calib