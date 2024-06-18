/*
 * @Author: 姚潘涛
 * @Date: 2024-06-14 10:03:17
 * @LastEditors: 姚潘涛
 * @LastEditTime: 2024-06-18 10:21:39
 * @Description:
 *
 * Copyright (c) 2024 by pandaman, All Rights Reserved.
 */
#include "utils/utils.h"

namespace xict_calib {
    /**
     * @brief 将左手坐标系转换为右手坐标系
     *
     * 本函数接受一个OpenCV的Mat矩阵作为输入，该矩阵代表了一个在左手坐标系中的变换。
     * 函数将这个变换转换为右手坐标系，并将结果存储在一个Eigen的4x4矩阵中。
     * 左手坐标系和右手坐标系的主要区别在于Y轴的方向。在左手坐标系中，Y轴向上，
     * 而在右手坐标系中，Y轴向下。因此，本函数通过翻转Y轴来实现坐标系的转换。
     *
     * @param input 输入的OpenCV矩阵，代表左手坐标系中的变换
     * @param output 输出的Eigen矩阵，代表右手坐标系中的变换
     */
    void utils::LeftHandToRightHand(const cv::Mat& input,
                                    Eigen::Matrix4f& output) {
        // 初始化一个4x4的单位矩阵，用于后续的坐标转换
        Eigen::Matrix4f eigen_input = Eigen::Matrix4f::Identity();
        // 将OpenCV的矩阵转换为Eigen的矩阵格式
        cv::cv2eigen(input, eigen_input);

        // 定义一个用于翻转Y轴的矩阵
        Eigen::Matrix3f Sy;
        Sy << 1, 0, 0, 0, -1, 0, 0, 0, 1;

        // 提取输入矩阵中的旋转部分
        Eigen::Matrix3f R = eigen_input.topLeftCorner(3, 3);
        // 应用Y轴翻转，将左手坐标系的旋转转换为右手坐标系
        R = Sy * R * Sy;

        // 计算新的X、Y、Z轴向量
        Eigen::Vector3f x, y, z;
        x = R.topLeftCorner(3, 1);
        y = R.topLeftCorner(3, 2).topRightCorner(3, 1);
        z = x.cross(y);

        // 提取输入矩阵中的平移部分，并应用Y轴翻转
        Eigen::Vector3f T = eigen_input.topRightCorner(3, 1);
        T                 = Sy * T;

        // 将旋转部分和平移部分组合成输出矩阵
        output.topLeftCorner(3, 3) = R;
        Eigen::Vector4f Th(T.x(), T.y(), T.z(), 1);
        output.topRightCorner(4, 1) = Th;
    }

    /**
     * @brief 从齐次变换矩阵中提取旋转角轴和平移量
     *
     * 本函数从给定的4x4齐次变换矩阵中提取出旋转部分的角轴表示和平移量。
     * 具体来说，它首先检查旋转矩阵是否是单位矩阵，然后将旋转部分转换为角轴表示，
     * 并输出原始旋转矩阵和从角轴表示还原的旋转矩阵，最后提取出平移量。
     *
     * @param eigen_input 4x4齐次变换矩阵
     * @param angle_axis 旋转的角轴表示
     * @param trans 平移量
     */
    void utils::GetAngleAxisAndTrans(const Eigen::Matrix4f& eigen_input,
                                     Eigen::AngleAxisf& angle_axis,
                                     Eigen::Vector3f& trans) {
        // 提取变换矩阵中的旋转部分
        Eigen::Matrix3f rotation_matrix = eigen_input.topLeftCorner(3, 3);
        // 检查旋转矩阵是否是单位矩阵
        if (rotation_matrix.isUnitary()) {
            printf("rotation_matrix is Unitary\n");
        }
        // 将旋转矩阵转换为角轴表示
        angle_axis = rotation_matrix;
        // 从角轴表示还原旋转矩阵，用于验证转换的准确性
        Eigen::Matrix3f back_rotation = angle_axis.matrix();
        // 输出原始旋转矩阵和还原后的旋转矩阵
        std::cout << "origin_rotation:\n" << rotation_matrix << std::endl;
        std::cout << "rotation_from_aa:\n" << back_rotation << std::endl;

        // 提取变换矩阵中的平移部分
        trans = eigen_input.topRightCorner(3, 1);
    }

    /**
     * @brief 在齐次变换矩阵中翻转Y轴
     *
     * 该函数接收一个OpenCV的4x4矩阵作为输入，并将其转换为Eigen矩阵格式。然后，通过乘以一个特定的变换矩阵，
     * 实现了Y轴的翻转。翻转后的矩阵被存储在提供的Eigen矩阵输出参数中。
     * 这种Y轴翻转对于处理计算机视觉中的图像变换特别有用，例如在坐标系转换中。
     *
     * @param input 输入的4x4变换矩阵，以OpenCV的Mat格式存储。
     * @param output
     * 输出的4x4变换矩阵，经过Y轴翻转后，以Eigen的Matrix4f格式存储。
     */
    void utils::InvertYInR(const cv::Mat& input, Eigen::Matrix4f& output) {
        // 初始化一个单位矩阵，用于后续的矩阵乘法操作。
        Eigen::Matrix4f eigen_input = Eigen::Matrix4f::Identity();
        // 将OpenCV的矩阵转换为Eigen格式。
        cv::cv2eigen(input, eigen_input);

        // 定义一个翻转Y轴的变换矩阵。
        Eigen::Matrix4f tp;
        tp << 1, 0, 0, 0, 0, -1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1;

        // 将输入矩阵与翻转矩阵相乘，得到输出矩阵。
        output = eigen_input * tp;
    }
}   // namespace xict_calib