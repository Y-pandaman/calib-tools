/*
 * @Author: 姚潘涛
 * @Date: 2024-06-14 10:03:17
 * @LastEditors: 姚潘涛
 * @LastEditTime: 2024-06-18 19:37:59
 * @Description:
 *
 * Copyright (c) 2024 by pandaman, All Rights Reserved.
 */
#ifndef CALIB_GROUND_UTILS_H
#define CALIB_GROUND_UTILS_H

#include <Eigen/Eigen>
#include <opencv2/core/eigen.hpp>
#include <opencv2/opencv.hpp>

namespace xict_calib {
    class Utils {
    public:
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
        static void GetAngleAxisAndTrans(const Eigen::Matrix4f& eigen_input,
                                         Eigen::AngleAxisf& angle_axis,
                                         Eigen::Vector3f& trans);

        /**
         * @brief 将OpenCV的旋转矩阵转换为Eigen的旋角轴和平移向量形式。
         * 此函数首先将OpenCV的旋转矩阵转换为Eigen数据格式，然后调用另一个函数进行转换。
         * 这种转换在跨库操作或需要特定旋转表示时很有用。
         *
         * @param input          输入的OpenCV旋转矩阵，格式为3x3。
         * @param angle_axis     输出的旋角轴表示，形式为Eigen::AngleAxisf。
         * @param trans          输出的平移向量，形式为Eigen::Vector3f。
         */
        static void GetAngleAxisAndTrans(const cv::Mat& input,
                                         Eigen::AngleAxisf& angle_axis,
                                         Eigen::Vector3f& trans);
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
        static void LeftHandToRightHand(const cv::Mat& input,
                                        Eigen::Matrix4f& output);
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
        static void InvertYInR(const cv::Mat& input, Eigen::Matrix4f& output);

        /**
         * @brief 从输入的OpenCV矩阵中提取四元数和平移向量。
         *
         * 该函数旨在处理包含旋转和平移的4x4变换矩阵。它将OpenCV矩阵转换为Eigen矩阵，
         * 并从中提取旋转矩阵和平移向量，然后将旋转矩阵转换为四元数形式。
         *
         * @param input 输入的OpenCV 4x4变换矩阵，格式为CV_32F。
         * @param quaternion 提取的四元数，用于表示旋转。
         * @param trans 提取的平移向量。
         */
        static void GetQuaternionAndTrans(const cv::Mat& input,
                                          Eigen::Quaternionf& quaternion,
                                          Eigen::Vector3f& trans);

        /**
         * @brief 计算角度轴权重
         *
         * 该函数根据给定的角度轴（AngleAxis）对象，计算出对应的角度权重向量。
         * 角度权重向量是角度与旋转轴的乘积，用于表示特定旋转角度对旋转轴的影响。
         *
         * @param angle_axis Eigen::AngleAxisf对象，表示一个旋转
         * @return Eigen::Vector3f 返回角度权重向量，即角度与旋转轴的乘积
         */
        static Eigen::Vector3f
        WeightAngleAxis(const Eigen::AngleAxisf& angle_axis);
    };
}   // namespace xict_calib

#endif   // CALIB_GROUND_UTILS_H
