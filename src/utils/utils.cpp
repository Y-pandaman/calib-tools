/*
 * @Author: 姚潘涛
 * @Date: 2024-06-14 10:03:17
 * @LastEditors: 姚潘涛
 * @LastEditTime: 2024-06-18 19:36:59
 * @Description:
 *
 * Copyright (c) 2024 by pandaman, All Rights Reserved.
 */
#include "utils/utils.h"

namespace xict_calib {
    void Utils::LeftHandToRightHand(const cv::Mat& input,
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

    void Utils::GetAngleAxisAndTrans(const Eigen::Matrix4f& eigen_input,
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

    void Utils::GetAngleAxisAndTrans(const cv::Mat& input,
                                     Eigen::AngleAxisf& angle_axis,
                                     Eigen::Vector3f& trans) {
        // 将OpenCV的旋转矩阵转换为Eigen格式
        Eigen::Matrix4f eigen_input;
        cv::cv2eigen(input, eigen_input);

        // 调用另一个函数进行旋角轴和平移向量的计算
        GetAngleAxisAndTrans(eigen_input, angle_axis, trans);
    }

    void Utils::InvertYInR(const cv::Mat& input, Eigen::Matrix4f& output) {
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

    void Utils::GetQuaternionAndTrans(const cv::Mat& input,
                                      Eigen::Quaternionf& quaternion,
                                      Eigen::Vector3f& trans) {
        // 将OpenCV矩阵转换为Eigen矩阵
        Eigen::Matrix4f eigen_input;
        cv::cv2eigen(input, eigen_input);

        // 从Eigen矩阵中提取旋转矩阵
        Eigen::Matrix3f rotation_matrix = eigen_input.topLeftCorner(3, 3);

        // 通过旋转矩阵构造四元数
        quaternion = Eigen::Quaternionf(rotation_matrix);

        // 从Eigen矩阵中提取平移向量
        trans = eigen_input.topRightCorner(3, 1);
    }

    Eigen::Vector3f
    Utils::WeightAngleAxis(const Eigen::AngleAxisf& angle_axis) {
        // 获取角度轴对象的旋转轴
        const Eigen::Vector3f& vec = angle_axis.axis();
        // 获取角度轴对象的旋转角度
        float angle = angle_axis.angle();
        // 计算并返回角度权重向量
        return angle * vec;
    }
}   // namespace xict_calib