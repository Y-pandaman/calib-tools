/*
 * @Author: 姚潘涛
 * @Date: 2024-06-14 10:03:17
 * @LastEditors: 姚潘涛
 * @LastEditTime: 2024-06-18 10:03:46
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
    class utils {
    public:
        static void GetAngleAxisAndTrans(const Eigen::Matrix4f& eigen_input,
                                         Eigen::AngleAxisf& angle_axis,
                                         Eigen::Vector3f& trans);

        static void GetAngleAxisAndTrans(const cv::Mat& input,
                                         Eigen::AngleAxisf& angle_axis,
                                         Eigen::Vector3f& trans) {
            Eigen::Matrix4f eigen_input;
            cv::cv2eigen(input, eigen_input);

            GetAngleAxisAndTrans(eigen_input, angle_axis, trans);
        }

        static void LeftHandToRightHand(const cv::Mat& input,
                                        Eigen::Matrix4f& output);

        static void InvertYInR(const cv::Mat& input, Eigen::Matrix4f& output);

        static void GetQuaternionAndTrans(const cv::Mat& input,
                                          Eigen::Quaternionf& quaternion,
                                          Eigen::Vector3f& trans) {
            Eigen::Matrix4f eigen_input;
            cv::cv2eigen(input, eigen_input);
            Eigen::Matrix3f rotation_matrix = eigen_input.topLeftCorner(3, 3);
            quaternion = Eigen::Quaternionf(rotation_matrix);
            trans      = eigen_input.topRightCorner(3, 1);
        }

        static Eigen::Vector3f
        WeightAngleAxis(const Eigen::AngleAxisf& angle_axis) {
            const Eigen::Vector3f& vec = angle_axis.axis();
            float angle                = angle_axis.angle();
            return angle * vec;
        }
    };
}   // namespace xict_calib

#endif   // CALIB_GROUND_UTILS_H
