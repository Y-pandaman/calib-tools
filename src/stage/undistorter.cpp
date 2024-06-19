/*
 * @Author: 姚潘涛
 * @Date: 2024-06-14 10:03:17
 * @LastEditors: 姚潘涛
 * @LastEditTime: 2024-06-18 19:30:37
 * @Description:
 *
 * Copyright (c) 2024 by pandaman, All Rights Reserved.
 */
#include "stage/undistorter.h"

namespace xict_calib {

    bool Undistorter::LoadCameraIntrin(const std::string& fs_path) {
        cv::FileStorage fs;
        // 尝试打开相机内参文件
        if (!fs.open(fs_path, cv::FileStorage::Mode::READ)) {
            printf("cannot open fs file %s\n", fs_path.c_str());
            return false;
        }
        // 从文件中读取内参矩阵K、畸变系数D和图像尺寸
        fs["K"] >> K;
        fs["D"] >> D;
        fs["image_size"] >> input_image_size;

        // 根据当前的内参初始化重映射所需的地图
        GetMapForRemapping(1.0, 0.0);
        // 设置标志，表示内参已加载
        map_inited = true;
        return true;
    }

    bool Undistorter::GetMapForRemapping(float new_size_factor, float balance) {
        // 检查相机矩阵K和畸变系数D是否为空
        if (K.empty() || D.empty()) {
            printf("K & D empty, cannot get map for remapping\n");
            return false;
        }

        // 初始化一个单位矩阵
        cv::Mat eye_mat = cv::Mat::eye(3, 3, CV_32F);

        // 计算新图像的尺寸
        this->new_image_size =
            cv::Size(this->input_image_size.width * new_size_factor,
                     this->input_image_size.height * new_size_factor);

        // 估计用于无扭曲和校正的新相机矩阵
        cv::fisheye::estimateNewCameraMatrixForUndistortRectify(
            K, D, input_image_size, eye_mat, new_K, balance, new_image_size);

        // 初始化用于无扭曲和校正的映射矩阵
        cv::fisheye::initUndistortRectifyMap(K, D, eye_mat, new_K,
                                             new_image_size, CV_16SC2,
                                             this->map1, this->map2);
        return true;
    }

    bool Undistorter::UndistortImage(cv::Mat input_image,
                                     cv::Mat& output_image) {
        // 检查映射矩阵是否已初始化，如果没有，则尝试初始化
        if (!map_inited) {
            bool flag = GetMapForRemapping();
            // 如果初始化失败，返回false
            if (!flag)
                return false;
        }
        // 使用映射矩阵对图像进行去畸变处理
        cv::remap(input_image, output_image, map1, map2, cv::INTER_LINEAR);
        return true;
    }

    bool Undistorter::GetMask(cv::Mat& out_mask) {
        // 检查是否已经初始化了映射，如果没有，则函数直接返回false。
        if (!map_inited)
            return false;

        // 如果掩码图像尚未生成，则生成一个新的掩码图像。
        if (!mask_generated) {
            // 创建一个全为255（白色）的掩码图像，大小与输入图像相同。
            mask = cv::Mat::ones(input_image_size, CV_8UC1) * 255;
            // 使用预计算的映射函数对掩码图像进行重映射，以匹配畸变矫正后的图像的几何形状。
            cv::remap(mask, mask, map1, map2, cv::INTER_LINEAR);
            // 设置标志，表示掩码图像已生成。
            mask_generated = true;
        }

        // 将生成的掩码图像复制到输出参数中。
        out_mask = mask.clone();
        // 函数成功完成，返回true。
        return true;
    }
}   // namespace xict_calib