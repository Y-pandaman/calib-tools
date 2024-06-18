/*
 * @Author: 姚潘涛
 * @Date: 2024-06-14 10:03:17
 * @LastEditors: 姚潘涛
 * @LastEditTime: 2024-06-18 10:21:19
 * @Description:
 *
 * Copyright (c) 2024 by pandaman, All Rights Reserved.
 */
#include "stage/undistorter.h"

namespace xict_calib {
    /**
     * 加载相机内参
     * @param fs_path 相机内参文件的路径
     * @return 是否成功加载内参
     *
     * 本函数用于从指定的文件中加载相机的内参矩阵K、畸变系数D以及图像尺寸。
     * 如果文件无法打开，将打印错误信息并返回false。成功加载后，会生成重映射所需的地图，
     * 并设置标志表示内参已加载。
     */
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
    /**
     * 获取用于重新映射的映射矩阵
     * @param new_size_factor 新图像尺寸相对于原始图像尺寸的因子
     * @param balance 平衡参数，用于调整优化的目标
     * @return 如果成功生成映射矩阵，则返回true；如果K或D为空，则返回false
     *
     * 本函数用于计算从原始扭曲图像到新尺寸下无扭曲图像的映射矩阵。
     * 它首先检查相机矩阵K和畸变系数D是否为空，然后根据新的尺寸因子和平衡参数，
     * 计算新的相机矩阵和用于无扭曲图像重新映射的映射矩阵。
     */
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

    /**
     * 获取畸变矫正的掩码图像。
     *
     * 该函数从内部状态中获取或生成一个畸变矫正所用的掩码图像，并将其复制到输出参数out_mask中。
     * 掩码图像是一幅单通道的8位无符号整数图像，其中的像素值为255表示对应位置在矫正后应该被保留，
     * 像素值为0表示对应位置在矫正后应该被忽略。
     *
     * @param out_mask 输出参数，接收到生成的掩码图像的一个副本。
     * @return 如果掩码图像已初始化并成功获取，则返回true；否则返回false。
     */
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
}