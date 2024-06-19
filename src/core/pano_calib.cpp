/*
 * @Author: 姚潘涛
 * @Date: 2024-06-14 10:03:17
 * @LastEditors: 姚潘涛
 * @LastEditTime: 2024-06-19 09:32:02
 * @Description:
 *
 * Copyright (c) 2024 by pandaman, All Rights Reserved.
 */
#include "core/pano_calib.h"
namespace xict_calib {

    bool CalibPanoHomography(const std::vector<int>& camera_idx_vec,
                             std::string root_dir, int tgt_w, int tgt_h) {
        // 遍历相机索引序列，对每个相机视图进行矫正
        for (int i = 0; i < camera_idx_vec.size(); ++i) {
            // 构建相机校准图像和空中视图图像的目录路径
            std::string images_dir         = root_dir + "/camera_calib/";
            std::string airview_images_dir = root_dir + "/airview/";

            // 打印当前正在处理的相机索引，用于调试和跟踪进度
            printf("CalibEachViewToAirView %d ....\n", camera_idx_vec[i]);
            // 调用函数对每个相机视图进行矫正并拼接到空中视图
            xict_calib::CalibEachViewToAirView(images_dir, airview_images_dir,
                                               camera_idx_vec[i], tgt_w, tgt_h);
            // 打印当前相机视图矫正完成的消息
            printf("CalibEachViewToAirView %d done \n", camera_idx_vec[i]);
        }
        // 函数始终返回true，表示处理过程成功
        return true;
    }

    bool TestPanoCalibResult(const std::vector<int>& camera_idx_vec,
                             const std::string& root_dir, int tgt_width,
                             int tgt_height) {
        // 用于存储加载的图像、掩模和单应性矩阵的向量。
        std::vector<cv::Mat> images;
        std::vector<cv::Mat> masks;
        std::vector<cv::Mat> Hs;

        char name[512];
        // 遍历相机索引来加载图像和掩模。
        for (int i = 0; i < camera_idx_vec.size(); ++i) {
            // 构建相机校准图像的目录路径。
            std::string images_dir = root_dir + "/camera_calib/";

            // 生成彩色图像的文件名并打印。
            sprintf(name, (images_dir + "/%d-%d.png").c_str(),
                    camera_idx_vec[i], 0);
            // 加载彩色图像。
            cv::Mat color_image = cv::imread(name);

            // 生成掩模图像的文件名并加载。
            sprintf(name, (images_dir + "/../%d_mask.png").c_str(),
                    camera_idx_vec[i]);
            cv::Mat mask_image = cv::imread(name, 0);

            // 调整掩模和彩色图像至目标尺寸。
            cv::resize(
                mask_image, mask_image,
                cv::Size(mask_image.cols * 2 / 3, mask_image.rows * 2 / 3));
            cv::resize(
                color_image, color_image,
                cv::Size(color_image.cols * 2 / 3, color_image.rows * 2 / 3));

            // 将调整尺寸后的图像和掩模添加到各自的向量中。
            images.emplace_back(color_image.clone());
            masks.emplace_back(mask_image.clone());

            // 初始化单应性矩阵并从文件中读取。
            cv::Mat H = cv::Mat::eye(3, 3, CV_64F);
            cv::FileStorage fs(images_dir + "/../camera_" +
                                   std::to_string(camera_idx_vec[i]) +
                                   "_homography.yaml",
                               cv::FileStorage::READ);
            fs["H"] >> H;
            fs.release();
            // 将单应性矩阵转换为32位浮点型。
            H.convertTo(H, CV_32F);
            // 将单应性矩阵添加到向量中。
            Hs.emplace_back(H);
        }
        // 调用函数执行拼接测试。
        xict_calib::TestStitching(images, masks, Hs, tgt_width, tgt_height);
        // 当前始终返回false。
        return false;
    }
}   // namespace xict_calib