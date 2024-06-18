/*
 * @Author: 姚潘涛
 * @Date: 2024-06-14 10:03:17
 * @LastEditors: 姚潘涛
 * @LastEditTime: 2024-06-18 16:29:19
 * @Description:
 * 
 * Copyright (c) 2024 by pandaman, All Rights Reserved. 
 */
#include "stage/extract_image.h"
#include <iostream>
#include <vector>
#include <string>
#include <cstdio>
#include <boost/filesystem.hpp>

namespace xict_calib {
    /**
     * 提取指定摄像头ID的视频图像，并进行去畸变处理。
     *
     * @param camera_ids 摄像头ID的集合。
     * @param video_path_base 视频文件和图像存储的基础路径。
     * @param new_size_factor 新尺寸因子，用于调整图像大小。
     * @param balance 平衡因子，用于调整去畸变过程中的优化平衡。
     */
    void ExtractImage(const std::vector<int>& camera_ids,
                      const std::string& video_path_base,
                      float new_size_factor, float balance) {
        // 打印基础视频路径
        std::cout << video_path_base << std::endl;

        std::vector<int> noblur_image_idx_vec;
        // 遍历摄像头ID列表
        for (int camera_id : camera_ids) {
            // 构建视频文件名和图像目录名
            std::string video_filename = video_path_base + "/camera_video_" + std::to_string(camera_id) + ".avi";
            std::string images_dir = video_path_base + "/camera_video_" + std::to_string(camera_id) + "/";

            // 打印当前处理的摄像头ID
            std::cout << "camera_id = " << camera_id << std::endl;

            // 构建去畸变视频路径
            std::string undistort_video_path = video_path_base + "/" + std::to_string(camera_id) + "_undistort.avi";

            // 检查视频文件是否存在
            if (!boost::filesystem::exists(video_filename)) {
                std::cout << "no such video: " << video_filename << std::endl;
                continue;
            }
            // 如果图像目录已存在，则跳过当前摄像头的处理
            if (boost::filesystem::exists(images_dir)) {
                std::cout << "images_dir has existed: " << images_dir << std::endl;
                continue;
            }
            // 对视频进行去畸变处理
            xict_calib::UndistortVideo(video_filename, camera_id, new_size_factor, balance);

            // 构建无模糊图像输出目录
            std::string noblur_images_dir = video_path_base + "/camera_calib";
            // 创建无模糊图像输出目录
            boost::filesystem::create_directories(noblur_images_dir);
            // 从去畸变视频中提取无模糊图像
            noblur_image_idx_vec = xict_calib::ExtractNoBlurImagesFromVideo(noblur_images_dir, undistort_video_path, camera_id, noblur_image_idx_vec);
        }
    }
}   // namespace xict_calib
