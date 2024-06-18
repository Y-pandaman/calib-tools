/*
 * @Author: 姚潘涛
 * @Date: 2024-06-14 10:03:17
 * @LastEditors: 姚潘涛
 * @LastEditTime: 2024-06-18 11:08:02
 * @Description:
 *
 * Copyright (c) 2024 by pandaman, All Rights Reserved.
 */
#include "core/camera_calib.h"
#include <iostream>
#include <unistd.h>
int main(int argc, char** argv) {
    // 初始化程序运行所需的参数
    std::filesystem::path root_dir;    // 数据根目录
    std::vector<int> camera_idx_set;   // 相机索引集合
    int optc;                          // getopt返回的选项字符
    float new_size_factor = 1.0;       // 视频缩放因子
    float balance = 1.0;   // 平衡因子，用于优化图像处理效果

    // 处理命令行参数
    while ((optc = getopt(argc, argv, "d:n:f:b:")) != -1) {
        switch (optc) {
        case 'd':
            root_dir = std::filesystem::path(optarg);
            break;
        case 'n':
            camera_idx_set.push_back(atoi(optarg));
            break;
        case 'f':
            new_size_factor = atof(optarg);
            printf("new_size_factor: %f\n", new_size_factor);
            break;
        case 'b':
            balance = atof(optarg);
            printf("balance: %f\n", balance);
            break;
        default:
            break;
        }
    }

    // 检查是否指定了数据根目录
    if (root_dir.empty()) {
        printf("no input path\n");
        return 0;
    }

    // 遍历每个相机索引，进行相机标定和视频处理
    for (int i = 0; i < camera_idx_set.size(); ++i) {
        int camera_idx = camera_idx_set[i];
        // 构建相机标定视频和原始视频的路径
        std::filesystem::path calib_video_path(root_dir);
        calib_video_path.append("camera_calib_" + std::to_string(camera_idx) +
                                ".avi");
        std::filesystem::path video_path(root_dir);
        video_path.append("camera_video_" + std::to_string(camera_idx) +
                          ".avi");
        // 构建相机标定图像保存的目录路径
        std::filesystem::path images_dir(root_dir);
        images_dir.append("camera_calib_" + std::to_string(camera_idx));

        // 打印相关路径信息
        printf("calib_video_path: %s,\n video_path: %s\n, images_dir: %s\n",
               calib_video_path.c_str(), video_path.c_str(),
               images_dir.c_str());

        // 从视频中提取无模糊图像
        xict_calib::ExtractNoBlurImagesFromVideo(images_dir, video_path);
        // 进行相机标定
        xict_calib::FisheyeCameraCalib(images_dir, camera_idx);
        // 对视频进行畸变矫正
        xict_calib::UndistortVideo(video_path, camera_idx, new_size_factor,
                                   balance);
    }
}
