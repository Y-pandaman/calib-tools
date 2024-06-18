/*
 * @Author: 姚潘涛
 * @Date: 2024-06-14 10:03:17
 * @LastEditors: 姚潘涛
 * @LastEditTime: 2024-06-18 16:49:15
 * @Description:
 *
 * Copyright (c) 2024 by pandaman, All Rights Reserved.
 */
#include "core/camera_calib.h"
#include <iostream>
#include <unistd.h>
#include <sstream>

int main(int argc, char** argv) {
    // 初始化程序运行所需的参数
    std::string root_dir;              // 数据根目录
    std::vector<int> camera_idx_set;   // 相机索引集合
    int optc;                          // getopt返回的选项字符
    float new_size_factor = 1.0;       // 视频缩放因子
    float balance = 1.0;               // 平衡因子，用于优化图像处理效果

    // 处理命令行参数
    while ((optc = getopt(argc, argv, "d:n:f:b:")) != -1) {
        switch (optc) {
        case 'd':
            root_dir = std::string(optarg);
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
        std::stringstream ss;
        ss << root_dir << "/camera_calib_" << camera_idx << ".avi";
        std::string calib_video_path = ss.str();
        
        ss.str(""); ss.clear();
        ss << root_dir << "/camera_video_" << camera_idx << ".avi";
        std::string video_path = ss.str();
        
        ss.str(""); ss.clear();
        ss << root_dir << "/camera_calib_" << camera_idx;
        std::string images_dir = ss.str();

        // 打印相关路径信息
        printf("calib_video_path: %s,\n video_path: %s\n, images_dir: %s\n",
               calib_video_path.c_str(), video_path.c_str(), images_dir.c_str());

        // 从视频中提取无模糊图像
        xict_calib::ExtractNoBlurImagesFromVideo(images_dir, video_path);
        // 进行相机标定
        xict_calib::FisheyeCameraCalib(images_dir, camera_idx);
        // 对视频进行畸变矫正
        std::cout << "foo1"<<std::endl;
        xict_calib::UndistortVideo(video_path, camera_idx, new_size_factor,
                                   balance);
    }
}