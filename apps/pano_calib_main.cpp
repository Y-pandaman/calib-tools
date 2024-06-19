/*
 * @Author: 姚潘涛
 * @Date: 2024-06-14 10:03:17
 * @LastEditors: 姚潘涛
 * @LastEditTime: 2024-06-18 20:58:20
 * @Description:
 *
 * Copyright (c) 2024 by pandaman, All Rights Reserved.
 */
#include "core/pano_calib.h"
#include "stage/extract_image.h"
#include <filesystem>
#include <getopt.h>
#include <iostream>
#include <vector>

int main(int argc, char** argv) {
    // 用于存储命令行选项的解析结果
    int optc = 0;
    // 视频文件的基础路径
    std::string video_path_base;
    // 存储摄像头ID的集合
    std::vector<int> camera_ids;
    // 标志位，用于判断是否获取所有图像
    bool flag_get_all_image = false;
    // 新尺寸因子，用于调整图像大小
    float new_size_factor = 1.2;
    // 平衡因子，用于调节图像处理中的权重
    float balance = 0.3;

    // 遍历命令行参数，解析选项
    while ((optc = getopt(argc, argv, "d:n:f:b:")) != -1) {
        switch (optc) {
        case 'd':
            // 设置视频文件的基础路径
            video_path_base = std::string(optarg);
            printf("video_path_base: %s\n", video_path_base);
            break;
        case 'n':
            // 添加摄像头ID到集合中
            camera_ids.push_back(atoi(optarg));
            printf("camera_id: %d\n", atoi(optarg));
            break;
        case 'f':
            // 设置新尺寸因子
            new_size_factor = atof(optarg);
            printf("new_size_factor: %f\n", new_size_factor);
            break;
        case 'b':
            // 设置平衡因子
            balance = atof(optarg);
            printf("balance: %f\n", balance);
            break;
        default:
            break;
        }
    }

    // 调用函数，从摄像头视频中提取图像
    xict_calib::ExtractImage(camera_ids, video_path_base, new_size_factor,
                             balance);
    // 调用函数，计算全景图像的 Homography
    xict_calib::CalibPanoHomography(camera_ids, video_path_base, 1280, 720);
    // 调用函数，测试全景图像校准结果
    xict_calib::TestPanoCalibResult(camera_ids, video_path_base, 1280, 720);
    return 0;
}