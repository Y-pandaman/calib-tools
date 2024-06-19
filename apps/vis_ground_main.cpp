/*
 * @Author: 姚潘涛
 * @Date: 2024-06-14 10:03:17
 * @LastEditors: 姚潘涛
 * @LastEditTime: 2024-06-18 15:56:41
 * @Description:
 *
 * Copyright (c) 2024 by pandaman, All Rights Reserved.
 */
// 绘制相机和标定版位姿

#include "core/viz_painter.h"
#include "utils/utils.h"
#include <filesystem>
#include <unistd.h>

int main(int argc, char** argv) {
    int optc;
    std::string path_base;
    int camera_id = 1;
    while ((optc = getopt(argc, argv, "d:n:")) != -1) {
        switch (optc) {
        case 'd':
            path_base = std::string(optarg);
            break;
        case 'n':
            camera_id = (atoi(optarg));
            break;
        default:
            break;
        }
    }

    cv::FileStorage fs;
    if (path_base.empty()) {
        std::cout << "poses_dir is empty" << std::endl;
        return 0;
    }
    for (int i = 0;; i++) {
        int axis_count = 0;
        cv::viz::Viz3d my_window("vis_ground");
        my_window.spinOnce();
        my_window.showWidget("worldAxes", cv::viz::WCoordinateSystem());
        std::string poses_file = path_base + "/camera_calib/poses/" +
                                 std::to_string(camera_id) + "-" +
                                 std::to_string(i) + ".png.yaml";
        if (!fs.open(poses_file, cv::FileStorage::READ)) {
            printf("open %s error\n", poses_file.c_str());
            break;
        }
        int poses_num;
        fs["poses_num"] >> poses_num;
        for (int j = 0; j < poses_num; j++) {
            cv::Mat pose_mat;

            fs["pose_" + std::to_string(j)] >> pose_mat;
            Eigen::Matrix4f inverted_matrix;
            xict_calib::Utils::InvertYInR(pose_mat, inverted_matrix);
            xict_calib::VizPainter::DrawAxis(inverted_matrix, my_window,
                                             std::to_string(axis_count), 1);
            axis_count++;
        }
        fs.release();

        std::string ground_yaml = path_base + "/ground.yaml";
        if (!fs.open(ground_yaml, cv::FileStorage::READ)) {
            printf("open %s error\n", ground_yaml.c_str());
            break;
        }
        cv::Mat ground_matrix;
        fs["ground_matrix"] >> ground_matrix;
        fs.release();

        xict_calib::VizPainter::DrawAxis(ground_matrix, my_window,
                                         std::to_string(axis_count++), 2);

        my_window.spin();
    }
}