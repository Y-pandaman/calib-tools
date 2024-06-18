/*
 * @Author: 姚潘涛
 * @Date: 2024-06-14 10:03:17
 * @LastEditors: 姚潘涛
 * @LastEditTime: 2024-06-18 15:30:26
 * @Description:
 *
 * Copyright (c) 2024 by pandaman, All Rights Reserved.
 */
#include "core/viz_painter.h"
#include <Eigen/Eigen>
#include <filesystem>
#include <iostream>
#include <opencv2/core/eigen.hpp>
#include <opencv2/viz.hpp>
#include <unistd.h>

void drawPlane(const cv::Mat& mat_matrix, cv::viz::Viz3d& window,
               const std::string& name) {
    Eigen::Matrix4d matrix;
    cv::cv2eigen(mat_matrix, matrix);
    cv::Point3d root(matrix.col(3).x(), matrix.col(3).y(), matrix.col(3).z());
}

int main(int argc, char** argv) {
    int optc;
    std::string camera_extrin_fs_path, ground_fs_path;
    int front_camera_id, back_camera_id;
    while ((optc = getopt(argc, argv, "f:b:d:g:")) != -1) {
        switch (optc) {
        case 'f':
            // front camera id
            front_camera_id = atoi(optarg);
            break;
        case 'b':
            // back camera id
            back_camera_id = atoi(optarg);
            break;
        case 'd':
            camera_extrin_fs_path = std::string(optarg);
            break;
        case 'g':
            ground_fs_path = std::string(optarg);
            break;
        default:
            break;
        }
    }

    cv::FileStorage fs;
    if (!fs.open(camera_extrin_fs_path, cv::FileStorage::READ)) {
        printf("cannot open %s\n", camera_extrin_fs_path.c_str());
    }
    cv::Mat matrix_back2front;
    std::string matrix_back2front_name("matrix" +
                                       std::to_string(back_camera_id) + "to" +
                                       std::to_string(front_camera_id));
    std::cout << "matrix_back2front_name: " << matrix_back2front_name
              << std::endl;
    fs[matrix_back2front_name] >> matrix_back2front;
    std::cout << "matrix_back2front:\n" << matrix_back2front << std::endl;
    cv::Mat matrix_front2back = matrix_back2front.inv();

    std::cout << "matrix_front2back:\n" << matrix_front2back << std::endl;

    if (!fs.open(ground_fs_path, cv::FileStorage::READ)) {
        printf("cannot open %s\n", ground_fs_path.c_str());
    }

    cv::Mat ground_matrix;
    fs["ground_matrix"] >> ground_matrix;
    std::cout << "ground_matrix:\n" << ground_matrix << std::endl;

    std::string window_name("vis_cam");
    /// Create a window
    std::cout << "viz3d mywindow\n";
    cv::viz::Viz3d myWindow(window_name);
    std::cout << "viz3d mywindow done\n";
    myWindow.spinOnce();
    std::cout << "spinOnce done\n";
    xict_calib::VizPainter::DrawAxis(ground_matrix, myWindow, "ground_axis", 3);
    xict_calib::VizPainter::DrawAxis(matrix_front2back, myWindow,
                                     "camera_back_axis", 1);

    myWindow.showWidget("worldAxes", cv::viz::WCoordinateSystem());

    myWindow.spin();
    return 0;
}
