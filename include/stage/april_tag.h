/*
 * @Author: 姚潘涛
 * @Date: 2024-06-14 10:03:17
 * @LastEditors: 姚潘涛
 * @LastEditTime: 2024-06-18 19:38:55
 * @Description:
 *
 * Copyright (c) 2024 by pandaman, All Rights Reserved.
 */
#pragma once

#include "utils/math_utils.h"
#include <Eigen/Eigen>
#include <opencv2/opencv.hpp>
#include <visp3/core/vpPixelMeterConversion.h>
#include <visp3/core/vpPoint.h>
#include <visp3/core/vpXmlParserCamera.h>
#include <visp3/detection/vpDetectorAprilTag.h>
#include <visp3/gui/vpDisplayGDI.h>
#include <visp3/gui/vpDisplayOpenCV.h>
#include <visp3/gui/vpDisplayX.h>
#include <visp3/io/vpImageIo.h>
#include <visp3/vision/vpPose.h>

namespace xict_calib {
    class AprilTagDetector {
    public:
        AprilTagDetector()
            : is_init_(false), tag_size_(0.0), quad_decimate_(0.0),
              n_threads_(0), display_tag_(false), color_id_(-1), thickness_(0),
              z_aligned_(false) { }

        ~AprilTagDetector();
        /**
         * @brief 初始化AprilTag检测器。
         *
         * 此函数配置了AprilTag检测器的参数，包括标签大小、相机参数、检测器的细化解码、边缘细化和姿
         * seedu估计方法。 它还设置了是否显示检测到的标签以及标签的颜色和厚度。
         *
         * @param tag_size 标签的边长，单位通常为米。
         * @param fx 相机的焦距x，单位为像素。
         * @param fy 相机的焦距y，单位为像素。
         * @param cx 相机的图像中心x，单位为像素。
         * @param cy 相机的图像中心y，单位为像素。
         */
        void Init(float tag_size, float fx, float fy, float cx, float cy);
        /**
         * @brief 初始化AprilTag检测器。
         * 这个函数配置了AprilTag检测器的各种参数，包括所使用的标签家族、位姿估计方法、多线程设置等。
         * 它还设置了标签检测的细化解码、边缘细化和位姿估计的开关，并根据需要调整了标签的显示设置。
         * 在配置完成后，初始化检测器对象。
         *
         * @throws vpException 如果初始化过程中发生错误，会抛出异常。
         */
        void Init();
        /**
         * @brief 计算位姿
         *
         * 本函数用于根据给定的特征点及其图像坐标，以及相机参数，计算物体相对于相机的位姿。
         * 如果初始化标志为真，则尝试使用Dementhon和Lagrange两种方法初始化位姿，并选择残差较小的方法。
         * 最后，通过虚拟视图与物体位姿的计算，更新位姿矩阵。
         *
         * @param point 特征点的3D坐标集合
         * @param ip 特征点在图像中的2D坐标集合
         * @param cam 相机参数
         * @param init 是否进行初始化计算
         * @param cMo 相机到物体的齐次变换矩阵
         */
        void ComputePose(std::vector<vpPoint>& point,
                         const std::vector<vpImagePoint>& ip,
                         const vpCameraParameters& cam, bool init,
                         vpHomogeneousMatrix& cMo);
        /**
         * @brief 检测图像中的AprilTag。
         *
         * 此函数接收一个OpenCV矩阵图像作为输入，尝试检测图像中的AprilTag。它首先将OpenCV的图像格式转换为vpImage格式，
         * 然后使用内部的AprilTag检测器进行检测。如果检测过程中抛出异常，则捕获异常并打印错误消息。
         *
         * @param image 输入的OpenCV矩阵图像。
         */
        void Detect(cv::Mat image);
        /**
         * @brief 检测图像中的AprilTag标签，并提取每个标签的角点。
         *
         * @param points 一个映射，将标签ID映射到其四个角点的2D坐标集合。
         * @param image 输入的OpenCV图像。
         */
        void Detect(std::map<int, std::vector<float2> >& points, cv::Mat image);
        /**
         * @brief 获取所有AprilTag的位姿
         *
         * 此函数遍历内部存储的每个AprilTag的相机到标记物的变换矩阵，并将其转换为Eigen的4x4位姿矩阵，
         * 然后将这些位姿矩阵添加到提供的引用参数中。
         *
         * @param poses
         * 一个引用参数，用于存储所有AprilTag的位姿矩阵，每个位姿是一个4x4的变换矩阵。
         */
        void GetPoses(std::vector<Eigen::Matrix4f>& poses);
        /**
         * @brief 获取AprilTag检测结果的图像点集合
         *
         * 此函数从AprilTag检测器中提取每个检测到的标签的顶点坐标，并将这些坐标存储在一个二维向量中。
         * 每个子向量代表一个标签的四个顶点，便于后续处理和使用。
         *
         * @param points 一个二维向量，用于存储每个标签的图像点坐标。
         */
        void GetImagePoints(std::vector<std::vector<float2> >& points);
        /**
         * @brief 获取AprilTag在世界坐标系中的四个角点
         *
         * 此函数计算并存储了AprilTag在世界坐标系中的四个角点。这些点以float2的格式存储，
         * 其中float2是包含x和y坐标的结构体。函数通过计算标签大小的一半，然后构建四个点的坐标，
         * 这四个点分别位于标签的左下、右下、右上和左上角。
         *
         * @param points
         * 一个float2类型的向量，用于存储计算得到的AprilTag在世界坐标系中的四个角点。
         */
        void GetWorldPoints(std::vector<float2>& pointsvpDetectorAprilTag);
        /**
         * @brief 获取检测到的AprilTag标签的ID列表
         *
         * 该函数通过调用内部的detector对象获取所有检测到的AprilTag标签的ID列表。
         * 它不接受任何参数，返回一个包含标签ID的整数向量。这个函数使得外部可以方便地访问
         * detector的检测结果，无需直接操作detector对象。
         *
         * @return std::vector<int> 包含所有检测到的标签ID的向量
         */
        std::vector<int> GetTagsID();

    public:
        bool is_init_;
        int tag_width_ = 2, tag_height_ = 2;
        double tag_size_;
        float quad_decimate_;
        int n_threads_;
        bool display_tag_;
        int color_id_;
        unsigned int thickness_;
        bool z_aligned_;
        vpDetectorAprilTag* detector_;
        vpCameraParameters* cam_;
        std::vector<vpHomogeneousMatrix> cMo_vec_;
    };
}   // namespace xict_calib