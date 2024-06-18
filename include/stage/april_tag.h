/*
 * @Author: 姚潘涛
 * @Date: 2024-06-14 10:03:17
 * @LastEditors: 姚潘涛
 * @LastEditTime: 2024-06-18 11:57:58
 * @Description:
 *
 * Copyright (c) 2024 by pandaman, All Rights Reserved.
 */
#pragma once

#include "utils/math_utils.h"
#include <Eigen/Eigen>
#include <opencv2/opencv.hpp>
// #include <visp3/core/vpHomogeneousMatrix.h>
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
        void Init(float tag_size, float fx, float fy, float cx, float cy);
        void Init();

        void Detect(cv::Mat image);
        void Detect(std::map<int, std::vector<float2> >& points, cv::Mat image);
        void GetPoses(std::vector<Eigen::Matrix4f>& poses);
        void GetImagePoints(std::vector<std::vector<float2> >& points);
        void GetWorldPoints(std::vector<float2>& pointsvpDetectorAprilTag);
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