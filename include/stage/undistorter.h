#ifndef CALIB_GROUND_UNDISTORTER_H
#define CALIB_GROUND_UNDISTORTER_H

#include <cstring>
#include <opencv2/opencv.hpp>

namespace xict_calib {
    class Undistorter {
    public:
        bool LoadCameraIntrin(const std::string& fs_path);
        bool GetMapForRemapping(float new_size_factor = 1.4,
                                float balance         = 1.0);
        bool UndistortImage(cv::Mat input_image, cv::Mat& output_image);
        bool GetMask(cv::Mat& out_mask);

    private:
        cv::Mat K, D;
        cv::Size input_image_size;
        cv::Mat new_K, new_D;
        cv::Size new_image_size;
        cv::Mat map1, map2;   // map for remapping
        cv::Mat mask;
        bool mask_generated = false;
        bool map_inited     = false;
    };
}   // namespace xict_calib
#endif   // CALIB_GROUND_UNDISTORTER_H
