/*
 * @Author: 姚潘涛
 * @Date: 2024-06-14 10:03:17
 * @LastEditors: 姚潘涛
 * @LastEditTime: 2024-06-18 10:01:56
 * @Description:
 *
 * Copyright (c) 2024 by pandaman, All Rights Reserved.
 */
#ifndef CALIB_GROUND_TAGPOSESSET_H
#define CALIB_GROUND_TAGPOSESSET_H

#include <cstdio>
#include <opencv2/opencv.hpp>
#include <vector>

namespace xict_calib {
    class TagPosesSet {
    public:
        int poses_num = 0;
        std::vector<cv::Mat> pose_matrix_list;
        std::vector<int> tags_id;

        bool ReadFromFileStorage(const cv::FileStorage& fs);
    };
}   // namespace xict_calib
#endif   // CALIB_GROUND_TAGPOSESSET_H
