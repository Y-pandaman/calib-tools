/*
 * @Author: 姚潘涛
 * @Date: 2024-06-14 10:03:17
 * @LastEditors: 姚潘涛
 * @LastEditTime: 2024-06-18 19:38:38
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
        /**
         * @brief 从OpenCV的文件存储对象中读取标签姿势数据。
         * @param fs 文件存储对象，包含标签姿势的数据。
         * @return 总是返回true，表示读取操作始终成功。
         *
         * 此函数从给定的文件存储对象中读取标签的姿势数据，包括每个标签的矩阵和ID。
         * 它首先读取标签的总数，然后循环读取每个标签的姿势矩阵和ID，并将它们添加到相应的列表中。
         */
        bool ReadFromFileStorage(const cv::FileStorage& fs);
    };
}   // namespace xict_calib
#endif   // CALIB_GROUND_TAGPOSESSET_H
