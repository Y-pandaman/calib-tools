/*
 * @Author: 姚潘涛
 * @Date: 2024-06-14 10:03:17
 * @LastEditors: 姚潘涛
 * @LastEditTime: 2024-06-18 09:49:18
 * @Description:
 *
 * Copyright (c) 2024 by pandaman, All Rights Reserved.
 */
#include "stage/tag_poses_set.h"
namespace xict_calib {
    bool TagPosesSet::ReadFromFileStorage(const cv::FileStorage& fs) {
        // 从文件存储中读取标签的总数
        fs["poses_num"] >> poses_num;

        // 循环读取每个标签的姿势矩阵和ID
        for (int x = 0; x < poses_num; x++) {
            // 防止越界读取，尽管逻辑上这不应该发生
            if (x >= poses_num)
                break;

            // 读取当前标签的姿势矩阵
            cv::Mat pose_matrix;
            fs["pose_" + std::to_string(x)] >> pose_matrix;

            // 读取当前标签的ID
            int id;
            fs["tag_" + std::to_string(x)] >> id;

            // 将读取的姿势矩阵和ID添加到列表中
            pose_matrix_list.push_back(pose_matrix);
            tags_id.push_back(id);
        }

        // 操作成功完成，返回true
        return true;
    }
}   // namespace xict_calib