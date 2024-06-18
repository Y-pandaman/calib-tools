#include "stage/tag_poses_set.h"
#include "utils/utils.h"
#include <Eigen/Eigen>
#include <filesystem>
#include <opencv2/core/eigen.hpp>
#include <opencv2/opencv.hpp>
#include <unistd.h>

int main(int argc, char** argv) {
    // 初始化变量
    std::filesystem::path video_dir;   // 视频目录路径
    std::vector<int> valid_num;        // 有效计数
    std::vector<int> useful_tags;      // 有用的标签
    std::vector<int> camera_ids;       // 摄像机ID列表
    std::vector<Eigen::Vector3f> mean_trans_list,
        mean_weighted_axis_list;   // 平均平移和加权轴列表

    // 处理命令行参数
    int optc;
    while ((optc = getopt(argc, argv, "t:d:n:")) != -1) {
        switch (optc) {
        case 'd':   // 视频目录选项
            video_dir = std::filesystem::path(optarg);
            break;
        case 'n':   // 摄像机ID选项
            camera_ids.push_back(atoi(optarg));
            break;
        case 't':   // 标签选项
            useful_tags.push_back(atoi(optarg));
            printf("useful tag id = %d\n", atoi(optarg));
            valid_num.push_back(0);                  // 初始化有效计数
            mean_trans_list.emplace_back(0, 0, 0);   // 初始化平均平移
            mean_weighted_axis_list.emplace_back(0, 0, 0);   // 初始化加权轴
            break;
        default:
            break;
        }
    }

    // 打印有用标签的数量
    printf("useful_tag size: %d\n", useful_tags.size());

    // 遍历每个相机ID
    for (int camera_id : camera_ids) {
        // 构建相机姿态文件的基础路径
        std::filesystem::path pose_file_base(video_dir);
        pose_file_base.append("camera_calib");
        pose_file_base.append("poses");

        // 遍历姿态文件列表
        std::filesystem::directory_iterator pose_file_list(pose_file_base);
        for (auto& filepath : pose_file_list) {
            cv::FileStorage fs;
            // 打开文件存储
            if (!fs.open(filepath.path(), cv::FileStorage::READ)) {
                printf("cannot open file %s\n", filepath.path().c_str());
                exit(0);
            }

            // 读取标签姿态数据
            xict_calib::TagPosesSet tag_poses_set;
            tag_poses_set.ReadFromFileStorage(fs);
            fs.release();

            // 打印姿态数量
            printf("poses_num: %d\n", tag_poses_set.poses_num);

            // 遍历每个姿态
            for (int i = 0; i < tag_poses_set.poses_num; i++) {
                int tag_id    = tag_poses_set.tags_id[i];
                int useful_id = -1;

                // 查找有用标签的索引
                for (int k = 0; k < useful_tags.size(); k++) {
                    if (tag_id == useful_tags[k]) {
                        useful_id = k;
                        break;
                    }
                }

                // 如果当前标签不是有用标签，则跳过
                if (useful_id == -1)
                    continue;

                // 输出左手法则矩阵
                std::cout << "left_pose_matrix:\n"
                          << tag_poses_set.pose_matrix_list[i] << std::endl;

                // 计算右手坐标系下的矩阵
                Eigen::Matrix4f right_hand_matrix;
                xict_calib::utils::InvertYInR(tag_poses_set.pose_matrix_list[i],
                                              right_hand_matrix);

                // 计算旋转角轴和平移量
                Eigen::Vector3f trans;
                Eigen::AngleAxisf angle_axis;
                xict_calib::utils::GetAngleAxisAndTrans(right_hand_matrix,
                                                        angle_axis, trans);
                printf("tag_id: %d\n", tag_id);
                std::cout << "this trans: \n" << trans << std::endl;

                // 加权平均角轴
                Eigen::Vector3f tp_vec =
                    xict_calib::utils::WeightAngleAxis(angle_axis);
                mean_weighted_axis_list[useful_id] += tp_vec;
                mean_trans_list[useful_id] += trans;
                valid_num[useful_id]++;
            }

            // 输出处理完成的消息
            printf("========read fs done: %s==============\n",
                   filepath.path().c_str());
        }
    }

    // 计算平均旋转和平移
    cv::FileStorage fs;
    std::filesystem::path fs_path(video_dir);
    fs_path.append("ground.yaml");
    if (!fs.open(fs_path, cv::FileStorage::WRITE)) {
        printf("cannot open %s\n", fs_path.c_str());
    }

    for (int i = 0; i < useful_tags.size(); i++) {
        // 归一化平均值
        mean_weighted_axis_list[i] /= (float)valid_num[i];
        mean_trans_list[i] /= (float)valid_num[i];

        // 计算旋转角和旋转轴
        float angle          = mean_weighted_axis_list[i].norm();
        Eigen::Vector3f axis = mean_weighted_axis_list[i].normalized();
        Eigen::AngleAxisf angle_axis(angle, axis);
        Eigen::Matrix3f rotation_matrix = angle_axis.toRotationMatrix();

        // 构建地面真值矩阵
        Eigen::Matrix4f ground_matrix      = Eigen::Matrix4f::Identity();
        ground_matrix.topLeftCorner(3, 3)  = rotation_matrix;
        ground_matrix.topRightCorner(3, 1) = mean_trans_list[i];
        cv::Mat mat_ground_matrix;
        cv::eigen2cv(ground_matrix, mat_ground_matrix);

        // 写入文件
        fs << "ground_matrix" << mat_ground_matrix;
        std::cout << "mat_ground_matrix:\n" << mat_ground_matrix << std::endl;
        break;
    }

    // 关闭文件存储
    fs.release();
}