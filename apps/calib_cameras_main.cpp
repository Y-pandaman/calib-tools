#include "stage/tag_poses_set.h"
#include "utils/utils.h"
#include <Eigen/Eigen>
#include <boost/filesystem.hpp>
#include <opencv2/core/eigen.hpp>
#include <opencv2/opencv.hpp>
#include <unistd.h>
#include <iostream>
#include <vector>
#include <string>

/**
 * 获取目录中的文件数量
 * @param list
 * 一个boost::filesystem::directory_iterator对象，指向目录中的第一个元素
 * @return 目录中文件的数量
 *
 * 该函数通过遍历给定的目录迭代器，计算目录中的文件数量。
 * 它不包括子目录中的文件，只统计当前目录层级的文件。
 */
int getFileNum(const boost::filesystem::directory_iterator& list) {
    int result = 0;                 // 初始化文件计数器
    for (auto& filename : list) {   // 遍历目录中的每个文件
        result++;                   // 对每个文件进行计数
    }
    return result;   // 返回计数结果
}

int main(int argc, char** argv) {
    // 初始化路径基础变量和相机ID列表
    boost::filesystem::path path_base;
    std::vector<int> camera_ids;

    // 用于循环处理命令行参数
    int optc;
    while ((optc = getopt(argc, argv, "d:n:")) != -1) {
        switch (optc) {
        case 'd':
            // 设置路径基础变量为命令行参数指定的路径
            // 文件路径
            path_base = boost::filesystem::path(optarg);
            break;
        case 'n':
            // 将命令行参数指定的相机ID转换为整数并添加到相机ID列表中
            // 相机编号
            camera_ids.push_back(atoi(optarg));
            break;
        default:
            // 忽略未知选项
            break;
        }
    }

    // 检查相机ID列表长度是否为2，如果不是，则退出程序
    if (camera_ids.size() != 2) {
        return 0;
    }

    // 初始化平均平移向量和平均旋转向量
    Eigen::Vector3f mean_trans_vec(0, 0, 0);
    Eigen::Vector3f mean_rotate_vec(0, 0, 0);
    // 初始化有效矩阵数量为0
    int valid_mat_num = 0;

    // 构建视频文件路径
    boost::filesystem::path video_path(path_base);
    // 存储每个相机的位姿目录路径
    std::vector<boost::filesystem::path> poses_dirs;
    // 存储每个相机的位姿文件迭代器
    std::vector<boost::filesystem::directory_iterator> poses_lists;
    // 初始化位姿文件数量为-1，用于后续计算和校验
    int poses_file_num = -1;
    // 遍历相机ID列表
    for (int i = 0; i < camera_ids.size(); i++) {
        // 将视频路径添加到相机ID对应的位姿目录
        poses_dirs.emplace_back(video_path);
        // 获取当前相机ID
        int camera_id = camera_ids[i];
        // 构建相机校准文件的完整路径
        poses_dirs[i].append("camera_calib/");
        poses_dirs[i].append("poses/");
        // 初始化位姿文件迭代器
        poses_lists.emplace_back(poses_dirs[i]);
        // 获取当前相机的位姿文件数量
        int tp = getFileNum(poses_lists[i]);
        // 如果是第一个相机，则将位姿文件数量赋值给poses_file_num
        // 确认图片数量一致
        if (poses_file_num == -1) {
            poses_file_num = tp;
        }
        // 如果其他相机的位姿文件数量与第一个相机不同，则输出错误信息并退出
        else if (poses_file_num != tp) {
            std::cout << "poses file num unequal!\n";
            return 0;
        }
    }
    // 输出位姿文件总数
    printf("pose_file_num = %d\n", poses_file_num);

    // 初始化相机位置变量，用于后续访问不同相机的位姿数据
    int camera_x = 0, camera_y = 1;

    // 遍历所有的位姿文件
    for (int i = 0; i < poses_file_num; i++) {
        // 打印当前处理的位姿文件ID
        printf("================== pose_file_id = %d ===================\n", i);

        // 存储每个相机的位姿文件路径
        std::vector<boost::filesystem::path> pose_file_path;
        // 存储每个相机的位姿集合
        std::vector<xict_calib::TagPosesSet> poses_set_list;

        // 遍历所有相机ID，为每个相机加载位姿文件
        // 读取两个对应的pose文件
        for (int j = 0; j < camera_ids.size(); j++) {
            int camera_id = camera_ids[j];
            // 构建位姿文件路径
            pose_file_path.emplace_back(poses_dirs[j]);
            pose_file_path[j].append(std::to_string(camera_id) + "-" +
                                     std::to_string(i) + ".png.yaml");

            // 打开位姿文件
            cv::FileStorage fs(pose_file_path[j].string(), cv::FileStorage::READ);
            // 读取位姿数据
            xict_calib::TagPosesSet poses_set;
            printf("readFromFileStorage %s ....\n", pose_file_path[j].c_str());
            poses_set.ReadFromFileStorage(fs);
            printf("readFromFileStorage %s done\n", pose_file_path[j].c_str());

            // 将位姿数据添加到列表中
            poses_set_list.emplace_back(poses_set);
        }

        // 遍历相机X的位姿数据，寻找与相机Y中相同标签ID的位姿
        int pose_x, pose_y;
        for (pose_x = 0; pose_x < poses_set_list[camera_x].poses_num;
             pose_x++) {
            bool match_flag = false;

            // 获取当前相机X的标签ID
            int tag_id = poses_set_list[camera_x].tags_id[pose_x];

            // 在相机Y的位姿数据中寻找相同的标签ID
            for (pose_y = 0; pose_y < poses_set_list[camera_y].poses_num;
                 pose_y++) {
                if (poses_set_list[camera_y].tags_id[pose_y] == tag_id) {
                    match_flag = true;
                    break;
                }
            }

            // 如果没有找到匹配的标签ID，则跳过当前位姿
            if (!match_flag)
                continue;   // no match tag in another photo (camera)

            // 计算相机X到标签和相机Y到标签的转换矩阵
            cv::Mat camx2tag =
                poses_set_list[camera_x].pose_matrix_list[pose_x];
            cv::Mat camy2tag =
                poses_set_list[camera_y].pose_matrix_list[pose_y];

            // 输出转换矩阵供调试
            std::cout << "tag_id: " << tag_id << std::endl;
            std::cout << "camerax_to_tag:\n " << camx2tag << std::endl;
            std::cout << "cameray_to_tag:\n " << camy2tag << std::endl;

            // 计算相机X到相机Y的转换矩阵
            cv::Mat tag2camy  = camy2tag.inv();
            cv::Mat camx2camy = camx2tag * tag2camy;

            // 输出相机X到相机Y的转换矩阵供调试
            std::cout << "cam_x_2_cam_y:\n " << camx2camy << std::endl;

            // 将OpenCV的矩阵转换为Eigen的矩阵，用于计算旋转和平移向量
            Eigen::Matrix4f eigen_camx2camy;
            cv::cv2eigen(camx2camy, eigen_camx2camy);
            Eigen::Vector3f trans_vec = eigen_camx2camy.topRightCorner(3, 1);

            // 检查转换向量的大小，如果大于0.1，则认为位姿无效
            if (trans_vec.norm() > 0.1) {
                printf("invalid tag pose\n");
                continue;
            }

            // 计算旋转矩阵和旋转角轴
            Eigen::Matrix3f rotation_matrix =
                eigen_camx2camy.topLeftCorner(3, 3);
            Eigen::AngleAxisf cur_angle_axis;
            cur_angle_axis.fromRotationMatrix(rotation_matrix);
            Eigen::Vector3f cur_rotate_vec =
                cur_angle_axis.angle() * cur_angle_axis.axis();

            // 输出有效位姿的详细信息
            printf("valid tag pose\n");
            printf("=================================\n");
            valid_mat_num++;
            mean_rotate_vec += cur_rotate_vec;
            std::cout << "cur_rotate_vec: " << cur_rotate_vec << std::endl;
            mean_trans_vec += trans_vec;
        }
    }

    // 平均转换向量和平均旋转向量除以有效矩阵数量，以求得平均值
    mean_trans_vec /= (float)valid_mat_num;
    mean_rotate_vec /= (float)valid_mat_num;

    // 输出有效矩阵数量、平均旋转向量和平均转换向量
    std::cout << "valid_mat_num: " << valid_mat_num << std::endl;
    std::cout << "mean_rotate_vec:\n" << mean_rotate_vec << std::endl;
    std::cout << "mean_trans_vec: \n" << mean_trans_vec << std::endl;

    // 计算平均旋转向量的模长，并将其归一化
    float mean_angle = mean_rotate_vec.norm();
    mean_rotate_vec  = mean_rotate_vec.normalized();

    // 根据平均旋转向量的模长和方向，创建平均旋转轴角表示
    Eigen::AngleAxisf mean_angle_axis(mean_angle, mean_rotate_vec);

    // 从旋转轴角表示转换为旋转矩阵
    Eigen::Matrix3f mean_rotation_matrix = mean_angle_axis.toRotationMatrix();

    // 初始化结果矩阵，并将旋转矩阵和转换向量填入结果矩阵
    Eigen::Matrix4f result_matrix      = Eigen::Matrix4f::Identity();
    result_matrix.topLeftCorner(3, 3)  = mean_rotation_matrix;
    result_matrix.topRightCorner(3, 1) = mean_trans_vec;

    // 输出旋转矩阵和结果矩阵
    std::cout << "mean_rotation_matrix:\n" << mean_rotation_matrix << std::endl;
    std::cout << "result_matrix:\n" << result_matrix << std::endl;

    // 构建文件路径并打开文件存储
    std::string fs_path = path_base.string() + "/camera_extrin_" + std::to_string(camera_ids[camera_x])+"_and_"+std::to_string(camera_ids[camera_y])+".yaml";
    cv::FileStorage fs(fs_path, cv::FileStorage::Mode::WRITE);

    // 将结果矩阵转换为OpenCV的Mat格式，并写入文件
    cv::Mat mat_result_matrix;
    cv::eigen2cv(result_matrix, mat_result_matrix);
    fs << "matrix" + std::to_string(camera_ids[camera_x]) + "to" +
              std::to_string(camera_ids[camera_y])
       << mat_result_matrix;

    // 计算结果矩阵的逆矩阵，并写入文件
    cv::Mat mat_result_matrix_inv = mat_result_matrix.inv();
    fs << "matrix" + std::to_string(camera_ids[camera_y]) + "to" +
              std::to_string(camera_ids[camera_x])
       << mat_result_matrix_inv;

    // 关闭文件存储
    fs.release();
}
