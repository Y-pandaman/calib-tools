#include "stage/april_tag.h"
#include "stage/extract_image.h"
#include <Eigen/Eigen>
#include <boost/filesystem.hpp>
#include <iostream>
#include <opencv2/core/eigen.hpp>
#include <opencv2/opencv.hpp>
#include <unistd.h>

#define CLOSE_IMSHOW

int main(int argc, char** argv) {
    int optc;
    std::string path_base;
    std::vector<int> camera_ids;
    float new_size_factor = 1.0;
    float balance         = 0.0;
    std::vector<float> tag_size;
    bool wait_key_0 = false;

    while ((optc = getopt(argc, argv, "f:b:d:v:n:s:c")) != -1) {
        switch (optc) {
        case 'c':
            wait_key_0 = true;
            break;
        case 'd':
            path_base = std::string(optarg);
            break;
        case 'n':
            camera_ids.push_back(atoi(optarg));
            break;
        case 's':
            tag_size.push_back(atof(optarg));
            break;
        case 'f':
            new_size_factor = atof(optarg);
            printf("new_size_factor: %f\n", new_size_factor);
            break;
        case 'b':
            balance = atof(optarg);
            printf("balance: %f\n", balance);
            break;
        default:
            break;
        }
    }

    // 检查是否设置了标签大小，如果没有设置，则输出错误信息并退出
    if (tag_size.size() == 0) {
        printf("Error tag size unset. \nExample: use -s 0.65 to set tag size 0.65m.\n");
        return 0;
    }

    // 初始化AprilTag检测器数组
    std::vector<std::vector<xict_calib::AprilTagDetector>> april_tag_detectors;
    april_tag_detectors.resize(camera_ids.size());
    for (int i = 0; i < camera_ids.size(); i++) {
        april_tag_detectors[i].resize(tag_size.size());
    }

    // 基于路径基础设置图像目录
    boost::filesystem::path images_dir_base(path_base);
    int view_num = camera_ids.size();
    std::vector<cv::Mat> Kmat_list(view_num);
    std::vector<Eigen::Matrix3f> K_list(view_num);

    // 提取图像并根据相机ID和标签大小初始化检测器
    xict_calib::ExtractImage(camera_ids, path_base, new_size_factor, balance);
    for (int j = 0; j < view_num; j++) {
        std::string fs_filename;
        int camera_id = camera_ids[j];
        fs_filename = "camera_" + std::to_string(camera_id) + "_intrin_undistort.yaml";

        // 读取相机内参文件
        boost::filesystem::path fs_path = images_dir_base;
        fs_path /= fs_filename;
        cv::FileStorage fs(fs_path.string(), cv::FileStorage::READ);
        fs["K"] >> Kmat_list[j];

        for (int p = 0; p < tag_size.size(); p++) {
            april_tag_detectors[j][p].Init(
                tag_size[p], Kmat_list[j].at<double>(0, 0),
                Kmat_list[j].at<double>(1, 1), Kmat_list[j].at<double>(0, 2),
                Kmat_list[j].at<double>(1, 2));
            printf("initialize april_tag_detector %d %d done\n", j, p);
        }
    }

    int id = 0;
    for (int k = 0; k < camera_ids.size(); k++) {
        boost::filesystem::path images_dir(images_dir_base);
        printf("images_dir_base: %s\n", images_dir_base.c_str());
        images_dir /= "camera_calib";
        printf("image_dir: %s\n", images_dir.c_str());

        if (!boost::filesystem::exists(images_dir)) {
            printf("no such images dir: %s\n", images_dir.c_str());
            return 0;
        }

        boost::filesystem::directory_iterator end_itr;
        boost::filesystem::directory_iterator list(images_dir);
        int frame_count = 0;
        int tag_count   = 0;

        for (boost::filesystem::directory_iterator itr = list; itr != end_itr; ++itr) {
            if (boost::filesystem::is_directory(itr->status()))
                continue;

            std::string str_id = itr->path().stem().string();
            str_id             = str_id.substr(0, str_id.find("-"));
            if (std::stoi(str_id) != camera_ids[k])
                continue;

            frame_count++;
            printf("%s\n", itr->path().c_str());

            cv::Mat image = cv::imread(itr->path().string());
            printf("detect image: %s\n", itr->path().c_str());

            april_tag_detectors[id][0].Detect(image);

            std::vector<int> tags_id = april_tag_detectors[id][0].GetTagsID();
            std::vector<std::vector<float2>> image_points;
            april_tag_detectors[id][0].GetImagePoints(image_points);

            std::vector<Eigen::Matrix4f> poses;
            april_tag_detectors[id][0].GetPoses(poses);

            boost::filesystem::path poses_file_path = itr->path().parent_path();
            poses_file_path /= "poses/";

            if (!boost::filesystem::exists(poses_file_path)) {
                boost::filesystem::create_directories(poses_file_path);
            }

            poses_file_path /= itr->path().filename().string() + ".yaml";
            cv::FileStorage fs(poses_file_path.string(), cv::FileStorage::WRITE);
            fs << "poses_num" << (int)poses.size();
            for (int i = 0; i < poses.size(); i++) {
                tag_count++;
                cv::Mat pose_mat;
                cv::eigen2cv(poses[i], pose_mat);
                std::string var_name = "pose_" + std::to_string(i);
                fs << var_name << pose_mat;
                var_name = "tag_" + std::to_string(i);
                fs << var_name << tags_id[i];
            }

            printf("image_points size = %zu\n", image_points.size());
            for (int i = 0; i < image_points.size(); i++) {
                cv::putText(
                    image, std::to_string(tags_id[i]),
                    cv::Point(image_points[i][3].x, image_points[i][3].y),
                    cv::FONT_HERSHEY_COMPLEX, 0.7, cv::Scalar(0, 255, 255), 2,
                    8, false);

                for (int j = 0; j < 4; j++) {
                    cv::circle(
                        image,
                        cv::Point(image_points[i][j].x, image_points[i][j].y),
                        2, cv::Scalar(255, 0, 0), 2);
                }
            }

#ifdef CLOSE_IMSHOW
            cv::imshow("test show", image);
            cv::resizeWindow("test show", 960, 540);
            if (wait_key_0)
                cv::waitKey(0);
            else
                cv::waitKey(5);
#endif

            boost::filesystem::path tag_result_dir(path_base);
            tag_result_dir /= "tag_result_dir_" + std::to_string(camera_ids[k]);

            if (!boost::filesystem::exists(tag_result_dir)) {
                boost::filesystem::create_directories(tag_result_dir);
            }

            boost::filesystem::path tag_result_path(tag_result_dir);
            tag_result_path /= itr->path().filename();
            cv::imwrite(tag_result_path.string(), image);

            printf("==============detect image done: %s=============\n",
                   itr->path().c_str());
        }
        id++;
    }
}
