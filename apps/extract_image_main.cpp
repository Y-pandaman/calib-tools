/*
 * @Author: 姚潘涛
 * @Date: 2024-06-14 10:03:17
 * @LastEditors: 姚潘涛
 * @LastEditTime: 2024-06-18 10:14:20
 * @Description:
 *
 * Copyright (c) 2024 by pandaman, All Rights Reserved.
 */
#include "stage/extract_image.h"

int main(int argc, char** argv) {
    int optc = 0;
    std::filesystem::path video_path_base;
    std::vector<int> camera_ids;
    bool flag_get_all_image = false;
    float new_size_factor   = 1.2;
    float balance           = 0.3;
    while ((optc = getopt(argc, argv, "d:n:f:b:")) != -1) {
        switch (optc) {
        case 'd':
            video_path_base = std::filesystem::path(optarg);
            printf("video_path_base: %s\n", video_path_base.c_str());
            break;
        case 'n':
            camera_ids.push_back(atoi(optarg));
            printf("camera_id: %d\n", atoi(optarg));
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
    xict_calib::ExtractImage(camera_ids, video_path_base, new_size_factor,
                             balance);
}
