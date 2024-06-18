#include "core/camera_calib.h"
namespace xict_calib {
    /**
     * 计算两个图像的差异分数。
     *
     * 该函数通过比较两个图像的每个像素值的差异，并计算这些差异值的平均值，来评估两个图像的相似度。
     * 差异分数越小，表示两个图像越相似；差异分数越大，表示两个图像越不相似。
     *
     * @param img_1 第一个图像，是一个CV_8UC1类型的OpenCV Mat对象。
     * @param img_2 第二个图像，是一个CV_8UC1类型的OpenCV Mat对象。
     * @return 返回两个图像的差异分数，这是一个浮点数，范围在0到255之间。
     */
    float DiffScore(const cv::Mat_<uchar>& img_1,
                    const cv::Mat_<uchar>& img_2) {
        float diff_value = 0.0f;   // 初始化差异值累加器。

        // 遍历两个图像的每个像素，计算像素值差异并累加到diff_value。
        for (int y = 0; y < img_1.rows; ++y) {
            for (int x = 0; x < img_1.cols; ++x) {
                diff_value += fabs(img_1(y, x) - img_2(y, x));
            }
        }

        // 计算差异值的平均值，作为差异分数。
        diff_value /= (img_2.rows * img_2.cols);

        return diff_value;   // 返回差异分数。
    }

    /**
     * 从视频中提取不模糊的图像
     *
     * @param images_dir 存储提取图像的目录
     * @param video_path 视频文件的路径
     * @param camera_idx 摄像头的索引，用于图像命名
     * @param noblur_image_idx_vec 存储不模糊图像索引的向量
     * @param motion_threshold 运动阈值，用于判断图像是否模糊
     * @param local_mini_stride 局部最小帧间隔，用于确定存储图像的频率
     * @param image_base_count 图像基础计数，用于图像命名
     * @return 返回更新后的不模糊图像索引向量
     */
    std::vector<int> ExtractNoBlurImagesFromVideo(
        std::string images_dir, std::string video_path, int camera_idx,
        std::vector<int> noblur_image_idx_vec, float motion_threshold,
        int local_mini_stride, int image_base_count) {
        assert(local_mini_stride % 2 == 1);   // 确保局部最小帧间隔为奇数

        // 初始化视频捕获对象并打开视频文件
        cv::VideoCapture color_video_capture;
        mkdir(images_dir.c_str(), 0755);   // 创建存储图像的目录
        color_video_capture.open(video_path);
        cv::Mat color_image;   // 用于存储当前帧的彩色图像
        int count = 0;         // 用于计数已处理的帧数
        std::vector<std::pair<float2, cv::Mat> >
            color_image_buffer;   // 用于临时存储彩色图像及其模糊得分
        cv::Mat prev_gray_image;   // 用于存储前一帧的灰度图像

        // 如果已经给定了不模糊图像的索引，只提取这些索引对应的帧
        if (noblur_image_idx_vec.size() > 0) {
            int image_idx = 0, count = 0;
            while (color_video_capture.read(color_image)) {
                if (image_idx == noblur_image_idx_vec[count]) {
                    // 根据camera_idx和计数为图像命名
                    char name[512];
                    if (camera_idx < 0) {
                        sprintf(name, (images_dir + "/%05d_color.png").c_str(),
                                count + image_base_count);
                    } else {
                        sprintf(name, (images_dir + "/%d-%d.png").c_str(),
                                camera_idx, count + image_base_count);
                    }
                    // 写入当前帧到指定文件
                    cv::imwrite(name, color_image);
                    ++count;
                }
                ++image_idx;
            }
        } else {
            int image_idx = 0;
            while (color_video_capture.read(color_image)) {
                // 将彩色图像转换为灰度图像
                cv::Mat gray_image;
                cv::cvtColor(color_image, gray_image, CV_BGR2GRAY);
                float blur_score = 1000.0f;   // 初始化模糊得分
                // 如果前一帧存在，计算当前帧与前一帧的差异得分
                if (prev_gray_image.rows > 0) {
                    blur_score = DiffScore(prev_gray_image, gray_image);
                }
                // std::cout << "blur_score: " << blur_score << std::endl;
                // 将当前帧的模糊得分和图像本身存入缓冲区
                color_image_buffer.emplace_back(
                    std::make_pair(make_float2(blur_score, (float)image_idx),
                                   color_image.clone()));
                // 当缓冲区达到指定大小时，检查其中的局部最小值
                if (color_image_buffer.size() == local_mini_stride) {
                    bool is_local_mini = true;
                    for (int i = 0; i < local_mini_stride; ++i) {
                        if (i != (local_mini_stride / 2) &&
                            color_image_buffer[local_mini_stride / 2].first.x >
                                color_image_buffer[i].first.x) {
                            is_local_mini = false;
                            break;
                        }
                    }
                    // 如果局部最小值的得分低于运动阈值，并且满足局部最小条件，则认为该帧不模糊
                    if (color_image_buffer[local_mini_stride / 2].first.x <
                            motion_threshold &&
                        is_local_mini) {
                        cv::imshow(
                            "non-blur image",
                            color_image_buffer[local_mini_stride / 2].second);
                        //                    cv::waitKey(1);
                        // 根据camera_idx和计数为图像命名
                        char name[512];
                        if (camera_idx < 0) {
                            sprintf(name,
                                    (images_dir + "/%05d_color.png").c_str(),
                                    count + image_base_count);
                        } else {
                            sprintf(name, (images_dir + "/%d-%d.png").c_str(),
                                    camera_idx, count + image_base_count);
                        }
                        // 将当前局部最小帧的索引和图像写入结果
                        noblur_image_idx_vec.emplace_back(
                            color_image_buffer[local_mini_stride / 2].first.y);
                        cv::imwrite(
                            name,
                            color_image_buffer[local_mini_stride / 2].second);
                        ++count;
                        // 清除缓冲区中早于局部最小帧的图像
                        for (int i = 0; i < local_mini_stride / 2; ++i) {
                            color_image_buffer.erase(
                                color_image_buffer.begin());
                        }
                    }
                    // 从缓冲区中清除最早的一帧图像
                    color_image_buffer.erase(color_image_buffer.begin());
                }
                prev_gray_image = gray_image.clone();   // 更新前一帧的灰度图像
                ++image_idx;
            }
        }

        return noblur_image_idx_vec;   // 返回更新后的不模糊图像索引向量
    }

    /**
     * 从视频中提取图像并保存到指定目录。
     *
     * @param images_dir 保存提取图像的目录路径。
     * @param video_path 视频文件的路径。
     * @param camera_idx 摄像头索引，用于在文件名中区分不同摄像头的数据。
     *
     * 此函数首先创建指定的图像保存目录，然后打开视频文件，逐帧读取视频中的彩色图像，
     * 并将每帧图像保存为PNG格式的文件，文件名包含帧编号和摄像头索引。
     */
    void ExtractImagesFromVideo(std::string images_dir, std::string video_path,
                                int camera_idx) {
        // 初始化用于读取视频的OpenCV对象
        cv::VideoCapture color_video_capture;
        // 创建图像保存目录，权限设置为755
        mkdir(images_dir.c_str(), 0755);
        // 打开视频文件
        color_video_capture.open(video_path);
        // 用于存储当前帧的彩色图像
        cv::Mat color_image;
        // 用于计数已保存的图像数量
        int count = 0;
        // 用于临时存储彩色图像及其对应的时间戳
        std::vector<std::pair<float, cv::Mat> > color_image_buffer;
        // 用于存储前一帧的灰度图像，这里未使用
        cv::Mat prev_gray_image;

        // 循环读取视频中的每一帧
        while (color_video_capture.read(color_image)) {
            // 准备图像文件名，格式为"目录/帧编号_color_摄像头索引.png"
            char name[512];
            sprintf(name, (images_dir + "/%05d_color_%d.png").c_str(), count,
                    camera_idx);
            // 将当前帧图像保存到文件
            cv::imwrite(name, color_image);
            // 增加帧计数
            ++count;
        }
    }

    /**
     * @brief FishEyeCameraCalib 大致功能是进行鱼眼相机的校准。
     *
     * @param images_dir 图像目录，包含用于校准的图像。
     * @param camera_idx 相机索引，用于区分不同的相机。
     */
    void FisheyeCameraCalib(std::string images_dir, int camera_idx) {
        // 校准板的行数和列数
        int pattern_rows = 6;
        int pattern_cols = 9;
        // 校准板的尺寸
        cv::Size patternsize(pattern_cols, pattern_rows);
        // 校准板上每个角的三维坐标
        std::vector<cv::Point3d> objp;
        for (int row = 0; row < patternsize.height; ++row) {
            for (int col = 0; col < patternsize.width; ++col) {
                objp.emplace_back(cv::Point3d(row, col, 0));
            }
        }

        // 保存所有校准图像中检测到的三维点和二维点
        std::vector<std::vector<cv::Point3d> > objpoints;
        std::vector<std::vector<cv::Point2f> > imgpoints;
        // 图像尺寸
        cv::Size image_size;
        for (int i = 0;; ++i) {
            char name[512];
            sprintf(name, (images_dir + "/%05d_color.png").c_str(), i);
            // 读取图像
            cv::Mat color_image = cv::imread(name);
            // 如果图像为空，则结束循环
            if (color_image.rows == 0)
                break;
            // 转换为灰度图像
            cv::Mat gray_image;
            cv::cvtColor(color_image, gray_image,
                         cv::COLOR_BGR2GRAY);   // source image
            // 如果是第一张图像，记录图像尺寸
            if (i == 0)
                image_size = color_image.size();
            // 存储角点
            std::vector<cv::Point2f> corners;

            // 检测棋盘格角点
            // CALIB_CB_FAST_CHECK saves a lot of time on images
            // that do not contain any chessboard corners
            bool patternfound = findChessboardCorners(
                gray_image, patternsize, corners,
                cv::CALIB_CB_ADAPTIVE_THRESH + cv::CALIB_CB_NORMALIZE_IMAGE +
                    cv::CALIB_CB_FAST_CHECK);

            // 如果找到棋盘格
            if (patternfound) {
                // 保存三维点
                objpoints.emplace_back(objp);
                // 精确角点位置
                cv::cornerSubPix(
                    gray_image, corners, cv::Size(7, 7), cv::Size(-1, -1),
                    cv::TermCriteria(CV_TERMCRIT_EPS + CV_TERMCRIT_ITER, 30,
                                     0.1));
                // 保存二维点
                imgpoints.emplace_back(corners);

                // 在原图上绘制角点
                cv::drawChessboardCorners(color_image, patternsize,
                                          cv::Mat(corners), patternfound);
            }
        }

        // 初始化相机内参矩阵和畸变参数
        cv::Mat Ka = cv::Mat::eye(3, 3, CV_64F);   // Creating distortion matrix
        cv::Mat Da = cv::Mat::ones(1, 4, CV_64F);
        cv::Mat Knew = cv::Mat::eye(3, 3, CV_64F);
        // 用于存储旋转向量和位移向量
        std::vector<cv::Vec3d> rvec;
        std::vector<cv::Vec3d> tvec;

        // 校准标志位
        int calibration_flags = cv::fisheye::CALIB_RECOMPUTE_EXTRINSIC +
                                cv::fisheye::CALIB_CHECK_COND +
                                cv::fisheye::CALIB_FIX_SKEW;
        // 执行鱼眼相机校准
        cv::fisheye::calibrate(objpoints, imgpoints, image_size, Ka, Da, rvec,
                               tvec,
                               calibration_flags);   // Calibration
        // 输出相机内参矩阵和畸变参数
        std::cout << "K matrix: " << Ka << std::endl;
        std::cout << "D matrix: " << Da << std::endl;
        // 保存相机内参
        cv::FileStorage fs(images_dir + "/../camera_" +
                               std::to_string(camera_idx) + "_intrin.yaml",
                           cv::FileStorage::WRITE);
        fs << "K " << Ka;
        fs << "D " << Da;
        fs << "image_size " << image_size;
        fs.release();

        // 是否进行可视化
        bool vis = false;
        if (vis) {
            // 初始化矫正映射矩阵
            cv::Mat eye_mat = cv::Mat::eye(3, 3, CV_32F);

            // 初始化两个映射矩阵
            cv::Mat map1, map2;

            // 开始计算矫正映射
            std::cout << "start initUndistortRectifyMap..\n";
            // 使用鱼眼镜头参数和矫正矩阵初始化矫正映射
            cv::fisheye::initUndistortRectifyMap(
                Ka, Da, eye_mat, Ka, image_size, CV_16SC2, map1, map2);
            std::cout << "initUndistortRectifyMap done!\n";

            // 循环处理所有图像
            // 对所有图像进行畸变矫正
            cv::Mat rec_color_image;
            for (int i = 0;; ++i) {
                // 构造图像文件名
                char name[512];
                sprintf(name, (images_dir + "/%05d_color.png").c_str(), i);

                // 读取图像
                cv::Mat color_image = cv::imread(name);

                // 如果图像为空，结束循环
                if (color_image.rows == 0)
                    break;

                // 应用映射矩阵进行图像矫正
                cv::remap(color_image, rec_color_image, map1, map2,
                          cv::INTER_LINEAR);

                // 显示矫正后的图像
                cv::imshow("rec_color_image", rec_color_image);

                // 等待按键
                cv::waitKey(1);
            }
        }
    }

    /**
     * 视频去畸变处理函数。
     *
     * 功能描述：该函数从指定的YAML文件中读取相机内参信息，计算用于校正畸变的新摄像机矩阵，
     * 初始化映射以进行畸变矫正，并将此映射应用于视频中的每一帧以进行畸变校正处理。
     * 最终输出校正后的视频及对应的掩模图像（首帧）至指定路径。
     *
     * @param video_path 输入视频文件的路径。
     * @param camera_idx
     * 相机的索引编号，用于构建读取/保存相关参数文件的名称。
     * @param new_size_factor 新视频大小相对于原视频的缩放因子。
     * @param balance 用于计算新摄像机矩阵的平衡参数，影响校正区域的大小。
     */
    void UndistortVideo(std::string video_path, int camera_idx,
                        float new_size_factor, float balance) {
        // 初始化畸变矩阵与系数矩阵
        cv::Mat Ka = cv::Mat::eye(3, 3, CV_64F);
        cv::Mat Da = cv::Mat::ones(1, 4, CV_64F);

        // 读取YAML文件获取内参、图像尺寸等信息
        cv::Size image_size;
        boost::filesystem::path fs_path(video_path);
        fs_path = fs_path.parent_path();
        fs_path /= "camera_" + std::to_string(camera_idx) + "_intrin.yaml";
        cv::FileStorage fs(fs_path.string(), cv::FileStorage::READ);
        fs["K"] >> Ka;
        fs["D"] >> Da;
        fs["image_size"] >> image_size;
        fs.release();

        // 计算新的视频尺寸
        cv::Size new_size(image_size.width * new_size_factor,
                          image_size.height * new_size_factor);

        // 输出读取到的内参、畸变系数及原始图像尺寸信息
        std::cout << "Intrinsics (Ka):\n" << Ka << std::endl;
        std::cout << "Distortion Coefficients (Da):\n" << Da << std::endl;
        std::cout << "Image Size:\n" << image_size << std::endl;

        // 估计用于校正畸变的新摄像机矩阵，初始化映射
        cv::Mat eye_mat = cv::Mat::eye(3, 3, CV_32F);
        cv::Mat map1, map2, new_Ka;
        cv::fisheye::estimateNewCameraMatrixForUndistortRectify(
            Ka, Da, image_size, eye_mat, new_Ka, balance, new_size);
        cv::fisheye::initUndistortRectifyMap(Ka, Da, eye_mat, new_Ka, new_size,
                                             CV_16SC2, map1, map2);

        // 输出新摄像机内参矩阵并保存至YAML文件
        fs_path = fs_path.parent_path();
        fs_path /=
            "camera_" + std::to_string(camera_idx) + "_intrin_undistort.yaml";
        fs.open(fs_path.string(), cv::FileStorage::WRITE);
        fs << "K" << new_Ka << "D" << Da << "image_size" << new_size;
        fs.release();

        // 准备输出视频文件
        cv::VideoWriter output_video;
        boost::filesystem::path output_video_path(video_path);
        output_video_path = output_video_path.parent_path();
        output_video_path /= std::to_string(camera_idx) + "_undistort.avi";
        output_video.open(output_video_path.string(),
                          cv::VideoWriter::fourcc('M', 'J', 'P', 'G'), 30,
                          new_size, true);

        // 打开视频并逐帧进行畸变校正
        cv::VideoCapture color_video_capture(video_path);
        cv::Mat color_image, mask_image, rec_color_image;
        int count = 0;
        while (color_video_capture.read(color_image)) {
            cv::remap(color_image, rec_color_image, map1, map2,
                      cv::INTER_LINEAR);
            output_video << rec_color_image;

            // 首帧生成并保存掩模图像
            if (count == 0) {
                mask_image = cv::Mat::ones(color_image.size(), CV_8UC1) * 255;
                cv::remap(mask_image, mask_image, map1, map2, cv::INTER_LINEAR);
                boost::filesystem::path mask_image_path(video_path);
                mask_image_path = mask_image_path.parent_path();
                mask_image_path /= std::to_string(camera_idx) + "_mask.png";
                cv::imwrite(mask_image_path.string(), mask_image);
            }
            std::cout << count++ << "\r";
        }

        // 释放资源
        color_video_capture.release();
        output_video.release();
    }

    /**
     * @brief 对每个视图进行校准，转换到空中视图
     *
     * 本函数的目的是通过检测图像中的AprilTag标记，来建立地面视图和空中视图之间的映射关系。
     * 它首先初始化AprilTag检测器，然后循环处理每对对应的地面视图和空中视图图像。在每对图像中，
     * 它检测AprilTag的位置，并尝试匹配来自两幅图像的标签。一旦找到足够的匹配点，它使用这些点
     * 来计算一个 homography（ homography
     * 是一种用于平面场景中图像之间映射的几何变换矩阵）。 最后，它将这个
     * homography 存储起来，用于将地面视图转换为空中视图。
     *
     * @param images_dir 地面视图图像的目录
     * @param airview_images_dir 空中视图图像的目录
     * @param camera_idx 相机索引，用于标识特定的相机
     * @param target_w_ 空中视图的目标宽度
     * @param target_h_ 空中视图的目标高度
     */
    void CalibEachViewToAirView(std::string images_dir,
                                std::string airview_images_dir, int camera_idx,
                                int target_w_, int target_h_) {
        // 初始化AprilTag检测器
        xict_calib::AprilTagDetector apriltag_detector;
        apriltag_detector.Init();

        // 用于存储检测到的角点
        std::vector<std::vector<cv::Point2f> > imgpoints_0, imgpoints_1;
        // 存储图像大小
        cv::Size image_size;
        // 无限循环，通过中断条件退出
        for (int i = 0;; ++i) {
            // 构造图像文件名
            char name[512];
            sprintf(name, (images_dir + "%d-%d.png").c_str(), camera_idx, i);
            // 读取地面视图图像
            cv::Mat color_image_0 = cv::imread(name);
            sprintf(name, (airview_images_dir + "/airview.jpg").c_str(), 0, i);
            // 读取空中视图图像
            cv::Mat temp_color_image_1 = cv::imread(name);
            cv::Mat color_image_1      = temp_color_image_1;

            // 设置目标图像的宽度和高度
            int target_w = target_w_;
            int target_h = target_h_;

            // 检查图像是否为空，如果是则退出循环
            if (color_image_0.rows == 0 || color_image_1.rows == 0)
                break;

            // 转换图像到灰度
            cv::Mat gray_image_0, gray_image_1;
            cv::cvtColor(color_image_0, gray_image_0, cv::COLOR_BGR2GRAY);
            cv::cvtColor(color_image_1, gray_image_1, cv::COLOR_BGR2GRAY);

            // 如果是第一张图像，记录图像大小
            if (i == 0)
                image_size = color_image_0.size();

            // 初始化角点存储
            std::vector<cv::Point2f> corners_0, corners_1;

            // 初始化标记点存储
            std::map<int, std::vector<float2> > points_0, points_1;
            std::vector<cv::Point2f> match_points_0, match_points_1;

            // 检测图像中的AprilTags
            std::cout << "-----------------------" << std::endl;
            std::map<int, std::vector<float2> >::iterator iter;
            apriltag_detector.Detect(points_0, color_image_0);
            apriltag_detector.Detect(points_1, color_image_1);

            // 在图像中绘制检测到的AprilTags
            for (iter = points_0.begin(); iter != points_0.end(); ++iter) {
                for (int i = 0; i < points_0[iter->first].size(); ++i) {
                    cv::circle(color_image_0,
                               cv::Point2f(points_0[iter->first][i].x,
                                           points_0[iter->first][i].y),
                               15, cv::Scalar(0, 0, 255), 12);
                }
            }
            for (iter = points_1.begin(); iter != points_1.end(); ++iter) {
                for (int i = 0; i < points_1[iter->first].size(); ++i) {
                    cv::circle(color_image_1,
                               cv::Point2f(points_1[iter->first][i].x,
                                           points_1[iter->first][i].y),
                               15, cv::Scalar(0, 0, 255), 12);
                }
            }

            // 缩小图像以供显示
            cv::Mat vis_color_image_0, vis_color_image_1;
            cv::resize(
                color_image_0, vis_color_image_0,
                cv::Size(color_image_0.cols / 4, color_image_0.rows / 4));
            cv::resize(
                color_image_1, vis_color_image_1,
                cv::Size(color_image_1.cols / 4, color_image_1.rows / 4));

            // 遍历空中视图中的标签，寻找匹配的地面视图标签
            std::map<int, std::vector<float2> >::iterator match_iter;
            for (iter = points_1.begin(); iter != points_1.end();
                 ++iter) {   // points_1无人机图像上的点
                std::cout << "tag id: " << iter->first << std::endl;

                if ((match_iter = points_0.find(iter->first)) !=
                    points_0.end()) {
                    std::cout << match_iter->first << std::endl;
                    std::cout << points_0[match_iter->first].size()
                              << std::endl;
                    for (int i = 0; i < points_0[match_iter->first].size();
                         ++i) {   // points_0 环视相机拍摄图像上的点
                        match_points_0.emplace_back(cv::Point2f(
                            points_0[match_iter->first][i].x * 2.0 / 3.0,
                            points_0[match_iter->first][i].y * 2.0 / 3.0));
                        float px = (points_1[match_iter->first][i].x +
                                    color_image_1.cols / 2) /
                                   (color_image_1.cols * 2) * target_w;
                        float py = (points_1[match_iter->first][i].y +
                                    color_image_1.rows / 2) /
                                   (color_image_1.rows * 2) * target_h;

                        match_points_1.emplace_back(cv::Point2f(px, py));
                    }
                }
            }
            std::cout << std::endl;
            std::cout << "-----------------------" << std::endl;

            vis_color_image_1 =
                cv::Mat::zeros(temp_color_image_1.rows * 2,
                               temp_color_image_1.cols * 2, CV_8UC3);
            temp_color_image_1.copyTo(vis_color_image_1(cv::Rect(
                temp_color_image_1.cols / 2, temp_color_image_1.rows / 2,
                temp_color_image_1.cols, temp_color_image_1.rows)));
            // 在图像中绘制匹配的点
            if (match_points_1.size() > 0) {
                for (int i = 0; i < match_points_0.size(); ++i) {
                    cv::circle(color_image_0, match_points_0[i], 15,
                               cv::Scalar(0, 255, 255), 12);
                    cv::circle(vis_color_image_1, match_points_1[i], 15,
                               cv::Scalar(0, 255, 255), 12);
                }
                cv::Mat vis_color_image_0;
                cv::resize(
                    color_image_0, vis_color_image_0,
                    cv::Size(color_image_0.cols / 4, color_image_0.rows / 4));
                cv::resize(vis_color_image_1, vis_color_image_1,
                           cv::Size(vis_color_image_1.cols / 4,
                                    vis_color_image_1.rows / 4));
                // 计算两幅图像之间的homography
                cv::Mat H =
                    cv::findHomography(match_points_0, match_points_1, cv::RHO);
                std::cout << "H:\n" << H << std::endl;
                // 存储homography矩阵
                cv::FileStorage fs(images_dir + "/../camera_" +
                                       std::to_string(camera_idx) +
                                       "_homography.yaml",
                                   cv::FileStorage::WRITE);
                fs << "H" << H;
                fs.release();
                printf("fs output H done\n");
                return;
            }
        }
    }

    /**
     * @brief 测试图像拼接过程中的融合算法
     *
     * 该函数通过给定一组图像、对应的掩模以及变换矩阵，实现图像的拼接融合。
     * 具体步骤包括：
     * 1. 初始化融合图像和权重图像。
     * 2. 对于每张图像，根据变换矩阵将其变形并融合到融合图像中。
     * 3. 计算每像素的加权平均值，得到最终的融合图像。
     *
     * @param images 输入的图像数组。
     * @param masks 对应的图像掩模数组，用于指定每像素的融合权重。
     * @param Hs 图像之间的变换矩阵数组。
     * @param tgt_w_ 目标图像的宽度。
     * @param tgt_h_ 目标图像的高度。
     */
    void TestStitching(std::vector<cv::Mat> images, std::vector<cv::Mat> masks,
                       std::vector<cv::Mat> Hs, int tgt_w_, int tgt_h_) {
        // 初始化融合图像和权重图像
        cv::Mat blend_image = cv::Mat::zeros(tgt_h_, tgt_w_, CV_32FC3);
        cv::Mat weight_image =
            cv::Mat::ones(tgt_h_, tgt_w_, CV_32FC1) * 0.0000001;

        for (int i = 0; i < images.size(); ++i) {
            // 初始化变形图像和掩模
            cv::Mat warp_image = cv::Mat::zeros(tgt_h_, tgt_w_, CV_8UC3);
            cv::Mat warp_mask  = cv::Mat::zeros(tgt_h_, tgt_w_, CV_8UC1);

            // 获取当前图像的大小
            cv::Size image_size = images[i].size();
            // 反转变换矩阵
            cv::Mat H = Hs[i].inv();

            // 遍历融合图像的每个像素
            int sign = 1;
            for (int row = 0; row < blend_image.rows; ++row) {
                for (int col = 0; col < blend_image.cols; ++col) {
                    cv::Vec3f v   = cv::Vec3f(col, row, 1.0);
                    cv::Mat res_v = H * v;
                    // 判断像素是否在当前图像范围内
                    if (sign * res_v.at<float>(2, 0) < 0) {
                        float x = res_v.at<float>(0, 0) / res_v.at<float>(2, 0);
                        float y = res_v.at<float>(1, 0) / res_v.at<float>(2, 0);

                        // 如果像素在掩模范围内，则提取对应像素值
                        if (y >= 1 && y < masks[i].rows - 1 && x >= 1 &&
                            x < masks[i].cols - 1) {
                            cv::Mat patch_1, patch_2;
                            cv::getRectSubPix(images[i], cv::Size(1, 1),
                                              cv::Point2f(x, y), patch_1);
                            cv::getRectSubPix(masks[i], cv::Size(1, 1),
                                              cv::Point2f(x, y), patch_2);
                            warp_image.at<cv::Vec3b>(row, col) =
                                patch_1.at<cv::Vec3b>(0, 0);
                            warp_mask.at<uchar>(row, col) =
                                patch_2.at<uchar>(0, 0);
                        }
                    }
                }
            }

            // 缩小变形图像以提高处理效率
            cv::Mat vis_warp_image;
            cv::resize(warp_image, vis_warp_image,
                       cv::Size(warp_image.cols / 4, warp_image.rows / 4));

            // 根据掩模对图像进行融合
            for (int row = 0; row < blend_image.rows; ++row) {
                for (int col = 0; col < blend_image.cols; ++col) {
                    if (warp_mask.at<uchar>(row, col) > 128) {
                        blend_image.at<float3>(row, col).x +=
                            (warp_image.at<uchar3>(row, col).x / 255.0f);
                        blend_image.at<float3>(row, col).y +=
                            (warp_image.at<uchar3>(row, col).y / 255.0f);
                        blend_image.at<float3>(row, col).z +=
                            (warp_image.at<uchar3>(row, col).z / 255.0f);
                        ++weight_image.at<float>(row, col);
                    }
                }
            }
        }

        // 根据权重对融合图像进行归一化
        for (int row = 0; row < blend_image.rows; ++row) {
            for (int col = 0; col < blend_image.cols; ++col) {
                blend_image.at<float3>(row, col).x /=
                    weight_image.at<float>(row, col);
                blend_image.at<float3>(row, col).y /=
                    weight_image.at<float>(row, col);
                blend_image.at<float3>(row, col).z /=
                    weight_image.at<float>(row, col);
            }
        }

        // 将浮点数图像转换为8位图像
        blend_image.convertTo(blend_image, CV_8UC3, 255.0f);

        // 保存融合后的图像
        cv::imwrite("./blend_image.png", blend_image);
        return;
    }
}   // namespace xict_calib