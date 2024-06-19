#include "stage/april_tag.h"

namespace xict_calib {
    AprilTagDetector::~AprilTagDetector() {
        delete detector_;
        delete cam_;
    }

    void AprilTagDetector::Init(float tag_size, float fx, float fy, float cx,
                                float cy) {
        // 初始化标签大小和其他检测器参数
        tag_size_      = tag_size;
        quad_decimate_ = 1.0;
        n_threads_     = 1;
        display_tag_   = false;
        color_id_      = -1;
        thickness_     = 2;
        z_aligned_     = false;

        // 默认选择使用的标签家族和位姿估计方法
        vpDetectorAprilTag::vpAprilTagFamily tagFamily =
            vpDetectorAprilTag::TAG_36h11;
        vpDetectorAprilTag::vpPoseEstimationMethod poseEstimationMethod =
            vpDetectorAprilTag::BEST_RESIDUAL_VIRTUAL_VS;

        try {
            // 初始化相机参数并打印配置信息
            cam_ = new vpCameraParameters;
            cam_->initPersProjWithoutDistortion(fx, fy, cx, cy);
            std::cout << *cam_ << std::endl;
            std::cout << "poseEstimationMethod: " << poseEstimationMethod
                      << std::endl;
            std::cout << "tagFamily: " << tagFamily << std::endl;
            std::cout << "nThreads : " << n_threads_ << std::endl;
            std::cout << "Z aligned: " << z_aligned_ << std::endl;

            // 配置AprilTag检测器
            detector_ = new vpDetectorAprilTag(tagFamily);
            detector_->setAprilTagQuadDecimate(quad_decimate_);
            detector_->setAprilTagPoseEstimationMethod(poseEstimationMethod);
            detector_->setAprilTagRefineDecode(true);
            detector_->setAprilTagRefineEdges(true);
            detector_->setAprilTagRefinePose(true);
            detector_->setAprilTagNbThreads(n_threads_);
            detector_->setDisplayTag(
                display_tag_,
                color_id_ < 0 ? vpColor::none : vpColor::getColor(color_id_),
                thickness_);
            detector_->setZAlignedWithCameraAxis(z_aligned_);
        } catch (const vpException& e) {
            // 捕获并打印任何异常
            std::cerr << "Catch an exception: " << e.getMessage() << std::endl;
        }
    }

    void AprilTagDetector::Init() {
        // 定义要使用的标签族
        vpDetectorAprilTag::vpAprilTagFamily tagFamily =
            vpDetectorAprilTag::TAG_36h11;

        // 定义要使用的位姿估计方法
        vpDetectorAprilTag::vpPoseEstimationMethod poseEstimationMethod =
            vpDetectorAprilTag::BEST_RESIDUAL_VIRTUAL_VS;

        try {
            // 输出位姿估计方法、标签族、线程数和 Z 轴对齐状态
            std::cout << "poseEstimationMethod: " << poseEstimationMethod
                      << std::endl;
            std::cout << "tagFamily: " << tagFamily << std::endl;
            std::cout << "nThreads : " << n_threads_ << std::endl;
            std::cout << "Z aligned: " << z_aligned_ << std::endl;

            // 使用指定的标签族初始化 AprilTag 检测器
            detector_ = new vpDetectorAprilTag(tagFamily);

            // 设置 AprilTag 检测器的各种参数
            detector_->setAprilTagQuadDecimate(
                quad_decimate_);   // 设置四边形检测的抽取因子
            detector_->setAprilTagPoseEstimationMethod(
                poseEstimationMethod);   // 设置位姿估计方法
            detector_->setAprilTagRefineDecode(true);   // 启用解码优化
            detector_->setAprilTagRefineEdges(true);    // 启用边缘优化
            detector_->setAprilTagRefinePose(true);     // 启用位姿优化
            detector_->setAprilTagNbThreads(
                n_threads_);   // 设置用于检测的线程数
            detector_->setDisplayTag(display_tag_,   // 设置是否显示检测到的标签
                                     color_id_ < 0
                                         ? vpColor::none   // 设置标签显示的颜色
                                         : vpColor::getColor(color_id_),
                                     thickness_);   // 设置标签显示线条的粗细
            detector_->setZAlignedWithCameraAxis(
                z_aligned_);   // 设置 Z 轴是否与相机轴对齐
        } catch (const vpException& e) {
            // 捕获并显示初始化过程中发生的任何异常
            std::cerr << "Catch an exception: " << e.getMessage() << std::endl;
        }
    }

    void ComputePose(std::vector<vpPoint>& point,
                     const std::vector<vpImagePoint>& ip,
                     const vpCameraParameters& cam, bool init,
                     vpHomogeneousMatrix& cMo) {
        vpPose pose;   // 创建位姿对象
        double x = 0, y = 0;
        // 遍历特征点集合，将图像坐标转换为世界坐标，并添加到位姿对象中
        for (unsigned int i = 0; i < point.size(); i++) {
            // 将图像坐标转换为世界坐标
            vpPixelMeterConversion::convertPoint(cam, ip[i], x, y);
            point[i].set_x(x);
            point[i].set_y(y);
            pose.addPoint(point[i]);   // 将点添加到位姿估计中
        }
        // 如果需要初始化位姿
        if (init == true) {
            vpHomogeneousMatrix cMo_dem;   // Dementhon方法的位姿矩阵
            vpHomogeneousMatrix cMo_lag;   // Lagrange方法的位姿矩阵
            // 分别使用Dementhon和Lagrange方法计算位姿
            pose.computePose(vpPose::DEMENTHON, cMo_dem);
            pose.computePose(vpPose::LAGRANGE, cMo_lag);
            // 计算两种方法的残差
            double residual_dem = pose.computeResidual(cMo_dem);
            double residual_lag = pose.computeResidual(cMo_lag);
            // 选择残差较小的位姿矩阵
            if (residual_dem < residual_lag)
                cMo = cMo_dem;
            else
                cMo = cMo_lag;
        }
        // 使用虚拟视图与物体位姿计算，更新位姿矩阵
        pose.computePose(vpPose::VIRTUAL_VS, cMo);
    }

    void AprilTagDetector::Detect(cv::Mat image) {
        try {
            // 将OpenCV的Mat图像转换为ViSP的图像格式，以供AprilTag检测器使用。
            vpImage<unsigned char> I;
            vpImageConvert::convert(image, I);

            // 清空之前的检测结果，为新的检测做准备。
            cMo_vec_.clear();
            // 调用AprilTag检测器的detect方法，进行标签检测。
            // tag_size_ 表示标签的实际大小
            // cam_ 表示相机参数
            // cMo_vec_ 用于存储检测到的标签的位姿
            detector_->detect(I, tag_size_, *cam_, cMo_vec_);

        } catch (const vpException& e) {
            // 捕获并处理检测过程中可能出现的异常。
            std::cerr << "Catch an exception: " << e.getMessage() << std::endl;
        }
    }

    void AprilTagDetector::Detect(std::map<int, std::vector<float2> >& points,
                                  cv::Mat image) {
        try {
            // 将OpenCV图像转换为vpImage格式，以供AprilTag检测器使用。
            vpImage<unsigned char> I;
            vpImageConvert::convert(image, I);

            // 使用AprilTag检测器检测图像中的标签。
            detector_->detect(I);
            // 获取检测到的标签角点。
            std::vector<std::vector<vpImagePoint> > vp_points =
                detector_->getTagsCorners();
            // 获取每个标签的ID。
            std::vector<int> tag_ids = detector_->getTagsId();

            // 清空存储点的映射，准备添加新的检测结果。
            points.clear();
            // 遍历每个检测到的标签。
            for (size_t i = 0; i < vp_points.size(); i++) {
                std::vector<vpImagePoint> pp = vp_points[i];
                // 获取与当前标签相关的消息，其中可能包含标签ID。
                std::string message = detector_->getMessage(i);
                // 在消息中查找标签ID的位置。
                std::size_t tag_id_pos = message.find("id: ");
                // 如果找到了标签ID，则提取ID并存储角点。
                if (tag_id_pos != std::string::npos) {
                    // 从消息中提取标签ID。
                    int tag_id = atoi(message.substr(tag_id_pos + 4).c_str());
                    // 在映射中为当前标签ID创建一个新的条目。
                    points[tag_id] = std::vector<float2>();
                    // 将当前标签的角点转换为float2格式，并添加到对应的条目中。
                    // std::cout << "Tag id: " << tag_id << std::endl;
                    for (size_t j = 0; j < pp.size(); j++) {
                        points[tag_id].emplace_back(
                            make_float2(pp[j].get_u(), pp[j].get_v()));
                    }
                }
            }
        } catch (const vpException& e) {
            // 捕获并处理vpException异常。
            std::cerr << "Catch an exception: " << e.getMessage() << std::endl;
        }
    }

    void AprilTagDetector::GetPoses(std::vector<Eigen::Matrix4f>& poses) {
        // 遍历每个标签的相机到标记物的变换矩阵
        for (size_t i = 0; i < cMo_vec_.size(); i++) {
            vpHomogeneousMatrix& cMo = cMo_vec_[i];

            // 提取旋转矩阵和平移向量
            vpRotationMatrix rotation       = cMo.getRotationMatrix();
            vpTranslationVector translation = cMo.getTranslationVector();

            // 将vpRotationMatrix转换为Eigen的旋转矩阵
            Eigen::Matrix3f R = Eigen::Matrix3f::Identity();
            R << rotation.getCol(0)[0], rotation.getCol(1)[0],
                rotation.getCol(2)[0], rotation.getCol(0)[1],
                rotation.getCol(1)[1], rotation.getCol(2)[1],
                rotation.getCol(0)[2], rotation.getCol(1)[2],
                rotation.getCol(2)[2];
            // 根据vpRotationMatrix的定义，调整Eigen旋转矩阵的Y轴方向
            R.col(1) *= -1;

            // 初始化一个Eigen的4x4位姿矩阵
            Eigen::Matrix4f pose = Eigen::Matrix4f::Identity();
            // 设置旋转部分
            pose.topLeftCorner(3, 3) = R;
            // 设置平移部分
            pose.topRightCorner(3, 1) << translation[0], translation[1],
                translation[2];

            // 将位姿矩阵添加到结果集合中
            poses.emplace_back(pose);
        }
    }

    void AprilTagDetector::GetImagePoints(
        std::vector<std::vector<float2> >& points) {
        // 清空并重新调整points的大小，以匹配检测到的标签数量
        points.clear();
        points.resize(detector_->getNbObjects());

        // 遍历每个检测到的标签
        for (size_t i = 0; i < detector_->getNbObjects(); i++) {
            // 获取当前标签的角点集合
            std::vector<vpImagePoint> pp = detector_->getPolygon(i);
            // 获取当前标签的相关信息字符串
            std::string message = detector_->getMessage(i);

            // 尝试从信息字符串中提取标签ID
            std::size_t tag_id_pos = message.find("id: ");
            if (tag_id_pos != std::string::npos) {
                // 如果ID存在，则转换为整数类型
                int tag_id = atoi(message.substr(tag_id_pos + 4).c_str());
            }

            // 将每个角点的u,v坐标转换为float2类型，并添加到对应的标签点集合中
            for (size_t j = 0; j < pp.size(); j++) {
                points[i].emplace_back(
                    make_float2(pp[j].get_u(), pp[j].get_v()));
            }
        }
    }

    void AprilTagDetector::GetWorldPoints(std::vector<float2>& points) {
        // 计算标签大小的一半，以便后续构建角点的坐标
        float half = tag_size_ * 0.5f;

        // 添加左下角点
        points.emplace_back(make_float2((-half), -(-half)));
        // 添加右下角点
        points.emplace_back(make_float2((half), -(-half)));
        // 添加右上角点
        points.emplace_back(make_float2((half), -(half)));
        // 添加左上角点
        points.emplace_back(make_float2((-half), -(half)));
    }

    std::vector<int> AprilTagDetector::GetTagsID() {
        return detector_->getTagsId();
    }
}   // namespace xict_calib