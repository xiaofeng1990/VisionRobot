#include "calibration.h"

#include <iostream>
#include <random> // C++11 随机数库
#include <chrono> // 时间相关功能
#include <iostream>
#include <fstream>
#include <vector>
#include "common/xt_utils.h"
Calibration::Calibration(/* args */)
{
}

Calibration::~Calibration()
{
    episode_.SetFreeMode(0);
    // episode_.GripperOn();
    // if (thread_ != nullptr && thread_->joinable())
    // {
    //     thread_->join();
    // }
}

bool Calibration::InitDevice()
{
    // 初始化设备
    // 这里可以添加设备初始化的代码
    orbbec_.OpenDeviceSyncAlign(); // 替换为实际的序列号
    cv::aruco::Dictionary dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_5X5_50);
    cv::aruco::DetectorParameters detector_params;
    cv::aruco::ArucoDetector detector(dictionary, detector_params);
    aruco_detector_ = detector; // 将检测器赋值给成员变量
    if (episode_.Connect("127.0.0.1", 12345))
    {
        std::cout << "Episode 连接成功 机械臂到初始位置" << std::endl;
        // 退出自由模式
        episode_.SetFreeMode(0);
        std::this_thread::sleep_for(std::chrono::milliseconds(static_cast<int>(100)));
        // 运动到初始位置
        auto result = episode_.MoveXYZRotation({260, 0, 300}, {180, 0, 90}, "xyz", 1);
        double sleep_time = result.get<double>();
        std::this_thread::sleep_for(std::chrono::milliseconds(static_cast<int>(sleep_time * 1000)));
        // 开启读取角度
    }

    return true;
}
// 进行标定 虚函数
void Calibration::PreSample()
{
    std::cout << "请安装深度相机，然后输入 p 开始预采样: " << std::endl;
    while (true)
    {
        char ch = std::cin.get();
        if (ch == 'p')
        {
            std::cout << "================ 进入预采样模式 ================" << std::endl;
            break;
        }
        else
        {
            std::cout << "你输入了: " << ch << std::endl;
            std::cout << "输入错误，应该输入 p" << std::endl;
        }
    }

    std::cout << "15s后进入自由模式, 请手动移动机械臂..." << std::endl;
    std::cout << "=========================================" << std::endl;
    //  测试采图,暂时不进入自由模式
    std::this_thread::sleep_for(std::chrono::seconds(5));
    episode_.SetFreeMode(1);
    bool pre_sample = true;
    int count = 0;
    while (pre_sample)
    {
        try
        {
            // 获取彩图
            auto frameset = orbbec_.GetFrameSet();
            cv::Mat color_mat;
            std::shared_ptr<ob::VideoStreamProfile> profile;
            profile = orbbec_.GetColorMat(frameset, color_mat);

            // 棋盘检测
            cv::Mat gray;
            cv::cvtColor(color_mat, gray, cv::COLOR_BGR2GRAY);
            cv::Size board_size;
            board_size.width = board_size_with_;
            board_size.height = board_size_height_;
            std::vector<cv::Point2f> pointbuf;
            auto found = cv::findChessboardCorners(gray, board_size, pointbuf, cv::CALIB_CB_ADAPTIVE_THRESH | cv::CALIB_CB_FAST_CHECK | cv::CALIB_CB_NORMALIZE_IMAGE);
            if (found)
            {
                cv::cornerSubPix(gray, pointbuf, cv::Size(11, 11), cv::Size(-1, -1), cv::TermCriteria(cv::TermCriteria::EPS + cv::TermCriteria::COUNT, 30, 0.0001));
                cv::drawChessboardCorners(color_mat, board_size, pointbuf, found);
            }
            cv::imshow("image", color_mat);
            int key = cv::waitKey(25);
            switch (key)
            {
            case 's':
            {
                if (!found)
                {
                    cv::putText(color_mat, "No Chessboard Detected", cv::Point(10, 70), cv::FONT_HERSHEY_SIMPLEX, 0.8, cv::Scalar(0, 0, 255), 2);
                    std::cout << "未检测到棋盘格，无法保存图像" << std::endl;
                    cv::imshow("image", color_mat);
                    int key = cv::waitKey(1000);
                    break;
                }
                std::cout << "检测到棋盘格，保存图像" << std::endl;
                // 判断图片质量
                double variance = EvaluateImageQuality(color_mat);
                std::cout << "图像清晰度方差: " << variance << std::endl;

                std::vector<double> angles_list;

                auto angles = episode_.GetMotorAngles();
                // std::cout << "获取角度 " << angles.dump() << std::endl;
                if (angles.is_array())
                    angles_list = angles.get<std::vector<double>>();
                // std::this_thread::sleep_for(std::chrono::milliseconds(500));

                if (angles_list.size() != 6)
                {
                    std::cout << "角度列表长度不正确，无法保存" << std::endl;
                    cv::putText(color_mat, "Get angles list fail", cv::Point(10, 70), cv::FONT_HERSHEY_SIMPLEX, 0.8, cv::Scalar(0, 0, 255), 2);
                    cv::imshow("image", color_mat);
                    int key = cv::waitKey(1000);
                    break;
                }
                position_list_.push_back(angles_list);
                std::cout << "保存点: " << count;
                for (auto &angle : angles_list)
                {
                    std::cout << angle << " ";
                }
                std::cout << std::endl;
                count++;
                break;
            }

            case 'q':
            {
                // 保存点
                cv::destroyAllWindows();
                std::cout << "q pressed. Exiting..." << std::endl;
                episode_.SetFreeMode(0);
                // is_free_model_.store(false, std::memory_order_release);
                // 输出点到本地文件
                remove_file("calibration/calibration_points.txt");
                std::cout << "保存预采样点到文件 calibration/calibration_points.txt" << std::endl;
                SaveVectorToText(position_list_, "calibration/calibration_points.txt");
                pre_sample = false;
                break;
            }
            default:
                break;
            }
        }
        catch (const std::exception &e)
        {
            std::cerr << e.what() << '\n';
        }
    }
}

void Calibration::Sample()
{
    // 退出自由模式
    episode_.SetFreeMode(0);
    std::this_thread::sleep_for(std::chrono::milliseconds(static_cast<int>(3000)));

    // 从文件加载预采样点
    std::ifstream infile("calibration/calibration_points.txt");
    if (!infile.is_open())
    {
        std::cerr << "无法打开预采样点文件 calibration/calibration_points.txt" << std::endl;
        return;
    }
    std::string line;
    position_list_.clear();
    while (std::getline(infile, line))
    {
        std::vector<double> angles;
        std::stringstream ss(line);
        std::string angle_str;
        while (std::getline(ss, angle_str, ';')) // 使用分号分隔
        {
            if (!angle_str.empty())
            {
                angles.push_back(std::stod(angle_str));
            }
        }

        if (angles.size() == 6) // 确保每个采样点有6个角度
        {
            position_list_.push_back(angles);
        }
    }
    // 输出采样点
    for (auto &angles : position_list_)
    {
        std::cout << "采样点: ";
        for (auto &angle : angles)
        {
            std::cout << angle << " ";
        }
        std::cout << std::endl;
    }

    int sample_index = 0;
    for (int i = 0; i < position_list_.size(); i++)
    {
        std::cout << "采样点 " << i + 1 << ": ";
        // 运动到指定角度
        auto result = episode_.AngleMode(position_list_[i], 1.0);
        double sleep_time = result.get<double>();
        std::this_thread::sleep_for(std::chrono::milliseconds(static_cast<int>(sleep_time * 1000)));
        std::this_thread::sleep_for(std::chrono::milliseconds(static_cast<int>(3000)));
        // 获取彩图

        auto frameset = orbbec_.GetFrameSet();
        cv::Mat color_mat;
        std::shared_ptr<ob::VideoStreamProfile> profile;
        profile = orbbec_.GetColorMat(frameset, color_mat);

        cv::Mat color_mat_copy = color_mat.clone(); // 复制一份用于后续处理
        cv::Mat gray;
        cv::cvtColor(color_mat_copy, gray, cv::COLOR_BGR2GRAY);
        cv::Size board_size;
        board_size.width = board_size_with_;
        board_size.height = board_size_height_;
        std::vector<cv::Point2f> pointbuf;

        auto found = cv::findChessboardCorners(gray, board_size, pointbuf, cv::CALIB_CB_ADAPTIVE_THRESH | cv::CALIB_CB_FAST_CHECK | cv::CALIB_CB_NORMALIZE_IMAGE);
        if (found)
        {
            cv::cornerSubPix(gray, pointbuf, cv::Size(11, 11), cv::Size(-1, -1), cv::TermCriteria(cv::TermCriteria::EPS + cv::TermCriteria::COUNT, 30, 0.0001));
            cv::drawChessboardCorners(color_mat_copy, board_size, pointbuf, found);

            // 获取变换矩阵
            cv::Mat rvec, tvec;
            auto T = episode_.GetT();
            std::cout << "T [ ";
            for (size_t i = 0; i < T.size(); i++)
            {
                std::cout << T[i] << " ";
            }
            std::cout << "]" << std::endl;
            cv::Mat T_mat = cv::Mat(4, 4, CV_64F, T.data());
            std::cout << "Transformation Matrix T: " << T_mat << std::endl;
            T_mat_list_.push_back(T_mat.clone());
            // 在图像上绘制坐标轴
            std::string filename = "calibration/images/" + std::to_string(sample_index) + ".jpg";
            sample_index++;
            cv::imwrite(filename, color_mat);
        }
        cv::imshow("image", color_mat_copy);
        cv::waitKey(3000);
        // 保存图像
    }

    // 打印采样点
    std::cout << "***************************************" << std::endl;
    for (const auto &mat : T_mat_list_)
    {
        std::cout << "Saving matrix of " << mat << std::endl;
    }
    std::cout << "***************************************" << std::endl;
    remove_file("calibration/T_end2base.yml");
    std::string affine_matrix_file = "calibration/T_end2base.yml";
    SaveMatsToYML(affine_matrix_file, T_mat_list_);
    // 输出采样点到文件
}
void Calibration::PerformCalibration()
{

    // 获取相机内参
    // 加载矩阵
    std::vector<cv::Mat> T_end2base = LoadMatsFromYML("calibration/T_end2base.yml");
    if (T_end2base.empty())
    {
        std::cerr << "没有加载到任何矩阵，请检查文件路径和内容。" << std::endl;
        return;
    }
    std::cout << "加载了 " << T_end2base.size() << " 个矩阵。" << std::endl;
    // 打印矩阵信息
    // for (const auto &mat : mats)
    // {
    //     std::cout << "矩阵大小: " << mat.size() << ", 类型: " << mat.type() << std::endl;
    //     std::cout << "矩阵内容: " << mat << std::endl;
    // }
    int image_count = static_cast<int>(T_end2base.size());
    std::vector<std::vector<cv::Point2f>> image_points;
    std::vector<std::vector<cv::Point3f>> object_points(1);

    cv::Size board_size;
    board_size.width = board_size_with_;
    board_size.height = board_size_height_;
    CalcChessboardCorners(board_size, square_size_, object_points[0]);
    int offset = board_size.width - 1;
    float grid_width = square_size_ * (board_size.width - 1);
    // 设置最后一个点的x坐标
    object_points[0][offset].x = object_points[0][0].x + grid_width;
    for (size_t i = 0; i < image_count; i++)
    {
        std::cout << "Processing image " << i + 1 << " of " << image_count << std::endl;
        // 获取图像
        std::string filename = "calibration/images/" + std::to_string(i) + ".jpg";
        cv::Mat image = cv::imread(filename);
        if (image.empty())
        {
            std::cerr << "无法加载图像: " << filename << std::endl;
            continue;
        }
        cv::Mat gray;
        cv::cvtColor(image, gray, cv::COLOR_BGR2GRAY);

        std::vector<cv::Point2f> pointbuf;

        auto found = cv::findChessboardCorners(gray, board_size, pointbuf, cv::CALIB_CB_ADAPTIVE_THRESH | cv::CALIB_CB_FAST_CHECK | cv::CALIB_CB_NORMALIZE_IMAGE);
        if (found)
        {
            cv::cornerSubPix(gray, pointbuf, cv::Size(11, 11), cv::Size(-1, -1), cv::TermCriteria(cv::TermCriteria::EPS + cv::TermCriteria::MAX_ITER, 30, 0.001));
            cv::drawChessboardCorners(image, board_size, pointbuf, found);
            image_points.push_back(pointbuf);
        }
        // cv::imshow("image", image);
        // cv::waitKey(3000);
    }
    object_points.resize(image_points.size(), object_points[0]);
    std::cout << "检测到 " << image_points.size() << " 张图像中的棋盘格角点。"
              << "角点数量: " << image_points[0].size() << std::endl;
    std::cout << "检测到 " << object_points.size() << " 张图像中的棋盘格角点。"
              << "角点数量: " << object_points[0].size() << std::endl;

    // 获取相机内参
    auto frameset = orbbec_.GetFrameSet();
    cv::Mat color_mat;
    std::shared_ptr<ob::VideoStreamProfile> profile;
    profile = orbbec_.GetColorMat(frameset, color_mat);
    // cv::Mat intrinsics = cv::Mat::eye(3, 3, CV_64F);
    // intrinsics.at<double>(0, 0) = profile->getIntrinsic().fx; // fx
    // intrinsics.at<double>(1, 1) = profile->getIntrinsic().fy; // fy
    // intrinsics.at<double>(0, 2) = profile->getIntrinsic().cx; // cx
    // intrinsics.at<double>(1, 2) = profile->getIntrinsic().cy; // cy
    OBCameraIntrinsic ob_intrinsic = profile->getIntrinsic();
    cv::Mat intrinsics = (cv::Mat_<double>(3, 3) << ob_intrinsic.fx, 0, ob_intrinsic.cx,
                          0, ob_intrinsic.fy, ob_intrinsic.cy,
                          0, 0, 1);

    OBCameraDistortion ob_distortion = profile->getDistortion();
    cv::Mat distortion = (cv::Mat_<double>(8, 1) << ob_distortion.k1, ob_distortion.k2, ob_distortion.p1, ob_distortion.p2, ob_distortion.k3, ob_distortion.k4, ob_distortion.k5, ob_distortion.k6);

    std::cout << profile->getDistortion().model << std::endl;

    std::cout << "相机内参矩阵: " << intrinsics << std::endl;
    std::cout << "相机畸变系数: " << distortion << std::endl;
    // 计算每张图像的位姿
    std::vector<cv::Mat> rvecs, tvecs;
    std::vector<cv::Mat> R_board2cameras, tvecs_board2cameras;

    for (size_t i = 0; i < image_points.size(); i++)
    {
        cv::Mat rvec, tvec;
        // 使用 solvePnP 计算位姿
        cv::solvePnP(object_points[i], image_points[i], intrinsics, distortion, rvec, tvec, false, cv::SOLVEPNP_ITERATIVE);
        rvecs.push_back(rvec);
        tvecs.push_back(tvec);
        // std::cout << "图像 " << i + 1 << " 的旋转向量: " << rvec << std::endl;
        // std::cout << "图像 " << i + 1 << " 的平移矩阵: " << tvec << std::endl;
        cv::Mat R;
        cv::Rodrigues(rvec, R); // 将旋转向量转换为旋转矩阵
        R_board2cameras.push_back(R);
        tvecs_board2cameras.push_back(tvec);
        // std::cout << "图像 " << i + 1 << " 的旋转矩阵: " << rvec << std::endl;
        // cv::Mat T = cv::Mat::eye(4, 4, CV_64F);
        // rvec.copyTo(T(cv::Rect(0, 0, 3, 3))); // 将旋转矩阵放入 T 的左上角
        // tvec.copyTo(T(cv::Rect(3, 0, 1, 3))); // 将平移向量放入 T 的最后一列
        // std::cout << "图像 " << i + 1 << " 的变换矩阵 T: " << T << std::endl;
        // 保存变换矩阵
    }
    std::vector<float> reprojection_error;
    // CalculateReprojectionError(object_points, image_points, rvecs, tvecs, intrinsics, distortion, reprojection_error);
    ComputeReprojectionErrors(object_points, image_points,
                              rvecs, tvecs, intrinsics, distortion, reprojection_error);
    // 将转换矩阵拆分为旋转矩阵和平移向量
    std::vector<cv::Mat> R_end2bases, tvecs_end2bases;
    for (size_t i = 0; i < T_end2base.size(); i++)
    {
        cv::Mat R, tvec;
        // 提取旋转矩阵和平移向量

        R = T_end2base[i](cv::Rect(0, 0, 3, 3));    // 提取旋转矩阵
        tvec = T_end2base[i](cv::Rect(3, 0, 1, 3)); // 提取平移向量

        R_end2bases.push_back(R);
        tvecs_end2bases.push_back(tvec);
        // std::cout << "T_end2base[" << i << "] 变换矩阵 : " << T_end2base[i] << std::endl;
        // std::cout << "T_end2base[" << i << "] 的旋转向量: " << R << std::endl;
        // std::cout << "T_end2base[" << i << "] 的平移向量: " << tvec << std::endl;
    }
    // 直行手眼标定
    cv::Mat R_cam2gripper, t_cam2gripper;
    // 使用 cv::calibrateHandEye 进行手眼标定
    cv::calibrateHandEye(R_end2bases, tvecs_end2bases,
                         R_board2cameras, tvecs_board2cameras,
                         R_cam2gripper, t_cam2gripper,
                         cv::CALIB_HAND_EYE_TSAI);
    std::cout << "手眼标定结果: " << std::endl;
    std::cout << "R_cam2gripper: " << R_cam2gripper << std::endl;
    std::cout << "t_cam2gripper: " << t_cam2gripper << std::endl;
    // 构建齐次矩阵

    cv::Mat T_cam2gripper = cv::Mat::eye(4, 4, CV_64F);
    R_cam2gripper.copyTo(T_cam2gripper(cv::Rect(0, 0, 3, 3))); // 将旋转矩阵放入 T 的左上角
    t_cam2gripper.copyTo(T_cam2gripper(cv::Rect(3, 0, 1, 3))); // 将平移向量放入 T 的最后一列
    std::cout << "T_cam2gripper: " << T_cam2gripper << std::endl;
    // 保存手眼标定结果
    remove_file("calibration/hand_eye_calibration.yml");
    cv::FileStorage fs("calibration/hand_eye_calibration.yml", cv::FileStorage::WRITE);
    fs << "R_cam2gripper" << R_cam2gripper;
    fs << "t_cam2gripper" << t_cam2gripper;
    fs << "T_cam2gripper" << T_cam2gripper;           // 保存齐次变换矩阵
    fs << "intrinsics" << intrinsics;                 // 保存相机内参矩阵
    fs << "distortion" << distortion;                 // 保存相机畸变系数
    fs << "reprojection_error" << reprojection_error; // 保存重投影误差
    fs.release();
    std::cout << "手眼标定结果已保存到 calibration/hand_eye_calibration.yml" << std::endl;
}

bool Calibration::CalibrationCamera()
{
    std::vector<std::vector<cv::Point2f>> image_points;
    std::vector<std::vector<cv::Point3f>> object_points(1);

    cv::Size board_size, image_size;
    board_size.width = board_size_with_;
    board_size.height = board_size_height_;
    CalcChessboardCorners(board_size, square_size_, object_points[0]);
    int offset = board_size.width - 1;
    float grid_width = square_size_ * (board_size.width - 1);
    // 设置最后一个点的x坐标
    object_points[0][offset].x = object_points[0][0].x + grid_width;

    for (size_t i = 0; i < 23; i++)
    {
        std::cout << "Processing image " << i + 1 << " of " << 23 << std::endl;
        // 获取图像
        std::string filename = "calibration/images/" + std::to_string(i) + ".jpg";
        cv::Mat image = cv::imread(filename);
        if (image.empty())
        {
            std::cerr << "无法加载图像: " << filename << std::endl;
            continue;
        }
        cv::Mat gray;
        cv::cvtColor(image, gray, cv::COLOR_BGR2GRAY);

        std::vector<cv::Point2f> pointbuf;

        auto found = cv::findChessboardCorners(gray, board_size, pointbuf, cv::CALIB_CB_ADAPTIVE_THRESH | cv::CALIB_CB_FAST_CHECK | cv::CALIB_CB_NORMALIZE_IMAGE);
        if (found)
        {
            cv::cornerSubPix(gray, pointbuf, cv::Size(11, 11), cv::Size(-1, -1), cv::TermCriteria(cv::TermCriteria::EPS + cv::TermCriteria::MAX_ITER, 30, 0.001));
            cv::drawChessboardCorners(image, board_size, pointbuf, found);
            image_points.push_back(pointbuf);
            image_size = image.size();
        }
        // cv::imshow("image", image);
        // cv::waitKey(3000);
    }
    object_points.resize(image_points.size(), object_points[0]);
    std::cout << "检测到 " << image_points.size() << " 张图像中的棋盘格角点。"
              << "角点数量: " << image_points[0].size() << std::endl;
    std::cout << "检测到 " << object_points.size() << " 张图像中的棋盘格角点。"
              << "角点数量: " << object_points[0].size() << std::endl;
    int iFixedPoint = -1; // 不固定点
    cv::Mat cameraMatrix, distCoeffs;
    std::vector<cv::Mat> rvecs, tvecs;
    std::vector<cv::Point3f> newObjPoints = object_points[0];
    double rms = cv::calibrateCameraRO(object_points, image_points, image_size, iFixedPoint,
                                       cameraMatrix, distCoeffs, rvecs, tvecs, newObjPoints,
                                       cv::CALIB_USE_LU);
    printf("RMS error reported by calibrateCamera: %g\n", rms);

    std::cout << "New board corners: " << std::endl;
    std::cout << newObjPoints[0] << std::endl;
    std::cout << newObjPoints[board_size.width - 1] << std::endl;
    std::cout << newObjPoints[board_size.width * (board_size.height - 1)] << std::endl;
    std::cout << newObjPoints.back() << std::endl;

    object_points.clear();
    object_points.resize(image_points.size(), newObjPoints);
    std::vector<float> reprojErrs;
    ComputeReprojectionErrors(object_points, image_points,
                              rvecs, tvecs, cameraMatrix, distCoeffs, reprojErrs);

    std::cout << "相机内参矩阵: " << cameraMatrix << std::endl;
    std::cout << "相机畸变系数: " << distCoeffs << std::endl;

    // 保存手眼标定结果
    // cv::FileStorage fs("calibration/hand_eye_calibration.yml", cv::FileStorage::WRITE);
    // fs << "R_cam2gripper" << R_cam2gripper;
    // fs << "t_cam2gripper" << t_cam2gripper;
    // fs << "T_cam2gripper" << T_cam2gripper;           // 保存齐次变换矩阵
    // fs << "intrinsics" << intrinsics;                 // 保存相机内参矩阵
    // fs << "distortion" << distortion;                 // 保存相机畸变系数
    // fs << "reprojection_error" << reprojection_error; // 保存重投影误差
    // fs.release();
    // std::cout << "手眼标定结果已保存到 calibration/hand_eye_calibration.yml" << std::endl;

    return true;
}

double Calibration::ComputeReprojectionErrors(
    const std::vector<std::vector<cv::Point3f>> &objectPoints,
    const std::vector<std::vector<cv::Point2f>> &imagePoints,
    const std::vector<cv::Mat> &rvecs, const std::vector<cv::Mat> &tvecs,
    const cv::Mat &cameraMatrix, const cv::Mat &distCoeffs,
    std::vector<float> &perViewErrors)
{
    std::vector<cv::Point2f> imagePoints2;
    int i, totalPoints = 0;
    double totalErr = 0, err;
    perViewErrors.resize(objectPoints.size());

    // for (i = 0; i < (int)objectPoints.size(); i++)
    // {
    //     cv::projectPoints(cv::Mat(objectPoints[i]), rvecs[i], tvecs[i],
    //                       cameraMatrix, distCoeffs, imagePoints2);
    //     err = cv::norm(cv::Mat(imagePoints[i]), cv::Mat(imagePoints2), cv::NORM_L2);
    //     int n = (int)objectPoints[i].size();
    //     perViewErrors[i] = (float)std::sqrt(err * err / n);
    //     totalErr += err * err;
    //     totalPoints += n;
    // }

    for (i = 0; i < (int)objectPoints.size(); i++)
    {
        cv::projectPoints(cv::Mat(objectPoints[i]), rvecs[i], tvecs[i],
                          cameraMatrix, distCoeffs, imagePoints2);
        err = cv::norm(cv::Mat(imagePoints[i]), cv::Mat(imagePoints2), cv::NORM_L2) / imagePoints2.size();

        perViewErrors[i] = err;
        totalErr += err;
    }

    std::cout << "平均重投影误差: " << totalErr / objectPoints.size() << std::endl;
    std::cout << "每张图像的重投影误差: ";
    for (const auto &error : perViewErrors)
    {
        std::cout << error << " ";
    }
    std::cout << std::endl;

    return std::sqrt(totalErr / totalPoints);
}

bool Calibration::GetArucoCenter(cv::Point3f &point3f, cv::Mat &mat)
{
    auto frameset = orbbec_.GetFrameSet();

    cv::Mat color_mat;
    std::shared_ptr<ob::VideoStreamProfile> profile;
    profile = orbbec_.GetColorMat(frameset, color_mat);

    OBCameraIntrinsic ob_intrinsic = profile->getIntrinsic();
    cv::Mat intrinsics = (cv::Mat_<double>(3, 3) << ob_intrinsic.fx, 0, ob_intrinsic.cx,
                          0, ob_intrinsic.fy, ob_intrinsic.cy,
                          0, 0, 1);

    OBCameraDistortion ob_distortion = profile->getDistortion();
    cv::Mat distortion = (cv::Mat_<double>(8, 1) << ob_distortion.k1, ob_distortion.k2, ob_distortion.p1, ob_distortion.p2, ob_distortion.k3, ob_distortion.k4, ob_distortion.k5, ob_distortion.k6);

    std::vector<std::vector<cv::Point2f>> corners, rejected;
    std::vector<int> ids;
    // detect markers and estimate pose
    aruco_detector_.detectMarkers(color_mat, corners, ids, rejected);
    std::vector<cv::Vec3d> rvecs, tvecs;

    if (ids.size() > 0)
    {
        cv::aruco::estimatePoseSingleMarkers(corners, marker_length_, intrinsics, distortion, rvecs, tvecs);
        cv::aruco::drawDetectedMarkers(color_mat, corners, ids);
        for (size_t i = 0; i < corners.size(); i++)
        {
            cv::Mat rvec = cv::Mat(rvecs[i]);
            cv::Mat tvec = cv::Mat(tvecs[i]);
            std::vector<cv::Point2f> corner = corners[i];
            OBPoint2f src;
            src.x = float((corner[0].x + corner[2].x) / 2);
            src.y = float((corner[0].y + corner[2].y) / 2);
            OBPoint3f target;
            auto align_frame_set = orbbec_.GetFrameSetSyncC2DAlign(frameset);
            if (orbbec_.Transformation2dto3d(src, target, align_frame_set))
            {
                // 将目标点转换为三维点
                point3f.x = target.x;
                point3f.y = target.y;
                point3f.z = target.z;
                if (target.z > 0)
                {
                    std::cout << "Aruco marker center: " << point3f << std::endl;
                    cv::drawFrameAxes(color_mat, intrinsics, distortion, rvec, tvec, marker_length_);
                    cv::circle(color_mat, cv::Point(int(src.x), int(src.y)), 5, cv::Scalar(0, 0, 255), -1);
                }
                else
                {
                    std::cout << "Aruco marker center z coordinate is negative, skipping this marker." << std::endl;
                    return false;
                }
            }

            else
            {
                std::cout << "get aruco marker center fail " << std::endl;
                return false;
            }
            // break;
        }
    }
    mat = color_mat.clone(); // 返回处理后的彩图
    return true;             // 成功获取 ArUco 中心点
}

bool Calibration::GetArucoPose(std::vector<cv::Mat> &T_list)
{
    auto frameset = orbbec_.GetFrameSet();
    cv::Mat color_mat;
    std::shared_ptr<ob::VideoStreamProfile> profile;
    profile = orbbec_.GetColorMat(frameset, color_mat);

    OBCameraIntrinsic ob_intrinsic = profile->getIntrinsic();
    cv::Mat intrinsics = (cv::Mat_<double>(3, 3) << ob_intrinsic.fx, 0, ob_intrinsic.cx,
                          0, ob_intrinsic.fy, ob_intrinsic.cy,
                          0, 0, 1);

    OBCameraDistortion ob_distortion = profile->getDistortion();
    cv::Mat distortion = (cv::Mat_<double>(8, 1) << ob_distortion.k1, ob_distortion.k2, ob_distortion.p1, ob_distortion.p2, ob_distortion.k3, ob_distortion.k4, ob_distortion.k5, ob_distortion.k6);

    std::vector<std::vector<cv::Point2f>> corners, rejected;
    std::vector<int> ids;
    // detect markers and estimate pose
    aruco_detector_.detectMarkers(color_mat, corners, ids, rejected);
    std::vector<cv::Vec3d> rvecs, tvecs;

    if (ids.size() > 0)
    {
        cv::aruco::estimatePoseSingleMarkers(corners, marker_length_, intrinsics, distortion, rvecs, tvecs);
        cv::aruco::drawDetectedMarkers(color_mat, corners, ids);
        for (size_t i = 0; i < corners.size(); i++)
        {
            cv::Mat rvec = cv::Mat(rvecs[i]);
            cv::Mat tvec = cv::Mat(tvecs[i]);
            std::vector<cv::Point2f> corner = corners[i];
            OBPoint2f src;
            src.x = float((corner[0].x + corner[2].x) / 2);
            src.y = float((corner[0].y + corner[2].y) / 2);
            std::cout << "Aruco marker 2d  center: x " << src.x << " y " << src.y << std::endl;
            OBPoint3f target;
            auto align_frame_set = orbbec_.GetFrameSetSyncC2DAlign(frameset);
            if (orbbec_.Transformation2dto3d(src, target, align_frame_set))
            {
                // 将目标点转换为三维点

                if (target.z > 0)
                {
                    std::cout << "Aruco marker 3d center: x " << target.x << " y " << target.y << " z " << target.z << std::endl;
                    cv::drawFrameAxes(color_mat, intrinsics, distortion, rvec, tvec, marker_length_);
                    cv::circle(color_mat, cv::Point(int(src.x), int(src.y)), 5, cv::Scalar(0, 0, 255), -1);
                }
                else
                {
                    std::cout << "Aruco marker center z coordinate is negative, skipping this marker." << std::endl;
                    return false;
                }

                // 获取姿态
                cv::Mat R;
                cv::Rodrigues(rvec, R); // 将旋转向量转换为旋转矩阵
                std::cout << "Aruco marker rotation matrix: " << R << std::endl;
                cv::Mat T = cv::Mat::eye(4, 4, CV_64F);
                R.copyTo(T(cv::Rect(0, 0, 3, 3)));    // 将旋转矩阵放入 T 的左上角
                tvec.copyTo(T(cv::Rect(3, 0, 1, 3))); // 将平移向量放入 T 的最后一列
                std::cout << "Aruco marker transformation matrix T: " << T << std::endl;
                T_list.push_back(T.clone()); // 保存变换矩阵
            }
            else
            {
                std::cout << "get aruco marker center fail " << std::endl;
                return false;
            }
        }
    }
    cv::imshow("image", color_mat);
    cv::waitKey(0);
    return true; // 成功获取 ArUco 中心点
}

// 测试标定
void Calibration::TestCalibration(const std::string &file_path)
{
    cv::FileStorage fs(file_path, cv::FileStorage::READ);
    if (!fs.isOpened())
    {
        std::cerr << "无法打开标定结果文件: " << file_path << std::endl;
        return;
    }
    cv::Mat T_cam2gripper;
    fs["T_cam2gripper"] >> T_cam2gripper;
    fs.release();
    std::cout << "T_cam2gripper:\n"
              << T_cam2gripper << std::endl;

    std::random_device rd;
    unsigned seed = rd() | static_cast<unsigned>(
                               std::chrono::high_resolution_clock::now()
                                   .time_since_epoch()
                                   .count());

    std::mt19937_64 engine(seed);
    std::uniform_int_distribution<int> dist_x(250, 380);
    std::uniform_int_distribution<int> dist_y(-220, 220);
    for (size_t i = 0; i < 10; i++)
    {
        // 移动到观察位置
        auto result = episode_.MoveXYZRotation({260, 0, 300}, {180, 0, 90}, "xyz", 1);
        double sleep_time = result.get<double>();
        std::this_thread::sleep_for(std::chrono::milliseconds(static_cast<int>(sleep_time * 1000)));

        std::this_thread::sleep_for(std::chrono::milliseconds(static_cast<int>(1000)));
        auto T = episode_.GetT();
        std::cout << "T [ ";
        for (size_t i = 0; i < T.size(); i++)
        {
            std::cout << T[i] << " ";
        }
        std::cout << "]" << std::endl;
        cv::Mat T_mat = cv::Mat(4, 4, CV_64F, T.data());
        std::cout << "Transformation Matrix T: " << T_mat << std::endl;

        cv::Mat T_end2base = T_mat.clone();
        std::cout << "T_end2base:\n"
                  << T_end2base << std::endl;

        // 获取变换矩阵
        cv::Point3f camera_point3f;
        cv::Mat color_mat;
        if (GetArucoCenter(camera_point3f, color_mat))
        {
            cv::imshow("image", color_mat);
            cv::waitKey(1000);
            std::cout << "相机坐标 "
                      << "x " << camera_point3f.x << " y " << camera_point3f.y << " z " << camera_point3f.z << std::endl;
            cv::Mat p_camera_homo = (cv::Mat_<double>(4, 1) << camera_point3f.x, camera_point3f.y, camera_point3f.z, 1.0);
            std::cout << "p_camera_homo:\n"
                      << p_camera_homo << std::endl;
            std::cout << "T_cam2gripper:\n"
                      << T_cam2gripper << std::endl;
            cv::Mat p_end = T_cam2gripper * p_camera_homo;
            std::cout << "p_end:\n"
                      << p_end << std::endl;

            cv::Mat p_base = T_end2base * p_end;
            std::cout << "T_end2base:\n"
                      << T_end2base << std::endl;
            std::cout << "p_base:\n"
                      << p_base << std::endl;
            cv::Point3f robot_point3f;

            robot_point3f.x = p_base.at<double>(0);
            robot_point3f.y = p_base.at<double>(1);
            robot_point3f.z = p_base.at<double>(2);
            std::cout << "机器人坐标 "
                      << "x " << robot_point3f.x << " y " << robot_point3f.y << " z " << robot_point3f.z << std::endl;
            // 移动到目标上方

            std::cout << " 开始移动" << std::endl;
            auto result = episode_.MoveXYZRotation({robot_point3f.x, robot_point3f.y, robot_point3f.z + sucker_length_ + 100}, {180, 0, 90}, "xyz", 1);
            double sleep_time = result.get<double>();
            std::this_thread::sleep_for(std::chrono::milliseconds(static_cast<int>(sleep_time * 1000)));

            // // 移动到目标
            result = episode_.MoveXYZRotation({robot_point3f.x, robot_point3f.y, robot_point3f.z + sucker_length_ - 8}, {180, 0, 90}, "xyz", 1);
            sleep_time = result.get<double>();
            std::this_thread::sleep_for(std::chrono::milliseconds(static_cast<int>(sleep_time * 1000)));

            // // 开启气泵
            episode_.GripperOn();

            // // 移动到目标上方
            result = episode_.MoveXYZRotation({robot_point3f.x, robot_point3f.y, robot_point3f.z + sucker_length_ + 100}, {180, 0, 90}, "xyz", 1);
            sleep_time = result.get<double>();
            std::this_thread::sleep_for(std::chrono::milliseconds(static_cast<int>(sleep_time * 1000)));

            int random_num_x = dist_x(engine);
            int random_num_y = dist_y(engine);

            result = episode_.MoveXYZRotation({static_cast<double>(random_num_x), static_cast<double>(random_num_y), sucker_length_ + 100}, {180, 0, 90}, "xyz", 1);
            sleep_time = result.get<double>();
            std::this_thread::sleep_for(std::chrono::milliseconds(static_cast<int>(sleep_time * 1000)));
            episode_.GripperOff();
            std::this_thread::sleep_for(std::chrono::seconds(1));
        }
    }
}

// 测试标定
void Calibration::TestCalibrationServoGripper(const std::string &file_path)
{
    cv::FileStorage fs(file_path, cv::FileStorage::READ);
    if (!fs.isOpened())
    {
        std::cerr << "无法打开标定结果文件: " << file_path << std::endl;
        return;
    }
    cv::Mat T_cam2gripper;
    fs["T_cam2gripper"] >> T_cam2gripper;

    std::cout << "T_cam2gripper:\n"
              << T_cam2gripper << std::endl;
    cv::Mat R_cam2gripper;

    fs["R_cam2gripper"] >> R_cam2gripper;
    std::cout << "R_cam2gripper:\n"
              << R_cam2gripper << std::endl;
    fs.release();
    RotationMatrixToEulerAngles(R_cam2gripper, XYZ);
    RotationMatrixToEulerAngles(R_cam2gripper, ZYX);

    // 创建一个单位矩阵
    cv::Mat T_servo2end = cv::Mat::eye(4, 4, CV_64F);
    // 设置平移向量
    T_servo2end.at<double>(0, 3) = 0;   // x 平移
    T_servo2end.at<double>(1, 3) = 0;   //  y 平移
    T_servo2end.at<double>(2, 3) = 120; // z 平移
    // T_servo2end.at<double>(2, 3) = 0; // z 平移
    T_servo2end.at<double>(3, 3) = 1; // 齐次坐标的最后一行
    std::cout << "T_servo2end:\n"
              << T_servo2end << std::endl;

    // 移动到观察位置
    auto result = episode_.MoveXYZRotation({260, 0, 300}, {180, 0, 90}, "xyz", 1);
    double sleep_time = result.get<double>();
    std::this_thread::sleep_for(std::chrono::milliseconds(static_cast<int>(sleep_time * 1000)));

    std::this_thread::sleep_for(std::chrono::milliseconds(static_cast<int>(1000)));
    auto T = episode_.GetT();
    std::cout << "T [ ";
    for (size_t i = 0; i < T.size(); i++)
    {
        std::cout << T[i] << " ";
    }
    std::cout << "]" << std::endl;
    cv::Mat T_mat = cv::Mat(4, 4, CV_64F, T.data());
    std::cout << "Transformation Matrix T: " << T_mat << std::endl;

    cv::Mat T_end2base = T_mat.clone();
    std::cout << "T_end2base:\n"
              << T_end2base << std::endl;
    // 提取旋转矩阵和平移向量
    cv::Mat R_end2base = T_end2base(cv::Rect(0, 0, 3, 3)); // 提取旋转矩阵
    cv::Mat t_end2base = T_end2base(cv::Rect(3, 0, 1, 3)); // 提取平移向量
    std::cout << "R_end2base:\n"
              << R_end2base << std::endl;
    std::cout << "t_end2base:\n"
              << t_end2base << std::endl;
    RotationMatrixToEulerAngles(R_end2base, XYZ);
    RotationMatrixToEulerAngles(R_end2base, ZYX);

    // 获取变换矩阵
    std::vector<cv::Mat>
        T_aruco_list;
    if (GetArucoPose(T_aruco_list))
    {

        for (auto &T_aruco : T_aruco_list)
        {
            std::cout << "T_aruco:\n"
                      << T_aruco << std::endl;
            cv::Mat R_aruco = T_aruco(cv::Rect(0, 0, 3, 3)); // 提取旋转矩阵
            cv::Mat t_aruco = T_aruco(cv::Rect(3, 0, 1, 3)); // 提取平移向量
            std::cout << "R_aruco:\n"
                      << R_aruco << std::endl;
            std::cout << "t_aruco:\n"
                      << t_aruco << std::endl;
            RotationMatrixToEulerAngles(R_aruco, XYZ);
            RotationMatrixToEulerAngles(R_aruco, ZYX);

            cv::Mat T_aruco2end = T_cam2gripper * T_aruco; // 计算末端到Aruco标记的变换矩阵
            std::cout << "T_aruco2end:\n"
                      << T_aruco2end << std::endl;
            cv::Mat T_aruco2base = T_end2base * T_aruco2end; // 计算末端到Aruco标记的变换矩阵
            std::cout << "T_aruco2base:\n"
                      << T_aruco2base << std::endl;
            std::cout << "T_servo2end.inv():\n"
                      << T_servo2end.inv() << std::endl;
            cv::Mat R_aruco2base = T_aruco2base(cv::Rect(0, 0, 3, 3)); // 提取旋转矩阵
            RotationMatrixToEulerAngles(R_aruco2base, XYZ);
            RotationMatrixToEulerAngles(R_aruco2base, ZYX);

            //  cv::Mat T_aruco2base_new = T_aruco2base * T_servo2end.inv(); // 计算末端到Aruco标记的变换矩阵
            cv::Mat T_aruco2base_new = T_aruco2base * T_servo2end; // 计算末端到Aruco标记的变换矩阵
            std::cout << "T_aruco2base_new:\n"
                      << T_aruco2base_new << std::endl;
            cv::Point3f robot_point3f;
            robot_point3f.x = T_aruco2base_new.at<double>(0, 3);
            robot_point3f.y = T_aruco2base_new.at<double>(1, 3);
            robot_point3f.z = T_aruco2base_new.at<double>(2, 3);
            std::cout << "机器人坐标 "
                      << "x " << robot_point3f.x << " y " << robot_point3f.y << " z " << robot_point3f.z << std::endl;

            cv::Mat T_x180 = cv::Mat::eye(4, 4, CV_64F);
            // 设置平移向量 绕x轴旋转180度
            T_x180.at<double>(1, 1) = -1;
            T_x180.at<double>(2, 2) = -1;
            std::cout << "T_x180:\n"
                      << T_x180 << std::endl;
            cv::Mat T_aruco2base_new_x180 = T_x180 * T_aruco2base_new;
            // cv::Mat T_aruco2base_new_x180 = T_aruco2base_new * T_x180;
            // 根据旋转矩阵计算欧拉角
            cv::Mat R = T_aruco2base_new_x180(cv::Rect(0, 0, 3, 3)); // 提取旋转矩阵

            cv::Vec3d orientation = RotationMatrixToEulerAngles(R, ZYX);
            std::cout << orientation[0] << " " << orientation[1] << " " << orientation[2] << std::endl;
            auto result = episode_.MoveXYZRotation({robot_point3f.x, robot_point3f.y, robot_point3f.z + 5}, {orientation[0], orientation[1], orientation[2]}, "xyz", 1);
            // auto result = episode_.MoveXYZRotation({robot_point3f.x, robot_point3f.y, robot_point3f.z + 100}, {orientation[2], orientation[1], orientation[0]}, "zyx", 1);
            double sleep_time = result.get<double>();
            std::this_thread::sleep_for(std::chrono::milliseconds(static_cast<int>(sleep_time * 1000)));

            // // // 移动到目标
            // result = episode_.MoveXYZRotation({robot_point3f.x, robot_point3f.y, robot_point3f.z + sucker_length_ - 8}, {180, 0, 90}, "xyz", 1);
            // sleep_time = result.get<double>();
            // std::this_thread::sleep_for(std::chrono::milliseconds(static_cast<int>(sleep_time * 1000)));

            // // // 开启气泵
            // episode_.GripperOn();

            // // // 移动到目标上方
            // result = episode_.MoveXYZRotation({robot_point3f.x, robot_point3f.y, robot_point3f.z + sucker_length_ + 100}, {180, 0, 90}, "xyz", 1);
            // sleep_time = result.get<double>();
            // std::this_thread::sleep_for(std::chrono::milliseconds(static_cast<int>(sleep_time * 1000)));

            // int random_num_x = dist_x(engine);
            // int random_num_y = dist_y(engine);

            // result = episode_.MoveXYZRotation({static_cast<double>(random_num_x), static_cast<double>(random_num_y), sucker_length_ + 100}, {180, 0, 90}, "xyz", 1);
            // sleep_time = result.get<double>();
            // std::this_thread::sleep_for(std::chrono::milliseconds(static_cast<int>(sleep_time * 1000)));
            // episode_.GripperOff();
            // std::this_thread::sleep_for(std::chrono::seconds(3));
        }
    }
}
// void Calibration::GetJointAngles()
// {
//     while (true)
//     {
//         try
//         {
//             if (!is_free_model_.load(std::memory_order_acquire))
//             {
//                 break;
//             }

//             if (!is_reading_.load(std::memory_order_acquire))
//             {
//                 auto angles = episode_.GetMotorAngles();
//                 // std::cout << "获取角度 " << angles.dump() << std::endl;
//                 if (angles.is_array())
//                     angles_list_ = angles.get<std::vector<double>>();
//                 std::this_thread::sleep_for(std::chrono::milliseconds(500));
//             }
//         }
//         catch (const std::exception &e)
//         {
//             std::cerr << e.what() << "get angles error" << '\n';
//         }
//     }
//     std::cout << "exit()" << std::endl;
// }
double Calibration::EvaluateImageQuality(cv::Mat &mat)
{

    cv::Mat gray;
    cv::cvtColor(mat, gray, cv::COLOR_BGR2GRAY);
    cv::Mat laplacian;
    cv::Laplacian(gray, laplacian, CV_64F); // 使用double精度

    // 计算均值和标准差
    cv::Scalar mean, stddev;
    cv::meanStdDev(laplacian, mean, stddev);

    // 方差作为清晰度指标 (方差越大，图像越清晰)
    double variance = stddev.val[0] * std::pow(255, 2);

    return variance;
}

void Calibration::SaveVectorToText(const std::vector<std::vector<double>> &data, const std::string &filename)
{
    std::ofstream outfile(filename);
    if (!outfile)
    {
        std::cerr << "Error opening file: " << filename << std::endl;
        return;
    }
    std::cout << "Saved " << data.size() << " rows to " << filename << std::endl;
    for (const auto &row : data)
    {
        for (size_t i = 0; i < row.size(); ++i)
        {
            outfile << row[i];
            if (i < row.size() - 1)
                outfile << ";"; // 用空格分隔数值
        }
        outfile << std::endl; // 每行结束换行
    }
    std::cout << "Saved " << data.size() << " rows to " << filename << std::endl;
}

void Calibration::SaveMatsToYML(const std::string &filename, const std::vector<cv::Mat> &mats)
{
    cv::FileStorage fs(filename, cv::FileStorage::WRITE);
    fs << "mat_sequence"
       << "["; // 开始序列

    for (const auto &mat : mats)
    {
        std::cout << "Saving matrix of " << mat << std::endl;
        fs << mat; // 直接写入矩阵，不需键名
    }

    fs << "]"; // 结束序列
    fs.release();
}

std::vector<cv::Mat> Calibration::LoadMatsFromYML(const std::string &filename)
{
    std::vector<cv::Mat> mats;
    cv::FileStorage fs(filename, cv::FileStorage::READ);
    cv::FileNode node = fs["mat_sequence"];
    if (node.type() == cv::FileNode::SEQ)
    {
        for (cv::FileNodeIterator it = node.begin(); it != node.end(); ++it)
        {
            cv::Mat mat;
            *it >> mat;
            mats.push_back(mat);
        }
    }
    fs.release();
    return mats;
}

void Calibration::CalcChessboardCorners(cv::Size board_size, float square_size, std::vector<cv::Point3f> &corners)
{
    corners.resize(0);
    for (int i = 0; i < board_size.height; i++)
    {
        for (int j = 0; j < board_size.width; j++)
        {
            corners.push_back(cv::Point3f(float(j * square_size),
                                          float(i * square_size), 0));
        }
    }
}

// 计算重投影误差
// void Calibration::CalculateReprojectionError(std::vector<std::vector<cv::Point3f>> object_points, std::vector<std::vector<cv::Point2f>> image_points, std::vector<cv::Mat> rvecs, std::vector<cv::Mat> tvecs, const cv::Mat &camera_matrix, const cv::Mat &dist_coeffs, std::vector<double> &reprojection_errors)
// // void Calibration::CalculateReprojectionError(std::vector<std::vector<cv::Point3f>> object_points, std::vector<std::vector<cv::Point2f>> image_points, std::vector<cv::Mat> rvecs, std::vector<cv::Mat> tvecs, const cv::Mat &camera_matrix, const std::vector<double> dist_coeffs, std::vector<double> &reprojection_errors)
// {
//     reprojection_errors.clear();
//     for (size_t i = 0; i < object_points.size(); i++)
//     {
//         std::vector<cv::Point2f> projected_points;
//         cv::projectPoints(object_points[i], rvecs[i], tvecs[i], camera_matrix, dist_coeffs, projected_points);
//         double error = cv::norm(image_points[i], projected_points, cv::NORM_L2);
//         reprojection_errors.push_back(error);
//     }
//     // 输出平均重投影误差
//     double total_error = std::accumulate(reprojection_errors.begin(), reprojection_errors.end(), 0.0);
//     double mean_error = total_error / reprojection_errors.size();
//     std::cout << "平均重投影误差: " << mean_error << std::endl;
//     std::cout << "每张图像的重投影误差: ";
//     for (const auto &error : reprojection_errors)
//     {
//         std::cout << error << " ";
//     }
//     std::cout << std::endl;
// }

// 旋转顺序枚举

// 旋转矩阵转欧拉角（直接计算）
cv::Vec3d Calibration::RotationMatrixToEulerAngles(const cv::Mat &R, RotationOrder order)
{
    CV_Assert(R.rows == 3 && R.cols == 3);

    double r00 = R.at<double>(0, 0), r01 = R.at<double>(0, 1), r02 = R.at<double>(0, 2);
    double r10 = R.at<double>(1, 0), r11 = R.at<double>(1, 1), r12 = R.at<double>(1, 2);
    double r20 = R.at<double>(2, 0), r21 = R.at<double>(2, 1), r22 = R.at<double>(2, 2);

    double x, y, z; // 欧拉角（弧度）

    const double PI = CV_PI;
    const double singularityThreshold = 0.99999;

    switch (order)
    {
    // ZYX顺序 (yaw-pitch-roll)
    case XYZ:
    {
        if (std::abs(r20) > singularityThreshold)
        {
            // 处理万向锁 (pitch ≈ ±90°)
            z = 0;
            if (r20 > 0)
            {
                y = -PI / 2;
                x = -std::atan2(-r12, r11);
            }
            else
            {

                y = PI / 2;
                x = std::atan2(-r12, r11);
            }
        }
        else
        {
            y = -std::asin(r20); // 确保在[-1,1]范围内
            float cos_y = std::cos(y);
            float sin_x = r21 / cos_y;
            float cos_x = r22 / cos_y;
            x = std::atan2(sin_x, cos_x); // 计算roll
            float sin_z = r10 / cos_y;
            float cos_z = r00 / cos_y;
            z = std::atan2(sin_z, cos_z); // 计算yaw
        }
        break;
    }

    // XYZ顺序 (roll-pitch-yaw)
    case ZYX:
    {
        if (std::abs(r02) > singularityThreshold)
        {
            // 处理万向锁
            x = 0;
            if (r02 > 0)
            {
                y = -PI / 2;
                x = std::atan2(r10, r11);
            }
            else
            {
                y = -PI / 2;
                x = std::atan2(r10, r11);
            }
        }
        else
        {
            y = std::atan2(r20, std::sqrt(r00 * r00 + r01 * r01)); // 确保在[-1,1]范围内

            x = std::atan2(-r12, r22);
            z = std::atan2(-r01, r00);
        }
        break;
    }

        // 其他顺序可根据需要添加...

    default:
        throw std::runtime_error("不支持的旋转顺序");
    }

    std::cout << "欧拉角 (度): " << cv::Vec3d(x, y, z) * 180 / CV_PI << std::endl;
    return cv::Vec3d(x, y, z) * 180 / CV_PI; // roll, pitch, yaw
}
