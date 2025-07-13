#include "calibration.h"

#include <iostream>
#include <random> // C++11 随机数库
#include <chrono> // 时间相关功能

Calibration::Calibration(/* args */)
{
}

Calibration::~Calibration()
{
    episode_.GripperOff();
}

bool Calibration::InitDevice()
{
    // 初始化设备
    // 这里可以添加设备初始化的代码

    orbbec_.OpenDevice(); // 替换为实际的序列号
    cv::aruco::Dictionary dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_5X5_50);
    cv::aruco::DetectorParameters detector_params;
    cv::aruco::ArucoDetector detector(dictionary, detector_params);
    aruco_detector_ = detector; // 将检测器赋值给成员变量
    if (episode_.Connect("127.0.0.1", 12345))
    {
        std::cout << "Episode 连接成功 移动到初始位置" << std::endl;
    }

    return true;
}
// 进行标定 虚函数
void Calibration::PreSample()
{
    // 进行标定的具体实现
    // 这里可以添加标定算法的代码
    // 例如，使用预采样点进行标定计算
}
bool Calibration::GetArucoCenter(cv::Point3f &point3f)
{
    auto frameset = orbbec_.GetFrameSet();
    cv::Mat color_mat;
    std::shared_ptr<ob::VideoStreamProfile> profile;
    profile = orbbec_.GetColorMat(frameset, color_mat);
    cv::Mat intrinsics = cv::Mat::eye(3, 3, CV_64F);
    intrinsics.at<double>(0, 0) = profile->getIntrinsic().fx; // fx
    intrinsics.at<double>(1, 1) = profile->getIntrinsic().fy; // fy
    intrinsics.at<double>(0, 2) = profile->getIntrinsic().cx; // cx
    intrinsics.at<double>(1, 2) = profile->getIntrinsic().cy; // cy
    cv::Mat distortion(1, 5, CV_64F);                         // 1行5列的双精度浮点矩阵
    distortion.at<double>(0) = profile->getDistortion().k1;   // k1
    distortion.at<double>(1) = profile->getDistortion().k2;   // k2
    distortion.at<double>(2) = profile->getDistortion().p1;   // p1
    distortion.at<double>(3) = profile->getDistortion().p2;   // p2
    distortion.at<double>(4) = profile->getDistortion().k3;   // k3
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
            if (orbbec_.Transformation2dto3d(src, target, frameset))
            {
                point3f.x = target.x;
                point3f.y = target.y;
                point3f.z = target.z;
                if (target.z <= 0.0f)
                {
                    std::cerr << "Z coordinate is less than or equal to 0, skipping point." << std::endl;
                    return false; // 如果 z 小于等于 0，返回 false
                }
            }
            else
            {
                return false; // 转换失败
            }
            // cv::drawFrameAxes(color_mat, intrinsics, distortion, rvec, tvec, marker_length_);
            cv::circle(color_mat, cv::Point(int(src.x), int(src.y)), 5, cv::Scalar(0, 0, 255), -1);
        }
    }
    cv::imshow("color", color_mat);
    cv::waitKey(25);
    return true; // 成功获取 ArUco 中心点
}
void Calibration::PerformCalibration()
{
    std::vector<cv::Point2f> point2f_list{cv::Point2f(350, 0), cv::Point2f(300, 50), cv::Point2f(250, 0), cv::Point2f(300, -50),
                                          cv::Point2f(400, 0), cv::Point2f(300, 100), cv::Point2f(250, 0), cv::Point2f(300, -100),
                                          cv::Point2f(430, 0), cv::Point2f(300, 150), cv::Point2f(250, 0), cv::Point2f(300, -150),
                                          cv::Point2f(430, 0), cv::Point2f(300, 200), cv::Point2f(250, 0), cv::Point2f(299, -200)};
    std::vector<float> z_list{30, 50, 70, 90, 110};
    std::vector<cv::Point3f> point3f_list;
    for (const auto &z : z_list)
    {
        for (const auto &point2f : point2f_list)
        {
            cv::Point3f point3f;
            point3f.x = point2f.x;
            point3f.y = point2f.y;
            point3f.z = z;
            point3f_list.push_back(point3f);
        }
    }

    auto result = episode_.MoveXYZRotation({320, 0, 100}, {180, 0, 90}, "xyz", 1);
    double sleep_time = result.get<double>();
    std::this_thread::sleep_for(std::chrono::milliseconds(static_cast<int>(sleep_time * 1000)));

    std::cout << "即将启动负压吸盘抓取..." << std::endl;
    episode_.GripperOn();
    std::this_thread::sleep_for(std::chrono::seconds(3));

    std::cout << "请固定 ArUco 标记于末端并保持吸附。" << std::endl;
    std::this_thread::sleep_for(std::chrono::seconds(10));
    std::vector<cv::Point3f> robot_point3f_list;
    std::vector<cv::Point3f> camera_point3f_list;
    for (int i = 0; i < point3f_list.size(); i++)
    {
        cv::Point3f robote_point3f = point3f_list[i];
        std::cout << "第 " << i << " 个点: " << robote_point3f << std::endl;
        auto result = episode_.MoveXYZRotation({robote_point3f.x, robote_point3f.y, robote_point3f.z + sucker_length_}, {180, 0, 90}, "xyz", 1);
        double sleep_time = result.get<double>();
        std::this_thread::sleep_for(std::chrono::milliseconds(static_cast<int>(sleep_time * 1000)));
        cv::Point3f camera_point3f;
        if (GetArucoCenter(camera_point3f))
        {
            std::cout << "获取 ArUco 中心点成功: " << camera_point3f << std::endl;
            robote_point3f.x += 50; // 标定板中心偏移
            robot_point3f_list.push_back(robote_point3f);
            camera_point3f_list.push_back(camera_point3f);
        }
        else
        {
            std::cerr << "获取 ArUco 中心点失败" << std::endl;
            continue; // 如果获取失败，跳过当前点
        }
    }
    if (camera_point3f_list.size() < 10)
    {
        std::cerr << "采集点数不足，无法进行标定" << std::endl;
        return;
    }
    std::cout << "采集点数: " << camera_point3f_list.size() << std::endl;
    // 进行标定计算
    cv::Mat affine = cv::estimateAffine3D(camera_point3f_list, robot_point3f_list);
    std::string affine_matrix_file = "calibration/affine_matrix.yml";
    cv::FileStorage fs(affine_matrix_file, cv::FileStorage::WRITE);
    fs << "AffineMatrix" << affine;
    fs.release();
    std::cout << "Affine Matrix:\n"
              << affine << std::endl;
    episode_.GripperOff();
}
// 加载标定结果
void Calibration::LoadCalibrationResult(const std::string &file_path)
{
    // 加载标定结果的具体实现
    // 这里可以添加从文件加载标定结果的代码
    // 例如，读取文件并解析标定参数
}
// 保存标定结果
void Calibration::SaveCalibrationResult(const std::string &file_path)
{
    // 保存标定结果的具体实现
    // 这里可以添加将标定结果保存到文件的代码
    // 例如，将标定参数写入文件
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
    cv::Mat affine_matrix;
    fs["AffineMatrix"] >> affine_matrix;
    fs.release();
    std::cout << "Affine Matrix:\n"
              << affine_matrix << std::endl;
    cv::Mat homo = cv::Mat::eye(4, 4, CV_64F);
    affine_matrix.copyTo(homo.rowRange(0, 3));
    std::cout << "homo:\n"
              << homo << std::endl;

    std::random_device rd;
    unsigned seed = rd() | static_cast<unsigned>(
                               std::chrono::high_resolution_clock::now()
                                   .time_since_epoch()
                                   .count());

    std::mt19937_64 engine(seed);
    std::uniform_int_distribution<int> dist_x(250, 380);
    std::uniform_int_distribution<int> dist_y(-110, 210);
    for (size_t i = 0; i < 10; i++)
    {
        cv::Point3f camera_point3f;
        if (GetArucoCenter(camera_point3f))
        {
            std::cout << "camera point :\n"
                      << camera_point3f << std::endl;
            cv::Mat p_homo = (cv::Mat_<double>(4, 1) << camera_point3f.x, camera_point3f.y, camera_point3f.z, 1.0);
            cv::Mat p_robot_homo = homo * p_homo;
            std::cout << "p_robot_homo:\n"
                      << p_robot_homo << std::endl;
            cv::Point3f robot_point3f;

            robot_point3f.x = p_robot_homo.at<double>(0);
            robot_point3f.y = p_robot_homo.at<double>(1);
            robot_point3f.z = p_robot_homo.at<double>(2);

            auto result = episode_.MoveXYZRotation({robot_point3f.x, robot_point3f.y, robot_point3f.z + sucker_length_ + 100}, {180, 0, 90}, "xyz", 1);
            double sleep_time = result.get<double>();
            std::this_thread::sleep_for(std::chrono::milliseconds(static_cast<int>(sleep_time * 1000)));

            result = episode_.MoveXYZRotation({robot_point3f.x, robot_point3f.y, robot_point3f.z + sucker_length_ - 8}, {180, 0, 90}, "xyz", 1);
            sleep_time = result.get<double>();
            std::this_thread::sleep_for(std::chrono::milliseconds(static_cast<int>(sleep_time * 1000)));
            episode_.GripperOn();

            result = episode_.MoveXYZRotation({robot_point3f.x, robot_point3f.y, robot_point3f.z + sucker_length_ + 100}, {180, 0, 90}, "xyz", 1);
            sleep_time = result.get<double>();
            std::this_thread::sleep_for(std::chrono::milliseconds(static_cast<int>(sleep_time * 1000)));
            int random_num_x = dist_x(engine);
            int random_num_y = dist_y(engine);
            result = episode_.MoveXYZRotation({static_cast<double>(random_num_x), static_cast<double>(random_num_y), sucker_length_ + 100}, {180, 0, 90}, "xyz", 1);
            sleep_time = result.get<double>();
            std::this_thread::sleep_for(std::chrono::milliseconds(static_cast<int>(sleep_time * 1000)));
            episode_.GripperOff();
            std::this_thread::sleep_for(std::chrono::seconds(5));
        }
    }
}

void Calibration::TestCalibration2(const std::string &file_path)
{
    cv::FileStorage fs(file_path, cv::FileStorage::READ);
    if (!fs.isOpened())
    {
        std::cerr << "无法打开标定结果文件: " << file_path << std::endl;
        return;
    }
    cv::Mat affine_matrix;
    fs["AffineMatrix"] >> affine_matrix;
    fs.release();
    std::cout << "Affine Matrix:\n"
              << affine_matrix << std::endl;
    cv::Mat homo = cv::Mat::eye(4, 4, CV_64F);
    affine_matrix.copyTo(homo.rowRange(0, 3));
    std::cout << "homo:\n"
              << homo << std::endl;

    // opencv 查找目标轮廓
    cv::Mat color_mat;
    std::shared_ptr<ob::FrameSet> frameset;
    for (size_t i = 0; i < 50; i++)
    {
        frameset = orbbec_.GetFrameSet();
        std::shared_ptr<ob::VideoStreamProfile> profile;
        profile = orbbec_.GetColorMat(frameset, color_mat);
    }
    cv::imwrite("color_mat.jpg", color_mat);
    cv::Mat gray;
    cv::cvtColor(color_mat, gray, cv::COLOR_BGR2GRAY);
    cv::imwrite("gray.jpg", gray);
    cv::Mat binary;
    cv::threshold(gray, binary, 90, 200, cv::THRESH_BINARY_INV);
    cv::imwrite("binary.jpg", binary);
    std::vector<std::vector<cv::Point>> contours;
    std::vector<cv::Vec4i> hierarchy;
    findContours(binary, contours, hierarchy, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
    // drawContours(color_mat, contours, -1, cv::Scalar(0, 255, 0), 2);
    // cv::imwrite("Contours.jpg", color_mat);
    std::vector<cv::Point> approxs;
    for (size_t i = 0; i < contours.size(); i++)
    {
        // 计算轮廓面积，过滤小轮廓
        double area = cv::contourArea(contours[i]);
        if (area < 15000)
            continue;
        std::cout << "area " << area << std::endl;

        // 轮廓多边形近似
        std::vector<cv::Point> approx;
        // 计算轮廓的周长
        double peri = cv::arcLength(contours[i], true);
        cv::approxPolyDP(contours[i], approx, 0.02 * peri, true);

        // 检查是否为四边形
        if (approx.size() != 4)
            continue;

        // 确保四边形是凸的
        if (!isContourConvex(approx))
            continue;

        // 计算最大内角，过滤非矩形的四边形
        double maxCosine = 0;
        for (int j = 0; j < 4; j++)
        {
            cv::Point pt0 = approx[j];
            cv::Point pt1 = approx[(j + 1) % 4];
            cv::Point pt2 = approx[(j + 2) % 4];

            // 计算边向量
            double dx1 = pt1.x - pt0.x;
            double dy1 = pt1.y - pt0.y;
            double dx2 = pt1.x - pt2.x;
            double dy2 = pt1.y - pt2.y;

            // 计算角度余弦
            double angle = fabs((dx1 * dx2 + dy1 * dy2) /
                                ((sqrt(dx1 * dx1 + dy1 * dy1) * sqrt(dx2 * dx2 + dy2 * dy2)) + 1e-10));

            // 更新最大余弦值
            maxCosine = std::max(maxCosine, angle);
        }

        // 如果所有角度都接近90度，则认为是矩形
        if (maxCosine < 0.3)
        {
            // 在原始图像上绘制四边形
            cv::polylines(color_mat, approx, true, cv::Scalar(0, 255, 0), 3);

            // 绘制顶点
            for (int j = 0; j < 4; j++)
            {
                cv::circle(color_mat, approx[j], 8, cv::Scalar(0, 0, 255), -1);
                cv::putText(color_mat, std::to_string(j + 1), cv::Point(approx[j].x - 10, approx[j].y - 10),
                            cv::FONT_HERSHEY_SIMPLEX, 0.8, cv::Scalar(255, 0, 0), 2);
            }

            // 打印顶点坐标
            std::cout << "Found quadrilateral: " << std::endl;
            for (int j = 0; j < 4; j++)
            {
                std::cout << "Point " << j + 1 << ": (" << approx[j].x << ", " << approx[j].y << ")" << std::endl;
            }
            approxs = approx;
        }
    }

    auto points = GetGridPointsInContour(approxs, 40);
    std::vector<cv::Point> points_reversed(points.rbegin(), points.rend());

    for (int i = 0; i < points_reversed.size(); i++)
    {
        cv::Point &point = points_reversed[i];
        cv::circle(color_mat, point, 1, cv::Scalar(0, 255, 0), -1);
        cv::putText(color_mat, std::to_string(i), point,
                    cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(0, 0, 255), 2);
    }
    cv::imwrite("Contours.jpg", color_mat);
    // cv::imshow("Contours", color_mat);
    // cv::waitKey(0);
    std::vector<cv::Point3f> robot_points;

    for (int i = 0; i < points_reversed.size(); i++)
    {
        cv::Point3f camera_point3f;
        OBPoint2f src;
        cv::Point &point = points_reversed[i];
        src.x = point.x;
        src.y = point.y;
        OBPoint3f target;
        if (orbbec_.Transformation2dto3d(src, target, frameset))
        {
            camera_point3f.x = target.x;
            camera_point3f.y = target.y;
            camera_point3f.z = target.z;
            cv::Mat p_homo = (cv::Mat_<double>(4, 1) << camera_point3f.x, camera_point3f.y, camera_point3f.z, 1.0);
            cv::Mat p_robot_homo = homo * p_homo;
            std::cout << "p_robot_homo:\n"
                      << p_robot_homo << std::endl;
            cv::Point3f robot_point3f;

            robot_point3f.x = p_robot_homo.at<double>(0);
            robot_point3f.y = p_robot_homo.at<double>(1);
            robot_point3f.z = p_robot_homo.at<double>(2);
            robot_points.push_back(robot_point3f);
        }
    }

    for (size_t i = 0; i < robot_points.size(); i++)
    {
        cv::Point3f &robot_point3f = robot_points[i];
        auto result = episode_.MoveXYZRotation({robot_point3f.x, robot_point3f.y, robot_point3f.z + sucker_length_ + 30}, {180, 0, 90}, "xyz", 1);
        double sleep_time = result.get<double>();
        std::this_thread::sleep_for(std::chrono::milliseconds(static_cast<int>(sleep_time * 1000)));
    }
}

// 方法2：网格采样轮廓内部点
std::vector<cv::Point> Calibration::GetGridPointsInContour(std::vector<cv::Point> &contour, int gridStep)
{
    std::vector<cv::Point> points;
    if (contour.empty())
        return points;

    // 1. 获取轮廓的边界矩形
    cv::Rect boundRect = cv::boundingRect(contour);

    // 2. 网格采样
    for (int y = boundRect.y + 10; y < boundRect.y + boundRect.height; y += gridStep)
    {
        for (int x = boundRect.x + 10; x < boundRect.x + boundRect.width; x += gridStep)
        {
            cv::Point pt(x, y);

            // 检查点是否在轮廓内
            if (pointPolygonTest(contour, pt, false) > 0)
            {
                points.push_back(pt);
            }
        }
    }

    return points;
}