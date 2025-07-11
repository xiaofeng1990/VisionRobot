#include "calibration.h"

#include <iostream>
#include <random> // C++11 随机数库
#include <chrono> // 时间相关功能
#include <iostream>
#include <fstream>
#include <vector>

Calibration::Calibration(/* args */)
{
}

Calibration::~Calibration()
{
    episode_.SetFreeMode(0);
    if (thread_ != nullptr && thread_->joinable())
    {
        thread_->join();
    }
}

bool Calibration::InitDevice()
{
    // 初始化设备
    // 这里可以添加设备初始化的代码
    orbbec_.OpenDevice(); // 替换为实际的序列号
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
    thread_.reset(new std::thread(std::bind(&Calibration::GetJointAngles, this)));
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
    while (true)
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
                continue;
            }
            std::cout << "检测到棋盘格，保存图像" << std::endl;
            // 判断图片质量
            double variance = EvaluateImageQuality(color_mat);
            std::cout << "图像清晰度方差: " << variance << std::endl;
            is_reading_.store(true, std::memory_order_release);
            std::vector<double> angles_list = angles_list_;
            is_reading_.store(false, std::memory_order_release);
            if (angles_list.size() != 6)
            {
                std::cout << "角度列表长度不正确，无法保存" << std::endl;
                continue;
            }
            position_list_.push_back(angles_list);
            std::cout << "保存点:  ";
            for (auto &angle : angles_list)
            {
                std::cout << angle << " ";
            }
            std::cout << std::endl;

            break;
        }

        case 'q':
        {
            // 保存点
            cv::destroyAllWindows();
            std::cout << "q pressed. Exiting..." << std::endl;
            episode_.SetFreeMode(0);
            is_free_model_.store(false, std::memory_order_release);
            // 输出点到本地文件
            SaveVectorToText(position_list_, "calibration/calibration_points.txt");
            return;
        }
        default:
            break;
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
            std::string filename = "calibration/images/" + std::to_string(i) + ".jpg";
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
    std::string affine_matrix_file = "calibration/T_end2base.yml";
    SaveMatsToYML(affine_matrix_file, T_mat_list_);
    // 输出采样点到文件
}
void Calibration::PerformCalibration()
{
}

// 测试标定
void Calibration::TestCalibration(const std::string &file_path)
{
    // 退出自由模式
    // is_free_model_.store(false, std::memory_order_release);
}

void Calibration::GetJointAngles()
{
    while (true)
    {
        if (!is_free_model_.load(std::memory_order_acquire))
        {
            break;
        }

        if (!is_reading_.load(std::memory_order_acquire))
        {
            auto angles = episode_.GetMotorAngles();
            // std::cout << "获取角度 " << angles.dump() << std::endl;
            if (angles.is_array())
                angles_list_ = angles.get<std::vector<double>>();
            std::this_thread::sleep_for(std::chrono::milliseconds(500));
        }
    }
    std::cout << "exit()" << std::endl;
}
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
    fs << "mat_sequence" << "["; // 开始序列

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