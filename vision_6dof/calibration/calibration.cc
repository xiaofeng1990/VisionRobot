#include "calibration.h"

#include <iostream>
#include <random> // C++11 随机数库
#include <chrono> // 时间相关功能

Calibration::Calibration(/* args */)
{
}

Calibration::~Calibration()
{
    episode_.SetFreeMode(0);
    if (thread_->joinable())
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
        thread_.reset(new std::thread(std::bind(&Calibration::GetJointAngles, this)));
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
    // is_free_model_.store(false, std::memory_order_release);

    std::cout << "15s后进入自由模式, 请手动移动机械臂..." << std::endl;
    std::cout << "=========================================" << std::endl;
    std::this_thread::sleep_for(std::chrono::seconds(15));
    //  测试采图,暂时不进入自由模式
    // episode_.SetFreeMode(1);
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
        board_size.width = 11;
        board_size.height = 8;
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

            is_reading_.store(true, std::memory_order_release);
            std::vector<double> angles_list = angles_list_;
            is_reading_.store(false, std::memory_order_release);
            position_list_.push_back(angles_list);
            // std::cout << "保存点 " << motors_degrees << std::endl;
            break;
        }

        case 'q':
        {
            // 保存点
            cv::destroyAllWindows();
            std::cout << "q pressed. Exiting..." << std::endl;
            episode_.SetFreeMode(0);
            is_free_model_.store(false, std::memory_order_release);
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
}
void Calibration::PerformCalibration()
{
    // 退出自由模式
}

// 测试标定
void Calibration::TestCalibration(const std::string &file_path)
{
    // 退出自由模式
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
            angles_list_ = angles.get<std::vector<double>>();
            std::this_thread::sleep_for(std::chrono::milliseconds(500));
        }
    }
    std::cout << "exit()" << std::endl;
}
