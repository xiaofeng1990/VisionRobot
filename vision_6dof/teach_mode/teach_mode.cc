#include "teach_mode.h"

TeachMode::TeachMode(/* args */)
{
}

TeachMode::~TeachMode()
{
    episode_.SetFreeMode(0);
    if (thread_->joinable())
    {
        thread_->join();
    }
}
bool TeachMode::InitDevice()
{
    if (episode_.Connect("127.0.0.1", 12345))
    {
        std::cout << "Episode 连接成功 移动到初始位置" << std::endl;
        thread_.reset(new std::thread(std::bind(&TeachMode::GetJointAngles, this)));
        return true;
    }
    else
    {
        std::cerr << "Episode 连接失败" << std::endl;
        return false;
    }
}

// 进行教学模式
void TeachMode::Teaching()
{
    std::cout << "================ 教学模式 ================" << std::endl;
    std::cout << "15s后进入自由模式, 请手动移动机械臂..." << std::endl;
    std::cout << "机械臂稳定时会自动记录位置" << std::endl;
    std::cout << "按 'q' 键结束并保存轨迹数据" << std::endl;
    std::cout << "=========================================" << std::endl;
    std::this_thread::sleep_for(std::chrono::seconds(15));
    // 进入自由模式
    episode_.SetFreeMode(1);
    while (true)
    {
        cv::Mat img(480, 800, CV_8UC3, cv::Scalar(0, 0, 0));
        cv::putText(img, "Teach Mode: Points recorded when stable", cv::Point(10, 30), cv::FONT_HERSHEY_SIMPLEX, 0.8, cv::Scalar(0, 0, 255), 2);
        std::string txt = "Recorded points: " + std::to_string(position_list_.size());
        cv::putText(img, txt, cv::Point(10, 110), cv::FONT_HERSHEY_SIMPLEX, 0.8, cv::Scalar(0, 0, 255), 2);
        is_reading_.store(true, std::memory_order_release);
        std::vector<double> angles_list = angles_list_;
        is_reading_.store(false, std::memory_order_release);
        std::string motors_degrees;
        for (auto &angle : angles_list)
        {
            motors_degrees.append(std::to_string(angle));
            motors_degrees.append(";");
        }
        txt = "Current angles: " + motors_degrees;
        cv::putText(img, txt, cv::Point(10, 150), cv::FONT_HERSHEY_SIMPLEX, 0.8, cv::Scalar(0, 0, 255), 2);

        // 必须显示图像（无图像窗口无法接收键盘事件）
        cv::imshow("Teach Mode", img);

        int key = cv::waitKey(10);
        switch (key)
        {
        case 's':
            position_list_.push_back(angles_list);
            std::cout << "保存点 " << motors_degrees << std::endl;
            break;
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
// 重复
void TeachMode::Repeat()
{
    std::cout << "================ 轨迹复现 ================" << std::endl;
    std::cout << "加载 " << position_list_.size() << " 点";
    std::cout << "================ 开始轨迹复现... ================" << std::endl;
    for (size_t i = 0; i < position_list_.size(); i++)
    {
        std::cout << "移动到位置点 " << i << "/" << position_list_.size() << std::endl;
        auto result = episode_.AngleMode(position_list_[i]);
        double sleep_time = result.get<double>();
        std::this_thread::sleep_for(std::chrono::milliseconds(static_cast<int>(sleep_time * 1000)));
    }
}

// 获取机械臂关节角度
void TeachMode::GetJointAngles()
{
    while (true)
    {
        if (!is_free_model_.load(std::memory_order_acquire))
        {
            break;
        }
        auto angles = episode_.GetMotorAngles();

        std::cout << "获取到的机械臂关节角度: " << angles.dump() << std::endl;
        if (!is_reading_.load(std::memory_order_acquire))
        {
            angles_list_ = angles.get<std::vector<double>>();
            std::this_thread::sleep_for(std::chrono::milliseconds(3000));
        }
    }
    std::cout << "exit()" << std::endl;
}

// 评估图片质量
bool TeachMode::EvaluateImageQuality(const std::vector<double> &angles, const std::string &image_path) {}
// 判断电机状态
bool TeachMode::IsMotorStable()
{
    return true;
}