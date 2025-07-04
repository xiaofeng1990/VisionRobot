#pragma one
#include <vector>
#include <string>
#include "episode/episode.h"
#include "orbbec/orbbec.h"
#include <thread>
class TeachMode
{
public:
    TeachMode(/* args */);
    ~TeachMode();
    bool InitDevice();

    // 进行教学模式
    void Teaching();
    // 重复
    void Repeat();

private:
    // 获取机械臂关节角度
    void GetJointAngles();
    // 评估图片质量
    bool EvaluateImageQuality(const std::vector<double> &angles, const std::string &image_path);
    // 判断电机状态
    bool IsMotorStable();

private:
    Episode episode_;
    std::unique_ptr<std::thread> thread_;
    std::atomic<bool> is_free_model_{true};
    std::atomic<bool> is_reading_{false};
    // 稳定性判断阈值，值越小要求越稳定
    float stability_threshold_{0.5};
    // 稳定性判断窗口大小
    int stability_window_{50};
    // 位置差异阈值，控制记录点的密度
    int position_diff_threshold_{20};
    std::vector<double> angles_list_;
    std::vector<std::vector<double>> position_list_;
};
