#pragma one
#include <vector>
#include <string>
#include "episode/episode.h"
#include "orbbec/orbbec.h"

class Calibration
{

public:
    Calibration(/* args */);
    ~Calibration();
    bool InitDevice();

    // 进行标定 虚函数
    void PerformCalibration();
    // 预采样
    void PreSample();
    // 加载标定结果
    void LoadCalibrationResult(const std::string &file_path);
    // 保存标定结果
    void SaveCalibrationResult(const std::string &file_path);
    // 测试标定
    void TestCalibration();

private:
    // 预采样点
    std::vector<std::vector<double>> pre_sample_points_;
    Episode episode_; // Episode 实例
    Orbbec orbbec_;   // Orbbec 实例
    // 标定结果
    // 保存路径
};
