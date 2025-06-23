#pragma one
#include <vector>
#include <string>
#include "episode/episode.h"
#include "orbbec/orbbec.h"
#include <opencv2/aruco.hpp>

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
    void TestCalibration(const std::string &file_path);

private:
    bool GetArucoCenter(cv::Point3f &point3f);

private:
    // 预采样点
    std::vector<std::vector<double>> pre_sample_points_;
    Episode episode_;                         // Episode 实例
    Orbbec orbbec_;                           // Orbbec 实例
    cv::aruco::ArucoDetector aruco_detector_; // ArUco 检测器
    float marker_length_{0.05};               // ArUco 标记的边长
    float sucker_length_{60};                 // 吸盘长度
    // 标定结果
    // 保存路径
};
