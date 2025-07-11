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
    // 正式采样
    void Sample();
    // 测试标定
    void TestCalibration(const std::string &file_path);

private:
    void GetJointAngles();
    double EvaluateImageQuality(cv::Mat &mat);
    void SaveVectorToText(const std::vector<std::vector<double>> &data, const std::string &filename);
    void SaveMatsToYML(const std::string &filename, const std::vector<cv::Mat> &mats);
    std::vector<cv::Mat> LoadMatsFromYML(const std::string &filename);

private:
    // 预采样点
    std::vector<std::vector<double>> pre_sample_points_;
    Episode episode_; // Episode 实例
    Orbbec orbbec_;   // Orbbec 实例
    std::unique_ptr<std::thread> thread_;
    std::atomic<bool> is_free_model_{true};
    std::atomic<bool> is_reading_{false};
    std::vector<double> angles_list_;

    std::vector<std::vector<double>> position_list_;
    std::vector<cv::Mat> T_mat_list_;
    float marker_length_{0.05}; // ArUco 标记的边长
    float sucker_length_{60};   // 吸盘长度
    int board_size_with_{11};   // 棋盘格宽度
    int board_size_height_{8};  // 棋盘格高度
    // 标定结果
    // 保存路径
};
