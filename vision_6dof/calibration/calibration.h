#pragma one
#include <vector>
#include <string>
#include "episode/episode.h"
#include "orbbec/orbbec.h"
#include <opencv2/aruco.hpp>

class Calibration
{
    enum RotationOrder
    {
        XYZ,
        XZY,
        YXZ,
        YZX,
        ZXY,
        ZYX
    };

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
    bool GetArucoCenter(cv::Point3f &point3f, cv::Mat &mat);
    bool GetArucoPose(std::vector<cv::Mat> &T_list);
    bool CalibrationCamera();
    void TestCalibrationServoGripper(const std::string &file_path);

private:
    void GetJointAngles();
    double EvaluateImageQuality(cv::Mat &mat);
    void SaveVectorToText(const std::vector<std::vector<double>> &data, const std::string &filename);
    void SaveMatsToYML(const std::string &filename, const std::vector<cv::Mat> &mats);
    std::vector<cv::Mat> LoadMatsFromYML(const std::string &filename);
    void CalcChessboardCorners(cv::Size board_size, float square_size, std::vector<cv::Point3f> &corners);
    void CalculateReprojectionError(std::vector<std::vector<cv::Point3f>> object_points, std::vector<std::vector<cv::Point2f>> image_points, std::vector<cv::Mat> rvecs, std::vector<cv::Mat> tvecs, const cv::Mat &camera_matrix, const cv::Mat &dist_coeffs, std::vector<double> &reprojection_errors);
    // void CalculateReprojectionError(std::vector<std::vector<cv::Point3f>> object_points, std::vector<std::vector<cv::Point2f>> image_points, std::vector<cv::Mat> rvecs, std::vector<cv::Mat> tvecs, const cv::Mat &camera_matrix, const std::vector<double> dist_coeffs, std::vector<double> &reprojection_errors);
    double ComputeReprojectionErrors(
        const std::vector<std::vector<cv::Point3f>> &objectPoints,
        const std::vector<std::vector<cv::Point2f>> &imagePoints,
        const std::vector<cv::Mat> &rvecs, const std::vector<cv::Mat> &tvecs,
        const cv::Mat &cameraMatrix, const cv::Mat &distCoeffs,
        std::vector<float> &perViewErrors);
    cv::Vec3d RotationMatrixToEulerAngles(const cv::Mat &R, RotationOrder order = ZYX);

private:
    // 预采样点
    std::vector<std::vector<double>> pre_sample_points_;
    Episode episode_; // Episode 实例
    Orbbec orbbec_;   // Orbbec 实例
    // std::unique_ptr<std::thread> thread_;
    // std::atomic<bool> is_free_model_{true};
    // std::atomic<bool> is_reading_{false};
    // std::vector<double> angles_list_;
    cv::aruco::ArucoDetector aruco_detector_; // ArUco 检测器
    std::vector<std::vector<double>> position_list_;
    std::vector<cv::Mat> T_mat_list_;
    cv::Point2d last_drop_position_; // 上次放置位置
    float marker_length_{50};        // ArUco 标记的边长
    float sucker_length_{60};        // 吸盘长度
    int board_size_with_{14};        // 棋盘格宽度
    int board_size_height_{10};      // 棋盘格高度
    float square_size_{25};          // 棋盘格方块大小,单位为毫米
    // 标定结果
    // 保存路径
};
