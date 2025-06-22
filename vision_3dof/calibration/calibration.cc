#include "calibration.h"
#include <opencv2/aruco.hpp>

Calibration::Calibration(/* args */)
{
}

Calibration::~Calibration()
{
}

bool Calibration::InitDevice()
{
    // 初始化设备
    // 这里可以添加设备初始化的代码

    orbbec_.OpenDevice(); // 替换为实际的序列号

    return true; //
}
// 进行标定 虚函数
void Calibration::PerformCalibration()
{
    // 进行标定的具体实现
    // 这里可以添加标定算法的代码
    // 例如，使用预采样点进行标定计算
}
// 预采样
void Calibration::PreSample()
{
    // 初始化 ArUco 检测器和字典
    // Ptr<aruco::DetectorParameters> parameters = aruco::DetectorParameters::create();
    // Ptr<aruco::Dictionary> dictionary = aruco::getPredefinedDictionary(aruco::DICT_6X6_250);

    cv::aruco::Dictionary dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_5X5_50);
    cv::aruco::DetectorParameters detector_params;
    cv::aruco::ArucoDetector detector(dictionary, detector_params);
    while (1)
    {
        auto frameset = orbbec_.GetFrameSet();
        cv::Mat color_mat;
        std::shared_ptr<ob::VideoStreamProfile> profile;

        profile = orbbec_.GetColorMat(frameset, color_mat);

        std::cout << "Intrinsic: " << profile->getIntrinsic().fx << ", " << profile->getIntrinsic().fy << ", "
                  << profile->getIntrinsic().cx << ", " << profile->getIntrinsic().cy << std::endl;

        std::vector<std::vector<cv::Point2f>> corners, rejected;
        std::vector<int> ids;
        // detect markers and estimate pose
        detector.detectMarkers(color_mat, corners, ids, rejected);
        cv::aruco::drawDetectedMarkers(color_mat, corners, ids);
        // cv::imshow("color", color_mat);
        cv::waitKey(1000);
    }
    cv::destroyAllWindows();
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
void Calibration::TestCalibration()
{
    // 测试标定的具体实现
    // 这里可以添加测试标定结果的代码
    // 例如，使用测试数据验证标定效果
}