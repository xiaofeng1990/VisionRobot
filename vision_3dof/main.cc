#include <string>
#include <iostream>
#include "orbbec/orbbec.h"
int main()
{
    std::cout << "3dof vision" << std::endl;
    Orbbec orbbec;
    if (!orbbec.GetDevice(0))
    {
        std::cerr << "failed to open device" << std::endl;
        return -1;
    }
    std::string serial_number = "CP3294Y0003N";
    if (orbbec.OpenDevice(serial_number))
    {
        std::cout << "open orbbec " << serial_number << " success" << std::endl;
    }
    else
    {
        std::cerr << "open orbbec " << serial_number << " failed" << std::endl;
        return -1;
    }

    std::shared_ptr<ob::FrameSet> frame_set = orbbec.GetFrameSet();

    cv::Mat mat;
    if (orbbec.GetColorMat(frame_set, mat))
    {
        cv::imwrite("./color.png", mat);
    }
    OBPoint3f target;
    OBPoint2f source;
    source.x = 408;
    source.y = 155;
    orbbec.Transformation2dto3d(source, target, frame_set);
    // for (size_t i = 0; i < 100; i++)
    // {
    //     cv::Mat mat;
    //     orbbec.GetDepthFrame(mat);
    //     std::cerr << "get depth" << std::endl;
    // }
    // {
    //     std::cerr << "get depth**********" << std::endl;
    //     cv::Mat mat;
    //     orbbec.GetDepthFrame(mat);
    //     std::vector<int> params;
    //     params.push_back(cv::IMWRITE_PNG_COMPRESSION);
    //     params.push_back(0);
    //     params.push_back(cv::IMWRITE_PNG_STRATEGY);
    //     params.push_back(cv::IMWRITE_PNG_STRATEGY_DEFAULT);
    //     // depth frame pixel value multiply scale to get distance in millimeter
    //     // float scale = videoFrame->as<ob::DepthFrame>()->getValueScale();

    //     cv::Mat cvtMat;
    //     // normalization to 0-255. 0.032f is 256/8000, to limit the range of depth to 8000mm
    //     mat.convertTo(cvtMat, CV_32F, 1 * 0.032f);

    //     // apply gamma correction to enhance the contrast for near objects
    //     cv::pow(cvtMat, 0.6f, cvtMat);

    //     //  convert to 8-bit
    //     cvtMat.convertTo(cvtMat, CV_8UC1, 10); // multiplier 10 is to normalize to 0-255 (nearly) after applying gamma correction

    //     // apply colormap
    //     cv::applyColorMap(cvtMat, mat, cv::COLORMAP_JET);

    //     cv::imwrite("./depth.jpg", mat);
    // }

    // 先硬件D2C

    return 0;
}
