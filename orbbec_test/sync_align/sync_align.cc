// Copyright (c) Orbbec Inc. All Rights Reserved.
// Licensed under the MIT License.

#include <libobsensor/ObSensor.hpp>

#include <opencv2/opencv.hpp>

#include <mutex>
#include <thread>

void saveDepthFrame(const std::shared_ptr<ob::DepthFrame> depthFrame, const uint32_t frameIndex)
{
    std::vector<int> params;
    params.push_back(cv::IMWRITE_PNG_COMPRESSION);
    params.push_back(0);
    params.push_back(cv::IMWRITE_PNG_STRATEGY);
    params.push_back(cv::IMWRITE_PNG_STRATEGY_DEFAULT);
    std::string depthName = "Depth_" + std::to_string(depthFrame->width()) + "x" + std::to_string(depthFrame->height()) + "_" + std::to_string(frameIndex) + "_" + std::to_string(depthFrame->timeStamp()) + "ms.png";
    cv::Mat depthMat(depthFrame->height(), depthFrame->width(), CV_16UC1, depthFrame->data());
    cv::imwrite(depthName, depthMat, params);
    std::cout << "Depth saved:" << depthName << std::endl;
}

void saveColorFrame(std::shared_ptr<ob::Frame> frame, const uint32_t frameIndex)
{
    // Create a format converter filter.
    auto formatConverter = std::make_shared<ob::FormatConvertFilter>();
    std::cout << "2222222222222" << std::endl;
    auto colorFrame = frame;
    std::cout << "saveColorFrame" << std::endl;
    // Convert the color frame to RGB format.
    // if (colorFrame->format() != OB_FORMAT_RGB)
    // {
    //     if (colorFrame->format() == OB_FORMAT_MJPG)
    //     {
    //         formatConverter->setFormatConvertType(FORMAT_MJPG_TO_RGB);
    //     }
    //     else if (colorFrame->format() == OB_FORMAT_UYVY)
    //     {
    //         formatConverter->setFormatConvertType(FORMAT_UYVY_TO_RGB);
    //     }
    //     else if (colorFrame->format() == OB_FORMAT_YUYV)
    //     {
    //         formatConverter->setFormatConvertType(FORMAT_YUYV_TO_RGB);
    //     }
    //     else
    //     {
    //         std::cout << "Color format is not support!" << std::endl;
    //     }
    //     colorFrame = formatConverter->process(colorFrame)->as<ob::ColorFrame>();
    // }
    // std::cout << "saveColorFrame" << std::endl;
    // // Processed the color frames to BGR format, use OpenCV to save to disk.
    // formatConverter->setFormatConvertType(FORMAT_RGB_TO_BGR);
    // colorFrame = formatConverter->process(colorFrame)->as<ob::ColorFrame>();

    std::vector<int> params;
    params.push_back(cv::IMWRITE_PNG_COMPRESSION);
    params.push_back(0);
    params.push_back(cv::IMWRITE_PNG_STRATEGY);
    params.push_back(cv::IMWRITE_PNG_STRATEGY_DEFAULT);
    std::string colorName = "Color_" + std::to_string(frameIndex) + "_" + std::to_string(colorFrame->timeStamp()) + "ms.png";
    cv::Mat depthMat(800, 1280, CV_8UC3, colorFrame->data());
    cv::imwrite(colorName, depthMat, params);
    std::cout << "Color saved:" << colorName << std::endl;
}

int main()
{

    ob::Context ob_ctx;
    std::shared_ptr<ob::DeviceList> dev_list = ob_ctx.queryDeviceList();
    // Get the number of connected devices
    if (dev_list->deviceCount() == 0)
    {

        std::cerr << "Device not found!" << std::endl;
        return 0;
    }

    // Create a device, 0 means the index of the first device
    std::shared_ptr<ob::Device> dev = dev_list->getDevice(0);

    // Create a pipeline with default device to manage stream
    auto pipe = std::make_shared<ob::Pipeline>(dev);

    // Configure which streams to enable or disable for the Pipeline by creating a Config
    auto config = std::make_shared<ob::Config>();

    // enable depth and color streams with specified format
    config->enableVideoStream(OB_STREAM_DEPTH, 1280, 800, OB_FPS_ANY, OB_FORMAT_Y16);
    config->enableVideoStream(OB_STREAM_COLOR, 1280, 800, OB_FPS_ANY, OB_FORMAT_RGB);

    // set the frame aggregate output mode to ensure all types of frames are included in the output frameset
    config->setFrameAggregateOutputMode(OB_FRAME_AGGREGATE_OUTPUT_ALL_TYPE_FRAME_REQUIRE);

    // Start the pipeline with config
    pipe->start(config);
    pipe->enableFrameSync();
    // Create a filter to align depth frame to color frame
    auto depth2colorAlign = std::make_shared<ob::Align>(OB_STREAM_COLOR);

    // create a filter to align color frame to depth frame
    auto color2depthAlign = std::make_shared<ob::Align>(OB_STREAM_DEPTH);

    for (int i = 0; i < 100; i++)
    {
        // Wait for a frameset from the pipeline
        auto frameSet = pipe->waitForFrameset(100);
        if (frameSet == nullptr)
        {
            std::cout << "******* continue *******" << std::endl;
            continue;
        }
        std::cout << "******* ok *******" << std::endl;
        // Get filter according to the align mode
        std::shared_ptr<ob::Filter> alignFilter = depth2colorAlign;

        std::shared_ptr<ob::Frame> frame_align = alignFilter->process(frameSet);
        std::cout << "******* frame_align *******" << std::endl;
        saveColorFrame(frame_align, i);
    }

    // Stop the Pipeline, no frame data will be generated
    pipe->stop();

    return 0;
}
