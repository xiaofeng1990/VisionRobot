#include "orbbec.h"
#include <opencv2/opencv.hpp>

Orbbec::Orbbec(/* args */) {}
Orbbec::~Orbbec() { CloseDevice(); }
std::shared_ptr<ob::DeviceList> Orbbec::GetDeviceList()
{
    // Query the list of connected devices
    std::shared_ptr<ob::DeviceList> dev_list = ob_ctx_.queryDeviceList();
    // Get the number of connected devices
    if (dev_list->deviceCount() == 0)
    {
        std::cerr << "Device not found!" << std::endl;
        return nullptr;
    }

    return dev_list;
}

bool Orbbec::GetCameraIntrinsics(XtCameraDepthIntrinsics &depth_intrinsics)
{
    if (!is_open_.load(std::memory_order_acquire))
    {

        std::cerr << "orbbec device not open" << std::endl;
        return false;
    }
    ob_camera_param intrinsics = pipeline_->getCameraParam();
    // XT_LOGT(INFO, TAG, " ********************* camera param! ********************* ");
    // XT_LOGT(INFO, TAG, "depthIntrinsic: fx: %f, fy: %f, cx: %f, cy: %f ,image width :%d image heigh: %d",
    //         intrinsics.depthIntrinsic.fx, intrinsics.depthIntrinsic.fy, intrinsics.depthIntrinsic.cx,
    //         intrinsics.depthIntrinsic.cy, intrinsics.depthIntrinsic.width, intrinsics.depthIntrinsic.height);
    // XT_LOGT(INFO, TAG, "rgbIntrinsic: fx: %f, fy: %f, cx: %f, cy: %f, ,image width :%d image heigh: %d",
    //         intrinsics.rgbIntrinsic.fx, intrinsics.rgbIntrinsic.fy, intrinsics.rgbIntrinsic.cx,
    //         intrinsics.rgbIntrinsic.cy, intrinsics.rgbIntrinsic.width, intrinsics.rgbIntrinsic.height);

    depth_intrinsics.fx = intrinsics.depthIntrinsic.fx;
    depth_intrinsics.fy = intrinsics.depthIntrinsic.fy;
    depth_intrinsics.cx = intrinsics.depthIntrinsic.cx;
    depth_intrinsics.cy = intrinsics.depthIntrinsic.cy;
    depth_intrinsics.width = intrinsics.depthIntrinsic.width;
    depth_intrinsics.height = intrinsics.depthIntrinsic.height;

    // OBCalibrationParam calibration = pipeline_->getCalibrationParam(config_);
    // XT_LOGT(INFO, TAG, " ********************* calibration param! ********************* ");
    // XT_LOGT(INFO, TAG, "depthIntrinsic: fx: %f, fy: %f, cx: %f, cy: %f ,image width :%d image heigh: %d",
    //         calibration.intrinsics[OB_SENSOR_DEPTH].fx, calibration.intrinsics[OB_SENSOR_DEPTH].fy,
    //         calibration.intrinsics[OB_SENSOR_DEPTH].cx, calibration.intrinsics[OB_SENSOR_DEPTH].cy,
    //         calibration.intrinsics[OB_SENSOR_DEPTH].width, calibration.intrinsics[OB_SENSOR_DEPTH].height);
    // XT_LOGT(INFO, TAG, "rgbIntrinsic: fx: %f, fy: %f, cx: %f, cy: %f ,image width :%d image heigh: %d",
    //         calibration.intrinsics[OB_SENSOR_COLOR].fx, calibration.intrinsics[OB_SENSOR_COLOR].fy,
    //         calibration.intrinsics[OB_SENSOR_COLOR].cx, calibration.intrinsics[OB_SENSOR_COLOR].cy,
    //         calibration.intrinsics[OB_SENSOR_COLOR].width, calibration.intrinsics[OB_SENSOR_COLOR].height);

    // XT_LOGT(INFO, TAG, "irIntrinsic: fx: %f, fy: %f, cx: %f, cy: %f ,image width :%d image heigh: %d",
    //         calibration.intrinsics[OB_SENSOR_IR].fx, calibration.intrinsics[OB_SENSOR_IR].fy,
    //         calibration.intrinsics[OB_SENSOR_IR].cx, calibration.intrinsics[OB_SENSOR_IR].cy,
    //         calibration.intrinsics[OB_SENSOR_IR].width, calibration.intrinsics[OB_SENSOR_IR].height);

    // XT_LOGT(INFO, TAG, "irLeftIntrinsic: fx: %f, fy: %f, cx: %f, cy: %f ,image width :%d image heigh: %d",
    //         calibration.intrinsics[OB_SENSOR_IR_LEFT].fx, calibration.intrinsics[OB_SENSOR_IR_LEFT].fy,
    //         calibration.intrinsics[OB_SENSOR_IR_LEFT].cx, calibration.intrinsics[OB_SENSOR_IR_LEFT].cy,
    //         calibration.intrinsics[OB_SENSOR_IR_LEFT].width, calibration.intrinsics[OB_SENSOR_IR_LEFT].height);

    // XT_LOGT(INFO, TAG, "irRightIntrinsic: fx: %f, fy: %f, cx: %f, cy: %f ,image width :%d image heigh: %d",
    //         calibration.intrinsics[OB_SENSOR_IR_RIGHT].fx, calibration.intrinsics[OB_SENSOR_IR_RIGHT].fy,
    //         calibration.intrinsics[OB_SENSOR_IR_RIGHT].cx, calibration.intrinsics[OB_SENSOR_IR_RIGHT].cy,
    //         calibration.intrinsics[OB_SENSOR_IR_RIGHT].width, calibration.intrinsics[OB_SENSOR_IR_RIGHT].height);

    return true;
}

std::shared_ptr<ob::Device> Orbbec::GetDevice(int index)
{
    // Query the list of connected devices
    std::shared_ptr<ob::DeviceList> dev_list = ob_ctx_.queryDeviceList();
    // Get the number of connected devices
    if (dev_list->deviceCount() == 0)
    {

        std::cerr << "Device not found!" << std::endl;
        return nullptr;
    }
    if (index > dev_list->deviceCount())
    {
        std::cerr << "index " << index << " out of device range " << dev_list->deviceCount() << "!" << std::endl;
        return nullptr;
    }

    // Create a device, 0 means the index of the first device
    std::shared_ptr<ob::Device> dev = dev_list->getDevice(index);

    // Get device information
    auto dev_info = dev->getDeviceInfo();

    // Get the name of the device
    std::cout << " **************************************** " << std::endl;
    std::cout << "Device name: " << dev_info->name() << std::endl;

    // Get the pid, vid, uid of the device
    std::cout << "Device pid:: " << dev_info->pid() << " vid: " << dev_info->vid() << ", uid: " << dev_info->uid() << std::endl;

    // By getting the firmware version number of the device
    auto fwVer = dev_info->firmwareVersion();
    std::cout << "Firmware version: " << fwVer << std::endl;

    // By getting the serial number of the device
    auto sn = dev_info->serialNumber();
    std::cout << "Serial number: " << sn << std::endl;

    // By getting the connection type of the device
    auto connectType = dev_info->connectionType();
    std::cout << "ConnectionType: " << connectType << std::endl;

    return dev;
}

std::shared_ptr<ob::Device> Orbbec::GetDevice(std::string serial_number)
{
    // Query the list of connected devices
    std::shared_ptr<ob::DeviceList> dev_list = ob_ctx_.queryDeviceList();
    // Get the number of connected devices
    if (dev_list->deviceCount() == 0)
    {
        // XT_LOGT(ERROR, TAG, "Device not found! ");
        return nullptr;
    }

    std::shared_ptr<ob::Device> dev = dev_list->getDeviceBySN(serial_number.c_str());
    // Create a device, 0 means the index of the first device

    // Get device information
    auto dev_info = dev->getDeviceInfo();

    // Get the name of the device
    // XT_LOGT(INFO, TAG, " **************************************** ");
    // XT_LOGT(INFO, TAG, "Device name: %s", dev_info->name());
    // Get the pid, vid, uid of the device
    // XT_LOGT(INFO, TAG, "Device pid:: %d vid: %d, uid: %s", dev_info->pid(), dev_info->vid(), dev_info->uid());
    // By getting the firmware version number of the device
    auto fwVer = dev_info->firmwareVersion();
    // XT_LOGT(INFO, TAG, "Firmware version: %s", fwVer);
    // By getting the serial number of the device
    auto sn = dev_info->serialNumber();
    // XT_LOGT(INFO, TAG, "Serial number: %s", sn);
    // By getting the connection type of the device
    auto connectType = dev_info->connectionType();
    // XT_LOGT(INFO, TAG, "ConnectionType: %s", connectType);

    return dev;
}

bool Orbbec::CloseDevice()
{
    if (pipeline_ != nullptr)
    {
        pipeline_->stop();
        pipeline_ = nullptr;
    }
    return true;
}

bool Orbbec::OpenDevice(std::string serial_number)
{
    std::shared_ptr<ob::Device> device = Orbbec::GetDevice(serial_number);
    if (device == nullptr)
    {
        // XT_LOGT(INFO, TAG, "open orbbec %s failed", serial_number.c_str());
        is_open_.store(false, std::memory_order_release);
        return false;
    }

    pipeline_ = std::make_shared<ob::Pipeline>(device);
    // Create a context, for getting devices and sensors
    context_ = std::make_shared<ob::Context>();
    // Activate device clock synchronization
    context_->enableDeviceClockSync(0);

    // Create a config and enable the depth and color streams.
    config_ = std::make_shared<ob::Config>();
    // Enable the color and depth streams.
    config_->enableStream(OB_STREAM_COLOR);
    config_->enableStream(OB_STREAM_DEPTH);

    config_->enableVideoStream(OB_STREAM_DEPTH, 1280, 800, OB_FPS_ANY, OB_FORMAT_ANY);
    //                   uint32_t fps = OB_FPS_ANY, OBFormat format = OB_FORMAT_ANY);
// Enable the IR stream.
#if 1
    // config_->enableStream(OB_STREAM_IR_LEFT);
    // config_->enableStream(OB_STREAM_IR_RIGHT);
    config_->enableVideoStream(OB_STREAM_IR_LEFT, 1280, 800, OB_FPS_ANY, OB_FORMAT_ANY);
    config_->enableVideoStream(OB_STREAM_IR_RIGHT, 1280, 800, OB_FPS_ANY, OB_FORMAT_ANY);
#else
    auto irLeftProfiles = pipeline_->getStreamProfileList(OB_SENSOR_IR_LEFT);
    if (irLeftProfiles == nullptr)
    {
        std::cerr << "The obtained IR(Left) resolution list is NULL. For monocular structured light devices, try "
                     "opening the IR data stream using the IR example. "
                  << std::endl;
    }
    else
    {
        // Open the default profile of IR_LEFT Sensor, which can be configured through the configuration file
        try
        {
            irLeftProfiles->getCount();
            for (uint32_t i = 0; i < irLeftProfiles->getCount(); ++i)
            {
                auto profile = irLeftProfiles->getProfile(i);
                XT_LOGT(INFO, TAG, "IR(Left) profile %d: width: %d, height: %d, fps: %d, format: %d", i,
                        profile->as<ob::VideoStreamProfile>()->width(), profile->as<ob::VideoStreamProfile>()->height(),
                        profile->as<ob::VideoStreamProfile>()->fps(), profile->as<ob::VideoStreamProfile>()->format());
            }
            // auto irLeftProfile =
            //     irLeftProfiles->getVideoStreamProfile(OB_WIDTH_ANY, OB_HEIGHT_ANY, OB_FORMAT_ANY, OB_FPS_ANY);
            auto irLeftProfile = irLeftProfiles->getProfile(OB_PROFILE_DEFAULT);
            config_->enableStream(irLeftProfile->as<ob::VideoStreamProfile>());
        }
        catch (...)
        {
            std::cout << "IR(Left) stream not found!" << std::endl;
        }
    }

    auto irRightProfiles = pipeline_->getStreamProfileList(OB_SENSOR_IR_RIGHT);

    // Open the default profile of IR_RIGHT Sensor, which can be configured through the configuration file
    try
    {
        auto irRightProfile = irRightProfiles->getProfile(OB_PROFILE_DEFAULT);
        config_->enableStream(irRightProfile->as<ob::VideoStreamProfile>());
    }
    catch (...)
    {
        std::cout << "IR(Right) stream not found!" << std::endl;
    }
#endif
    // Set the frame aggregate output mode to all type frame require.
    config_->setFrameAggregateOutputMode(OB_FRAME_AGGREGATE_OUTPUT_ALL_TYPE_FRAME_REQUIRE);
    format_onverter_ = std::make_shared<ob::FormatConvertFilter>();
    // Start the pipeline.
    pipeline_->enableFrameSync();
    pipeline_->start(config_);
    auto cameraParam = pipeline_->getCameraParam();
    is_open_.store(true, std::memory_order_release);
    return true;
}
bool Orbbec::GetColorFrame(cv::Mat &mat)
{
    if (!is_open_.load(std::memory_order_acquire))
    {
        // XT_LOGT(ERROR, TAG, "orbbec device not open");
        return false;
    }
    int read_count = 0;
    while (read_count < max_read_count_)
    {
        // Wait for frameSet from the pipeline.
        std::shared_ptr<ob::FrameSet> frameSet = pipeline_->waitForFrameset(100);
        if (!frameSet)
        {
            // XT_LOGT(WARN, TAG, "No color frames received in 100ms...");
            read_count++;
            continue;
        }
        // get color frame from frameset.
        // auto colorFrame = frameSet->colorFrame();
        auto color_frame = frameSet->getFrame(OB_FRAME_COLOR)->as<ob::ColorFrame>();

        // Convert the color frame to RGB format.
        if (color_frame->format() != OB_FORMAT_RGB)
        {
            if (color_frame->format() == OB_FORMAT_MJPG)
            {
                format_onverter_->setFormatConvertType(FORMAT_MJPG_TO_RGB);
            }
            else if (color_frame->format() == OB_FORMAT_UYVY)
            {
                format_onverter_->setFormatConvertType(FORMAT_UYVY_TO_RGB);
            }
            else if (color_frame->format() == OB_FORMAT_YUYV)
            {
                format_onverter_->setFormatConvertType(FORMAT_YUYV_TO_RGB);
            }
            else
            {
                // XT_LOGT(WARN, TAG, "Color format is not support!");
                continue;
                read_count++;
            }
            color_frame = format_onverter_->process(color_frame)->as<ob::ColorFrame>();
        }

        // Processed the color frames to BGR format, use OpenCV to save to disk.
        format_onverter_->setFormatConvertType(FORMAT_RGB_TO_BGR);
        color_frame = format_onverter_->process(color_frame)->as<ob::ColorFrame>();

        std::vector<int> params;
        params.push_back(cv::IMWRITE_PNG_COMPRESSION);
        params.push_back(0);
        params.push_back(cv::IMWRITE_PNG_STRATEGY);
        params.push_back(cv::IMWRITE_PNG_STRATEGY_DEFAULT);
        cv::Mat color_raw_mat(color_frame->height(), color_frame->width(), CV_8UC3, color_frame->data());
        color_raw_mat.copyTo(mat);
        break;
    }
    bool ret = false;
    if (read_count >= max_read_count_)
    {
        // XT_LOGT(ERROR, TAG, "read color frame failed count %d out of range %d", read_count, max_read_count_);
    }
    else
    {
        ret = true;
        // XT_LOGT(INFO, TAG, "read color frame success, count %d", read_count);
    }
    return ret;
}

bool Orbbec::GetDepthFrame(cv::Mat &mat)
{
    // // Start the pipeline with config.
    // for (int i = 0; i < 15; ++i)
    // {
    //     auto lost = pipeline->waitForFrameset(100);
    // }

    if (!is_open_.load(std::memory_order_acquire))
    {
        // XT_LOGT(ERROR, TAG, "orbbec device not open");
        return false;
    }
    int read_count = 0;
    while (read_count < max_read_count_)
    {
        // Wait for frameSet from the pipeline.
        std::shared_ptr<ob::FrameSet> frameSet = pipeline_->waitForFrameset(100);
        if (!frameSet)
        {
            // XT_LOGT(WARN, TAG, "No depth frames received in 100ms...");
            read_count++;
            continue;
        }
        auto depth_frame = frameSet->getFrame(OB_FRAME_DEPTH)->as<ob::DepthFrame>();

        std::vector<int> params;
        params.push_back(cv::IMWRITE_PNG_COMPRESSION);
        params.push_back(0);
        params.push_back(cv::IMWRITE_PNG_STRATEGY);
        params.push_back(cv::IMWRITE_PNG_STRATEGY_DEFAULT);

        cv::Mat depth_mat(depth_frame->height(), depth_frame->width(), CV_16UC1, depth_frame->data());
        depth_mat.copyTo(mat);
        break;
    }
    bool ret = false;
    if (read_count >= max_read_count_)
    {
        // XT_LOGT(ERROR, TAG, "read depth frame failed count %d out of range %d", read_count, max_read_count_);
    }
    else
    {
        ret = true;
        // XT_LOGT(INFO, TAG, "read depth frame success, count %d", read_count);
    }
    return ret;
}

bool Orbbec::GetIrFrame(std::vector<cv::Mat> &ir_mat_list)
{
    if (!is_open_.load(std::memory_order_acquire))
    {
        // XT_LOGT(ERROR, TAG, "orbbec device not open");
        return false;
    }

    int read_count = 0;
    while (read_count < max_read_count_)
    {
        // Wait for frameSet from the pipeline.
        std::shared_ptr<ob::FrameSet> frameSet = pipeline_->waitForFrameset(100);
        if (!frameSet)
        {
            // XT_LOGT(WARN, TAG, "No depth frames received in 100ms...");
            read_count++;
            continue;
        }
        auto left_frame = frameSet->getFrame(OB_FRAME_IR_LEFT)->as<ob::IRFrame>();
        auto right_frame = frameSet->getFrame(OB_FRAME_IR_RIGHT)->as<ob::IRFrame>();
        std::vector<int> params;
        params.push_back(cv::IMWRITE_PNG_COMPRESSION);
        params.push_back(0);
        params.push_back(cv::IMWRITE_PNG_STRATEGY);
        params.push_back(cv::IMWRITE_PNG_STRATEGY_DEFAULT);

        cv::Mat left_mat(left_frame->height(), left_frame->width(), CV_8UC1, left_frame->data());
        cv::Mat right_mat(right_frame->height(), right_frame->width(), CV_8UC1, right_frame->data());
        ir_mat_list.push_back(left_mat);
        ir_mat_list.push_back(right_mat);
        break;
    }
    bool ret = false;
    if (read_count >= max_read_count_)
    {
        // XT_LOGT(ERROR, TAG, "read ir frame failed count %d out of range %d", read_count, max_read_count_);
    }
    else
    {
        ret = true;
        // XT_LOGT(INFO, TAG, "read ir frame success, count %d", read_count);
    }
    return ret;
}
