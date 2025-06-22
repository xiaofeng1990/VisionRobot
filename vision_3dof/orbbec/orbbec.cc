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

bool Orbbec::OpenDevice()
{
    device_ = Orbbec::GetDevice(0);
    if (device_ == nullptr)
    {
        // XT_LOGT(INFO, TAG, "open orbbec %s failed", serial_number.c_str());
        is_open_.store(false, std::memory_order_release);
        return false;
    }

    pipeline_ = std::make_shared<ob::Pipeline>(device_);
    // Create a context, for getting devices and sensors
    context_ = std::make_shared<ob::Context>();
    // Activate device clock synchronization
    context_->enableDeviceClockSync(0);

    // Create a config and enable the depth and color streams.
    config_ = Orbbec::CreateHwD2CAlignConfig();
    // config_ = std::make_shared<ob::Config>();
    // Enable the color and depth streams.
    // config_->enableStream(OB_STREAM_COLOR);
    // config_->enableVideoStream(OB_STREAM_COLOR, OB_WIDTH_ANY, OB_HEIGHT_ANY, OB_FPS_ANY, OB_FORMAT_ANY);

#if 1
    // config_->enableStream(OB_STREAM_IR_LEFT);
    // config_->enableStream(OB_STREAM_IR_RIGHT);
    // config_->enableVideoStream(OB_STREAM_IR_LEFT, OB_WIDTH_ANY, OB_HEIGHT_ANY, OB_FPS_ANY, OB_FORMAT_ANY);
    // config_->enableVideoStream(OB_STREAM_IR_RIGHT, OB_WIDTH_ANY, OB_HEIGHT_ANY, OB_FPS_ANY, OB_FORMAT_ANY);
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

                printf("IR(Left) profile %d: width: %d, height: %d, fps: %d, format: %d \n", i,
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
    format_onverter_ = std::make_shared<ob::FormatConvertFilter>();
    // config_->setFrameAggregateOutputMode(OB_FRAME_AGGREGATE_OUTPUT_ALL_TYPE_FRAME_REQUIRE);
    // Start the pipeline.
    pipeline_->enableFrameSync();
    pipeline_->start(config_);
    for (int i = 0; i < 15; ++i)
    {
        auto lost = pipeline_->waitForFrameset(100);
    }
    is_open_.store(true, std::memory_order_release);
    return true;
}
std::shared_ptr<ob::VideoStreamProfile> Orbbec::GetColorMat(std::shared_ptr<ob::FrameSet> frame_set, cv::Mat &mat)
{
    if (!is_open_.load(std::memory_order_acquire))
    {
        // XT_LOGT(ERROR, TAG, "orbbec device not open");
        return nullptr;
    }
    int read_count = 0;
    if (frame_set == nullptr)
    {
        std::cout << "frame_set is null" << std::endl;
        return nullptr;
    }

    // get color frame from frameset.
    // auto colorFrame = frameSet->colorFrame();
    auto color_frame = frame_set->getFrame(OB_FRAME_COLOR)->as<ob::ColorFrame>();

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
            std::cout << "Color format is not support!" << std::endl;
            return nullptr;
        }
        color_frame = format_onverter_->process(color_frame)->as<ob::ColorFrame>();
    }

    // Processed the color frames to BGR format, use OpenCV to save to disk.
    format_onverter_->setFormatConvertType(FORMAT_RGB_TO_BGR);
    color_frame = format_onverter_->process(color_frame)->as<ob::ColorFrame>();
    cv::Mat color_raw_mat(color_frame->height(), color_frame->width(), CV_8UC3, color_frame->data());
    color_raw_mat.copyTo(mat);

    return color_frame->getStreamProfile()->as<ob::VideoStreamProfile>();
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

std::shared_ptr<ob::FrameSet> Orbbec::GetFrameSet()
{
    if (!is_open_.load(std::memory_order_acquire))
    {
        // XT_LOGT(ERROR, TAG, "orbbec device not open");
        return nullptr;
    }
    int read_count = 0;
    while (read_count < max_read_count_)
    {
        // Wait for frameSet from the pipeline.
        std::shared_ptr<ob::FrameSet> frame_set = pipeline_->waitForFrameset(100);
        if (!frame_set)
        {
            // XT_LOGT(WARN, TAG, "No depth frames received in 100ms...");
            read_count++;
            continue;
        }
        return frame_set;
    }

    return nullptr;
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

bool Orbbec::Transformation2dto3d(OBPoint2f source, OBPoint3f &target, std::shared_ptr<ob::FrameSet> frame_set)
{
    // Get the color frame and check its validity
    auto color_frame = frame_set->getFrame(OB_FRAME_COLOR);

    // Get the depth frame and check its validity
    auto depth_frame = frame_set->getFrame(OB_FRAME_DEPTH);

    // Get the width and height of the color and depth frames
    auto depth_frame_width = depth_frame->as<ob::VideoFrame>()->getWidth();
    auto depth_frame_height = depth_frame->as<ob::VideoFrame>()->getHeight();

    // Get the stream profiles for the color and depth frames
    auto color_profile = color_frame->getStreamProfile();
    auto depth_profile = depth_frame->getStreamProfile();
    auto extrinsicD2C = depth_profile->getExtrinsicTo(color_profile);

    // Get the intrinsic and distortion parameters for the color and depth streams
    auto depth_intrinsic = depth_profile->as<ob::VideoStreamProfile>()->getIntrinsic();
    // Access the depth data from the frame
    uint16_t *pDepthData = (uint16_t *)depth_frame->getData();
    uint16_t convertAreaWidth = 3;
    uint16_t convertAreaHeight = 3;

    // Get the depth value of the current pixel
    float depth_value = (float)pDepthData[static_cast<uint16_t>(source.y) * depth_frame_width + static_cast<uint16_t>(source.x)];
    if (depth_value == 0)
    {
        std::cout << "The depth value is 0, so it's recommended to point the camera at a flat surface" << std::endl;
        return false;
    }
    bool result = ob::CoordinateTransformHelper::transformation2dto3d(source, depth_value, depth_intrinsic, extrinsicD2C, &target);
    if (!result)
    {
        return false;
    }
    PrintRuslt("2d to 3D: pixel coordinates and depth transform to point in 3D space", source, target, depth_value);
    return true;
}
void Orbbec::PrintRuslt(std::string msg, OBPoint2f source, OBPoint3f target, float depth_value)
{
    std::cout << msg << ":" << "depth " << depth_value << " (" << source.x << ", " << source.y << ") -> (" << target.x << ", " << target.y << ", " << target.z << ")" << std::endl;
}
std::vector<OBPropertyItem> Orbbec::GetPropertyList()
{
    std::vector<OBPropertyItem> property_vec;
    property_vec.clear();
    if (!device_)
        return property_vec;
    uint32_t size = device_->getSupportedPropertyCount();
    for (uint32_t i = 0; i < size; i++)
    {
        OBPropertyItem property_item = device_->getSupportedProperty(i);
        if (IsPrimaryTypeProperty(property_item) && property_item.permission != OB_PERMISSION_DENY)
        {
            property_vec.push_back(property_item);
        }
    }
    return property_vec;
}

// Print a list of supported properties
void Orbbec::PrintfPropertyList()
{
    std::vector<OBPropertyItem> property_list = GetPropertyList();
    std::cout << "size: " << property_list.size() << std::endl;
    if (property_list.empty())
    {
        std::cout << "No supported property!" << std::endl;
    }
    std::cout << "\n------------------------------------------------------------------------\n";
    for (size_t i = 0; i < property_list.size(); i++)
    {
        auto property_item = property_list[i];
        std::string strRange = "";

        OBIntPropertyRange int_range;
        OBFloatPropertyRange float_range;
        switch (property_item.type)
        {
        case OB_BOOL_PROPERTY:
            strRange = "Bool value(min:0, max:1, step:1)";
            break;
        case OB_INT_PROPERTY:
        {
            try
            {
                int_range = device_->getIntPropertyRange(property_item.id);
                strRange = "Int value(min:" + std::to_string(int_range.min) + ", max:" + std::to_string(int_range.max) + ", step:" + std::to_string(int_range.step) + ")";
            }
            catch (...)
            {
                std::cout << "get int property range failed." << std::endl;
            }
        }
        break;
        case OB_FLOAT_PROPERTY:
            try
            {
                float_range = device_->getFloatPropertyRange(property_item.id);
                strRange = "Float value(min:" + std::to_string(float_range.min) + ", max:" + std::to_string(float_range.max) + ", step:" + std::to_string(float_range.step) + ")";
            }
            catch (...)
            {
                std::cout << "get float property range failed." << std::endl;
            }
            break;
        default:
            break;
        }

        std::cout.setf(std::ios::right);
        std::cout.fill('0');
        std::cout.width(2);
        std::cout << i << ". ";
        std::cout << property_item.name << "(" << (int)property_item.id << ")";
        std::cout << ", permission=" << PermissionTypeToString(property_item.permission) << ", range=" << strRange << std::endl;
    }
    std::cout << "------------------------------------------------------------------------\n";
}

bool Orbbec::IsPrimaryTypeProperty(OBPropertyItem propertyItem)
{
    return propertyItem.type == OB_INT_PROPERTY || propertyItem.type == OB_FLOAT_PROPERTY || propertyItem.type == OB_BOOL_PROPERTY;
}

std::string Orbbec::PermissionTypeToString(OBPermissionType permission)
{
    switch (permission)
    {
    case OB_PERMISSION_READ:
        return "R/_";
    case OB_PERMISSION_WRITE:
        return "_/W";
    case OB_PERMISSION_READ_WRITE:
        return "R/W";

    default:
        break;
    }
    return "_/_";
}

void Orbbec::EnableFrameSync(bool sync)
{
    if (!pipeline_)
        return;
    if (sync)
    {
        // enable frame sync inside the pipeline, which is synchronized by frame timestamp
        pipeline_->enableFrameSync();
    }
    else
    {
        // turn off sync
        pipeline_->disableFrameSync();
    }
}

bool Orbbec::CheckIfSupportHDW2CAlign(std::shared_ptr<ob::StreamProfile> color_stream_profile, std::shared_ptr<ob::StreamProfile> depth_sream_frofile)
{
    auto hwD2CSupportedDepthStreamProfiles = pipeline_->getD2CDepthProfileList(color_stream_profile, ALIGN_D2C_HW_MODE);
    if (hwD2CSupportedDepthStreamProfiles->count() == 0)
    {
        return false;
    }

    // Iterate through the supported depth stream profiles and check if there is a match with the given depth stream profile
    auto depthVsp = depth_sream_frofile->as<ob::VideoStreamProfile>();
    auto count = hwD2CSupportedDepthStreamProfiles->getCount();
    for (uint32_t i = 0; i < count; i++)
    {
        auto sp = hwD2CSupportedDepthStreamProfiles->getProfile(i);
        auto vsp = sp->as<ob::VideoStreamProfile>();
        if (vsp->getWidth() == depthVsp->getWidth() && vsp->getHeight() == depthVsp->getHeight() && vsp->getFormat() == depthVsp->getFormat() && vsp->getFps() == depthVsp->getFps())
        {
            // Found a matching depth stream profile, it is means the given stream profiles support hardware depth-to-color alignment
            std::cout << "Found a matching depth stream profile for hardware depth-to-color alignment: "
                      << "width: " << vsp->getWidth() << ", height: " << vsp->getHeight()
                      << ", format: " << vsp->getFormat() << ", fps: " << vsp->getFps() << std::endl;

            return true;
        }
    }
    return false;
}

std::shared_ptr<ob::Config> Orbbec::CreateHwD2CAlignConfig()
{
    auto coloStreamProfiles = pipeline_->getStreamProfileList(OB_SENSOR_COLOR);
    auto depthStreamProfiles = pipeline_->getStreamProfileList(OB_SENSOR_DEPTH);

    // Iterate through all color and depth stream profiles to find a match for hardware depth-to-color alignment
    auto colorSpCount = coloStreamProfiles->getCount();
    auto depthSpCount = depthStreamProfiles->getCount();
    for (uint32_t i = 0; i < colorSpCount; i++)
    {
        auto colorProfile = coloStreamProfiles->getProfile(i);
        auto colorVsp = colorProfile->as<ob::VideoStreamProfile>();
        for (uint32_t j = 0; j < depthSpCount; j++)
        {
            auto depthProfile = depthStreamProfiles->getProfile(j);
            auto depthVsp = depthProfile->as<ob::VideoStreamProfile>();

            // make sure the color and depth stream have the same fps, due to some models may not support different fps
            if (colorVsp->getFps() != depthVsp->getFps())
            {
                // If the fps of the color and depth streams are not the same, skip this pair
                continue;
            }

            // Check if the given stream profiles support hardware depth-to-color alignment
            if (CheckIfSupportHDW2CAlign(colorProfile, depthProfile))
            {
                // If support, create a config for hardware depth-to-color alignment
                auto hwD2CAlignConfig = std::make_shared<ob::Config>();
                hwD2CAlignConfig->enableStream(colorProfile);                                                    // enable color stream
                hwD2CAlignConfig->enableStream(depthProfile);                                                    // enable depth stream
                hwD2CAlignConfig->setAlignMode(ALIGN_D2C_HW_MODE);                                               // enable hardware depth-to-color alignment
                hwD2CAlignConfig->setFrameAggregateOutputMode(OB_FRAME_AGGREGATE_OUTPUT_ALL_TYPE_FRAME_REQUIRE); // output frameset with all types of frames
                return hwD2CAlignConfig;
            }
        }
    }
    return nullptr;
}

void Orbbec::GetProfilesSupport(OBSensorType sensor_type)
{
    auto stream_profiles = pipeline_->getStreamProfileList(OB_SENSOR_COLOR);
    auto profiles_ount = stream_profiles->getCount();
    for (uint32_t i = 0; i < profiles_ount; i++)
    {
        auto profile = stream_profiles->getProfile(i);
        auto sp = profile->as<ob::VideoStreamProfile>();
        std::cout << "Profile " << i << ": width: " << sp->getWidth() << ", height: " << sp->getHeight()
                  << ", fps: " << sp->getFps() << ", format: " << sp->getFormat() << std::endl;
    }
}
