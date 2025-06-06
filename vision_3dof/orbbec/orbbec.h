#pragma once
#include <atomic>
#include <opencv2/opencv.hpp>

#include "common/xt_struct.h"
#include "libobsensor/ObSensor.hpp"
#include "libobsensor/hpp/Error.hpp"

class Orbbec
{
public:
  Orbbec(/* args */);
  ~Orbbec();
  std::shared_ptr<ob::DeviceList> GetDeviceList();
  std::shared_ptr<ob::Device> GetDevice(int index);
  std::shared_ptr<ob::Device> GetDevice(std::string serial_number);
  bool OpenDevice(std::string serial_number);
  bool OpenDevice(int index);
  bool CloseDevice();
  bool GetColorFrame(cv::Mat &mat);
  bool GetDepthFrame(cv::Mat &mat);
  bool GetIrFrame(std::vector<cv::Mat> &ir_mat_list);
  // 获取相机内参
  bool GetCameraIntrinsics(XtCameraDepthIntrinsics &depth_intrinsics);
  // void GetDepthFrame(std::shared_ptr<ob::Device> dev, ob::DepthFrame& depth_frame);
  // void GetIrFrame(std::shared_ptr<ob::Device> dev, ob::IrFrame& ir_frame);
  // void GetSkeletonFrame(std::shared_ptr<ob::Device> dev, ob::SkeletonFrame& skeleton_frame);
  // void GetPipeline(std::shared_ptr<ob::Device> dev);
private:
  /* data */
  ob::Context ob_ctx_;
  std::shared_ptr<ob::Pipeline> pipeline_;
  std::shared_ptr<ob::Context> context_;
  std::shared_ptr<ob::Config> config_;
  std::shared_ptr<ob::FormatConvertFilter> format_onverter_;
  std::atomic<bool> is_open_{false};
  int max_read_count_{10};
};
