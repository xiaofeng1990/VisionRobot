#pragma once
#include <atomic>
#include <opencv2/opencv.hpp>

#include "common/xt_struct.h"

#include "libobsensor/hpp/Error.hpp"
#include <libobsensor/ObSensor.hpp>
#include "libobsensor/hpp/Utils.hpp"
#include "libobsensor/hpp/Frame.hpp"
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
  bool GetColorMat(std::shared_ptr<ob::FrameSet> frame_set, cv::Mat &mat);
  bool GetDepthFrame(cv::Mat &mat);
  bool GetIrFrame(std::vector<cv::Mat> &ir_mat_list);
  std::vector<OBPropertyItem> GetPropertyList();
  void PrintfPropertyList();
  std::shared_ptr<ob::FrameSet> GetFrameSet();
  // 获取相机内参
  bool GetCameraIntrinsics(XtCameraDepthIntrinsics &depth_intrinsics);
  // void GetDepthFrame(std::shared_ptr<ob::Device> dev, ob::DepthFrame& depth_frame);
  // void GetIrFrame(std::shared_ptr<ob::Device> dev, ob::IrFrame& ir_frame);
  // void GetSkeletonFrame(std::shared_ptr<ob::Device> dev, ob::SkeletonFrame& skeleton_frame);
  // void GetPipeline(std::shared_ptr<ob::Device> dev);

  bool Transformation2dto3d(OBPoint2f source, OBPoint3f &target, std::shared_ptr<ob::FrameSet> frame_set);
  // 属性
  void EnableFrameSync(bool sync);

private:
  bool IsPrimaryTypeProperty(OBPropertyItem propertyItem);
  std::string PermissionTypeToString(OBPermissionType permission);

  bool CheckIfSupportHDW2CAlign(std::shared_ptr<ob::StreamProfile> color_stream_profile, std::shared_ptr<ob::StreamProfile> depth_sream_frofile);
  std::shared_ptr<ob::Config> CreateHwD2CAlignConfig();
  void PrintRuslt(std::string msg, OBPoint2f source, OBPoint3f target, float depth_value);

private:
  /* data */
  ob::Context ob_ctx_;
  std::shared_ptr<ob::Device> device_{nullptr};
  std::shared_ptr<ob::Pipeline> pipeline_{nullptr};
  std::shared_ptr<ob::Context> context_{nullptr};
  std::shared_ptr<ob::Config> config_{nullptr};
  std::shared_ptr<ob::FormatConvertFilter> format_onverter_{nullptr};
  std::atomic<bool> is_open_{false};
  int max_read_count_{10};
};
