#ifndef AI_INFER_INCLUDE_STRUCT_H_
#define AI_INFER_INCLUDE_STRUCT_H_
#include <future>
#include <memory>
#include <opencv2/opencv.hpp>
#include <string>
#include <vector>

#include "xt_status.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct _XtConfigExtra {
    // image save path
    const char* save_path;
    // save all input image
    // 'save_if_below_threshold' invalid if 'save_all' is true
    bool save_all;
    // save input image when detect result below threshold
    bool save_if_below_threshold;
} XtConfigExtra;

typedef struct _XtConfig {
    const char* model_path;
    float conf_thres;
    float iou_thres;
    XtConfigExtra extra;
    int reserve[32];
} XtConfig;

typedef struct _XtModelConfig {
    std::string model_file;
    std::string label_file;
    std::string backend_plugin;
    std::string type;
    int max_batch_size;
    int thread_pool_num;
    float confidence_threshold;
    float iou_threshold;
    bool visualize;
} XtModelConfig;

typedef struct _XtModbusConfig {
    int32_t slave_id;
    std::string ip;
    size_t port;
    uint32_t connect_timeout;
    uint32_t reconnect_interval;  // ms
    uint32_t retry_write_times;
    bool debug;
} XtModbusConfig;

typedef enum _XtImageType {
    ImageType_UNKNOWN,
    ImageType_GRAY,
    ImageType_JPEG,
    ImageType_BGR,
} XtImageType;

typedef enum _XtAngle {
    Angle_0,
    Angle_90,
    Angle_180,
    Angle_270,
} XtAngle;

typedef struct _XtBox {
    float left;
    float top;
    float right;
    float bottom;
} XtBox;

typedef struct _XtItem {
    int class_id;
    float confidence;
    std::string label;
    XtBox box;
    std::vector<std::vector<int>> ocr_box;
} XtItem;
typedef struct _XtCameraDepthIntrinsics {
    float fx;
    float fy;
    float cx;
    float cy;
    int16_t width;
    int16_t height;
} XtCameraDepthIntrinsics;

typedef struct _XtAllData {
    std::string uuid;
    std::string times;
    int modbus_data;
    cv::Mat color_image;
    cv::Mat depth_image;
    std::vector<cv::Mat> ir_image_list;
    // cv::Mat ir_image_left;
    // cv::Mat ir_image_right;
    XtCameraDepthIntrinsics depth_intrinsics;
} XtAllData;

#define XT_RESULT_ITEM_DESC_LEN 256
#define XT_RESULT_ITEM_COUNT 500

typedef struct _XtRect {
    int left;
    int top;
    int width;
    int height;
} XtRect;

#define XT_VERSION_CHAR_LEN 150
typedef struct _XtVersion {
    int major;
    int minor;
    int revision;
    char build[XT_VERSION_CHAR_LEN];
    char desc[XT_VERSION_CHAR_LEN];
} XtVersion;

typedef int XtRetCode;

#define XT_OK 0
#define XT_FAIL 1
#define XT_NULL_POINTER 2
#define XT_UNINITALIZED 3
#define XT_UNSUPPORTED 4

#define XT_MODEL_ERROR 10
#define XT_MODEL_NO_FILE 11
#define XT_MODEL_NO_PERMISSION 12

#define XT_IMAGE_NONSUPPORT 20

#ifdef __cplusplus
}
#endif

#endif  // AI_INFER_INCLUDE_STRUCT_H_
