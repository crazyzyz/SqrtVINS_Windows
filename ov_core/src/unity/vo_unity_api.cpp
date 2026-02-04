/**
 * @file vo_unity_api.cpp
 * @brief C API implementation for Unity Visual Odometry Library
 */

#include "vo_unity_api.h"
#include "VOUnityBridge.h"

#include <cstring>

// Version information
#define VO_VERSION_MAJOR 1
#define VO_VERSION_MINOR 1
#define VO_VERSION_PATCH 0

#if defined(__CYGWIN32__) || defined(WIN32) || defined(_WIN32) ||              \
    defined(__WIN32__) || defined(_WIN64) || defined(WINAPI_FAMILY)
#define UNITY_INTERFACE_API __stdcall
#else
#define UNITY_INTERFACE_API
#endif

// Static callback function that strictly matches Unity's signature
static void UNITY_INTERFACE_API OnRenderEvent(int eventID) {
  ov_core::VOUnityBridge::getInstance().onRenderEvent(eventID);
}

// ============================================================================
// Initialization and Shutdown
// ============================================================================

VOErrorCode vo_initialize(const VOCameraParams *camera_params,
                          const VOTrackingParams *tracking_params) {
  if (camera_params == nullptr) {
    return VO_ERROR_INVALID_PARAM;
  }

  try {
    return ov_core::VOUnityBridge::getInstance().initialize(*camera_params,
                                                            tracking_params);
  } catch (...) {
    return VO_ERROR_INVALID_PARAM;
  }
}

VOErrorCode vo_shutdown(void) {
  try {
    return ov_core::VOUnityBridge::getInstance().shutdown();
  } catch (...) {
    return VO_SUCCESS; // Shutdown should not fail
  }
}

int vo_is_initialized(void) {
  try {
    return ov_core::VOUnityBridge::getInstance().isInitialized() ? 1 : 0;
  } catch (...) {
    return 0;
  }
}

// ============================================================================
// Frame Processing
// ============================================================================

VOErrorCode vo_process_frame(const unsigned char *image_data, int width,
                             int height, int channels, double timestamp) {
  if (image_data == nullptr) {
    return VO_ERROR_INVALID_IMAGE;
  }

  try {
    return ov_core::VOUnityBridge::getInstance().processFrame(
        image_data, width, height, channels, timestamp);
  } catch (...) {
    return VO_ERROR_TRACKING_FAILED;
  }
}

// ============================================================================
// Data Retrieval
// ============================================================================

int vo_get_feature_count(void) {
  try {
    return ov_core::VOUnityBridge::getInstance().getFeatureCount();
  } catch (...) {
    return -1;
  }
}

VOErrorCode vo_get_features(VOFeature *features, int max_count,
                            int *out_count) {
  if (features == nullptr || out_count == nullptr || max_count <= 0) {
    return VO_ERROR_INVALID_PARAM;
  }

  try {
    return ov_core::VOUnityBridge::getInstance().getFeatures(
        features, max_count, out_count);
  } catch (...) {
    *out_count = 0;
    return VO_ERROR_TRACKING_FAILED;
  }
}

VOErrorCode vo_get_pose(VOPose *pose) {
  if (pose == nullptr) {
    return VO_ERROR_INVALID_PARAM;
  }

  try {
    return ov_core::VOUnityBridge::getInstance().getPose(pose);
  } catch (...) {
    std::memset(pose, 0, sizeof(VOPose));
    return VO_ERROR_POSE_FAILED;
  }
}

VOErrorCode vo_get_point_cloud(float *points, int max_points, int *out_count) {
  if (points == nullptr || out_count == nullptr || max_points <= 0) {
    return VO_ERROR_INVALID_PARAM;
  }

  try {
    return ov_core::VOUnityBridge::getInstance().getPointCloud(
        points, max_points, out_count);
  } catch (...) {
    *out_count = 0;
    return VO_ERROR_TRACKING_FAILED;
  }
}

VOErrorCode vo_get_debug_image(unsigned char *output_image, int width,
                               int height, int draw_points, int draw_flow) {
  if (output_image == nullptr) {
    return VO_ERROR_INVALID_PARAM;
  }

  try {
    return ov_core::VOUnityBridge::getInstance().getDebugImage(
        output_image, width, height, draw_points != 0, draw_flow != 0);
  } catch (...) {
    return VO_ERROR_TRACKING_FAILED;
  }
}

// ============================================================================
// Runtime Configuration
// ============================================================================

VOErrorCode vo_set_max_features(int count) {
  try {
    return ov_core::VOUnityBridge::getInstance().setMaxFeatures(count);
  } catch (...) {
    return VO_ERROR_INVALID_PARAM;
  }
}

VOErrorCode vo_set_fast_threshold(int threshold) {
  try {
    return ov_core::VOUnityBridge::getInstance().setFastThreshold(threshold);
  } catch (...) {
    return VO_ERROR_INVALID_PARAM;
  }
}

// ============================================================================
// Utility Functions
// ============================================================================

const char *vo_get_error_string(VOErrorCode code) {
  switch (code) {
  case VO_SUCCESS:
    return "Success";
  case VO_ERROR_NOT_INITIALIZED:
    return "Library not initialized";
  case VO_ERROR_ALREADY_INITIALIZED:
    return "Library already initialized";
  case VO_ERROR_INVALID_PARAM:
    return "Invalid parameter";
  case VO_ERROR_INVALID_IMAGE:
    return "Invalid image data or dimensions";
  case VO_ERROR_TRACKING_FAILED:
    return "Feature tracking failed";
  case VO_ERROR_POSE_FAILED:
    return "Pose estimation failed";
  default:
    return "Unknown error";
  }
}

void vo_get_version(int *major, int *minor, int *patch) {
  if (major)
    *major = VO_VERSION_MAJOR;
  if (minor)
    *minor = VO_VERSION_MINOR;
  if (patch)
    *patch = VO_VERSION_PATCH;
}

// ============================================================================
// Native Rendering Plugin Interface
// ============================================================================

extern "C" {

VO_API UnityRenderEventCallback vo_get_render_event_func(void) {
  return OnRenderEvent;
}

VO_API VOErrorCode vo_set_native_texture(void *texture_ptr, int width,
                                         int height) {
  return ov_core::VOUnityBridge::getInstance().setNativeTexture(texture_ptr,
                                                                width, height);
}

} // extern "C"
