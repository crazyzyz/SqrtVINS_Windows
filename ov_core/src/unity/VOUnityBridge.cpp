/**
 * @file VOUnityBridge.cpp
 * @brief Implementation of the Unity bridge class using full VioManager MSCKF
 */

#include "VOUnityBridge.h"

#include "cam/CamRadtan.h"
#include "core/VioManager.h"
#include "core/VioManagerOptions.h"
#include "state/State.h"
#include "track/TrackBase.h"
#include "types/IMU.h"
#include "utils/DataType.h"
#include "utils/print.h"
#include "utils/quat_ops.h"
#include "utils/sensor_data.h"

#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>

#include <cmath>
#include <cstring>

#if defined(__ANDROID__) || defined(ANDROID)
#include <GLES2/gl2.h>
#include <android/log.h>
#define LOGD(...) __android_log_print(ANDROID_LOG_DEBUG, "VOBridge", __VA_ARGS__)
#elif defined(_WIN32)
#include <GL/gl.h>
#include <windows.h>
#ifndef GL_RG
#define GL_RG 0x8227
#endif
#define LOGD(...) ((void)0)
#else
#define LOGD(...) ((void)0)
#endif

// ============================================================================
// Singleton & Lifecycle
// ============================================================================

VOUnityBridge &VOUnityBridge::getInstance() {
  static VOUnityBridge instance;
  return instance;
}

VOUnityBridge::VOUnityBridge() : vio_manager_(nullptr), initialized_(false) {
  std::memset(&last_valid_pose_, 0, sizeof(VOPose));
  last_valid_pose_.qw = 1.0f;
  last_valid_pose_.valid = 0;
}

VOUnityBridge::~VOUnityBridge() { shutdown(); }

// ============================================================================
// Initialize
// ============================================================================

VOErrorCode VOUnityBridge::initialize(const VOCameraParams &camera,
                                      const VOImuParams *imu_params,
                                      const VOExtrinsics *extrinsics,
                                      const VOTrackingParams *tracking) {
  std::lock_guard<std::mutex> lock(mutex_);

  // Validate camera parameters
  if (camera.fx <= 0 || camera.fy <= 0)
    return VO_ERROR_INVALID_PARAM;
  if (camera.width <= 0 || camera.height <= 0)
    return VO_ERROR_INVALID_PARAM;
  if (camera.cx < 0 || camera.cx >= camera.width || camera.cy < 0 ||
      camera.cy >= camera.height)
    return VO_ERROR_INVALID_PARAM;

  // Release previous resources
  if (initialized_) {
    vio_manager_.reset();
    initialized_ = false;
  }

  camera_params_ = camera;

  try {
    LOGD("initialize: building VioManagerOptions...");
    // ---------------------------------------------------------------
    // Build VioManagerOptions
    // ---------------------------------------------------------------
    ov_srvins::VioManagerOptions params;

    // --- State options ---
    params.state_options.num_cameras = 1;
    params.state_options.do_fej = true;
    params.state_options.use_rk4_integration = true;
    params.state_options.do_calib_camera_pose = false;
    params.state_options.do_calib_camera_intrinsics = false;
    params.state_options.do_calib_camera_timeoffset = false;
    params.state_options.max_clone_size = 11;
    params.state_options.max_slam_features = 25;

    // --- Camera intrinsics (CamRadtan) ---
    auto cam = std::make_shared<ov_core::CamRadtan>(camera.width, camera.height);
    VecX cam_values(8);
    cam_values << (DataType)camera.fx, (DataType)camera.fy,
                  (DataType)camera.cx, (DataType)camera.cy,
                  (DataType)camera.k1, (DataType)camera.k2,
                  (DataType)camera.p1, (DataType)camera.p2;
    cam->set_value(cam_values);
    params.camera_intrinsics[0] = cam;

    // --- Camera extrinsics (q_ItoC, p_IinC) ---
    // The extrinsics input is T_cam_imu (T_CfromI) as a 4x4 row-major matrix
    // VioManager expects a 7-element vector: [q_ItoC (JPL x,y,z,w), p_IinC (3)]
    VecX cam_ext(7);
    if (extrinsics != nullptr) {
      // Parse the 4x4 row-major T_cam_imu = T_CfromI
      Eigen::Matrix4d T_CfromI;
      for (int r = 0; r < 4; r++)
        for (int c = 0; c < 4; c++)
          T_CfromI(r, c) = (double)extrinsics->T_cam_imu[r * 4 + c];

      Eigen::Matrix3d R_ItoC = T_CfromI.block<3, 3>(0, 0);
      Eigen::Vector3d p_IinC = T_CfromI.block<3, 1>(0, 3);

      // Convert R_ItoC to JPL quaternion [x, y, z, w]
      Vec4 q_ItoC = ov_core::rot_2_quat(R_ItoC);

      cam_ext.head<4>() = q_ItoC;
      cam_ext.tail<3>() = p_IinC.cast<DataType>();
    } else {
      // Identity extrinsics: camera aligned with IMU
      cam_ext << 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0;
    }
    params.camera_extrinsics[0] = cam_ext;

    // Sync camera params to init_options (InertialInitializer also validates)
    params.init_options.num_cameras = 1;
    params.init_options.camera_intrinsics[0] = cam;
    params.init_options.camera_extrinsics[0] = cam_ext;

    LOGD("initialize: camera + extrinsics set, configuring IMU noise...");

    // --- IMU noise parameters ---
    if (imu_params != nullptr) {
      params.imu_noises.sigma_w = (DataType)imu_params->noise_gyro;
      params.imu_noises.sigma_w_2 =
          (DataType)(imu_params->noise_gyro * imu_params->noise_gyro);
      params.imu_noises.sigma_wb = (DataType)imu_params->gyro_walk;
      params.imu_noises.sigma_wb_2 =
          (DataType)(imu_params->gyro_walk * imu_params->gyro_walk);
      params.imu_noises.sigma_a = (DataType)imu_params->noise_acc;
      params.imu_noises.sigma_a_2 =
          (DataType)(imu_params->noise_acc * imu_params->noise_acc);
      params.imu_noises.sigma_ab = (DataType)imu_params->acc_walk;
      params.imu_noises.sigma_ab_2 =
          (DataType)(imu_params->acc_walk * imu_params->acc_walk);
    }
    // else: use NoiseManager defaults

    // --- Tracker parameters ---
    if (tracking != nullptr) {
      if (tracking->max_features > 0)
        params.num_pts = tracking->max_features;
      if (tracking->fast_threshold > 0)
        params.fast_threshold = tracking->fast_threshold;
      if (tracking->grid_x > 0)
        params.grid_x = tracking->grid_x;
      if (tracking->grid_y > 0)
        params.grid_y = tracking->grid_y;
      if (tracking->min_px_dist > 0)
        params.min_px_dist = tracking->min_px_dist;
    } else {
      // Reasonable defaults for mobile
      params.num_pts = 150;
      params.fast_threshold = 20;
      params.grid_x = 5;
      params.grid_y = 5;
      params.min_px_dist = 10;
    }

    // --- Other parameters ---
    params.use_stereo = false;  // Monocular
    params.use_klt = true;
    params.use_aruco = false;   // No ArUco on Android
    params.record_timing_information = false;
    params.try_zupt = true;     // Enable zero-velocity updates
    params.gravity_mag = 9.81;
    params.calib_camimu_dt = 0.0;

    LOGD("initialize: all params set, creating VioManager...");

    // --- Initialize VioManager ---
    vio_manager_ = std::make_shared<ov_srvins::VioManager>(params);

    LOGD("initialize: VioManager created successfully");
    initialized_ = true;
    frame_count_ = 0;

    // Reset last valid pose
    std::memset(&last_valid_pose_, 0, sizeof(VOPose));
    last_valid_pose_.qw = 1.0f;
    last_valid_pose_.valid = 0;

    PRINT_INFO("[VOBridge] VioManager initialized successfully\n");
    PRINT_INFO("[VOBridge] Camera: %dx%d, fx=%.1f fy=%.1f\n",
               camera.width, camera.height, camera.fx, camera.fy);
    if (imu_params)
      PRINT_INFO("[VOBridge] IMU noise: gw=%.6f, aw=%.6f\n",
                 imu_params->noise_gyro, imu_params->noise_acc);

    return VO_SUCCESS;
  } catch (const std::exception &e) {
    PRINT_ERROR("[VOBridge] Initialize failed: %s\n", e.what());
    vio_manager_.reset();
    return VO_ERROR_INVALID_PARAM;
  }
}

// ============================================================================
// Shutdown
// ============================================================================

VOErrorCode VOUnityBridge::shutdown() {
  std::lock_guard<std::mutex> lock(mutex_);

#if defined(__ANDROID__) || defined(ANDROID)
  if (imu_collector_) {
    imu_collector_->stop();
    imu_collector_.reset();
    imu_use_native_ = false;
  }
#endif

  vio_manager_.reset();
  initialized_ = false;
  frame_count_ = 0;

  std::memset(&last_valid_pose_, 0, sizeof(VOPose));
  last_valid_pose_.qw = 1.0f;
  last_valid_pose_.valid = 0;

  return VO_SUCCESS;
}

bool VOUnityBridge::isInitialized() const {
  std::lock_guard<std::mutex> lock(mutex_);
  return initialized_;
}

// ============================================================================
// Feed IMU
// ============================================================================

VOErrorCode VOUnityBridge::feedImu(const VOImuData &imu) {
  std::lock_guard<std::mutex> lock(mutex_);

  if (!initialized_ || vio_manager_ == nullptr)
    return VO_ERROR_NOT_INITIALIZED;

  try {
    ov_core::ImuData msg;
    msg.timestamp = imu.timestamp;
    // gyroscope reading (rad/s)
    msg.wm << (DataType)imu.gx, (DataType)imu.gy, (DataType)imu.gz;
    // accelerometer reading (m/s^2)
    msg.am << (DataType)imu.ax, (DataType)imu.ay, (DataType)imu.az;

    vio_manager_->feed_measurement_imu(msg);
    return VO_SUCCESS;
  } catch (const std::exception &e) {
    PRINT_ERROR("[VOBridge] feedImu failed: %s\n", e.what());
    return VO_ERROR_TRACKING_FAILED;
  }
}

// ============================================================================
// Process Frame
// ============================================================================

VOErrorCode VOUnityBridge::processFrame(const uint8_t *data, int width,
                                        int height, int channels,
                                        double timestamp) {
  std::lock_guard<std::mutex> lock(mutex_);

  if (!initialized_ || vio_manager_ == nullptr)
    return VO_ERROR_NOT_INITIALIZED;

  if (width != camera_params_.width || height != camera_params_.height)
    return VO_ERROR_INVALID_IMAGE;

  if (channels != 1 && channels != 4)
    return VO_ERROR_INVALID_IMAGE;

  try {
    cv::Mat gray;

    if (channels == 1) {
      gray = cv::Mat(height, width, CV_8UC1, const_cast<uint8_t *>(data)).clone();
    } else if (channels == 4) {
      cv::Mat rgba(height, width, CV_8UC4, const_cast<uint8_t *>(data));
      cv::cvtColor(rgba, gray, cv::COLOR_RGBA2GRAY);
    }

    // Save for debug visualization
    current_frame_ = gray.clone();

    // Build CameraData message
    ov_core::CameraData msg;
    msg.timestamp = timestamp;
    msg.sensor_ids.push_back(0);      // Camera 0
    msg.images.push_back(gray);
    msg.masks.push_back(cv::Mat::zeros(gray.size(), CV_8UC1));

    // Feed to VioManager
    vio_manager_->feed_measurement_camera(msg);

    frame_count_++;
    if (frame_count_ % 30 == 0) {
      bool is_init = vio_manager_->initialized();
      PRINT_DEBUG("[VOBridge] Frame %d, VIO initialized: %d\n",
                  frame_count_, is_init ? 1 : 0);
    }

    return VO_SUCCESS;
  } catch (const std::exception &e) {
    PRINT_ERROR("[VOBridge] processFrame failed: %s\n", e.what());
    return VO_ERROR_TRACKING_FAILED;
  }
}

// ============================================================================
// Get Pose
// ============================================================================

VOErrorCode VOUnityBridge::getPose(VOPose *out) const {
  std::lock_guard<std::mutex> lock(mutex_);

  if (!initialized_ || vio_manager_ == nullptr) {
    *out = last_valid_pose_;
    return VO_ERROR_NOT_INITIALIZED;
  }

  // Check if VIO has been initialized (static/dynamic init complete)
  if (!vio_manager_->initialized()) {
    out->px = 0;
    out->py = 0;
    out->pz = 0;
    out->qx = 0;
    out->qy = 0;
    out->qz = 0;
    out->qw = 1.0f;
    out->valid = 0;
    return VO_SUCCESS;
  }

  try {
    auto state = vio_manager_->get_state();
    if (!state || !state->imu) {
      *out = last_valid_pose_;
      return VO_ERROR_POSE_FAILED;
    }

    // Extract pose from state:
    // imu->Rot() = R_GtoI (rotation from global to IMU)
    // imu->pos() = p_IinG (position of IMU in global)
    Eigen::Matrix<DataType, 3, 3> R_GtoI = state->imu->Rot();
    Eigen::Matrix<DataType, 3, 1> p_IinG = state->imu->pos();

    // Convert to double for coordinate conversion
    Eigen::Matrix3d R_GtoI_d = R_GtoI.template cast<double>();
    Eigen::Vector3d p_IinG_d = p_IinG.template cast<double>();

    convertToUnityCoordinates(p_IinG_d, R_GtoI_d, *out);

    const_cast<VOUnityBridge *>(this)->last_valid_pose_ = *out;
    return VO_SUCCESS;
  } catch (const std::exception &e) {
    PRINT_ERROR("[VOBridge] getPose failed: %s\n", e.what());
    *out = last_valid_pose_;
    return VO_ERROR_POSE_FAILED;
  }
}

// ============================================================================
// Coordinate Conversion
// ============================================================================

void VOUnityBridge::convertToUnityCoordinates(const Eigen::Vector3d &p_IinG,
                                               const Eigen::Matrix3d &R_GtoI,
                                               VOPose &out) const {
  // sqrtVINS (OpenVINS convention):
  //   Right-handed, gravity-aligned global frame
  //   X-right/east, Y-forward/north, Z-up
  //   R_GtoI: rotation from global to IMU body
  //   p_IinG: position of IMU in global frame
  //
  // Unity:
  //   Left-handed, Y-up
  //   X-right, Y-up, Z-forward
  //
  // The key insight: swapping Y<->Z converts between these two systems.
  // The permutation matrix P = [1 0 0; 0 0 1; 0 1 0] has det=-1,
  // which naturally handles the handedness flip (right->left).
  // No additional axis negation is needed.

  // Position conversion: swap Y and Z
  out.px = static_cast<float>(p_IinG.x());   // X -> X
  out.py = static_cast<float>(p_IinG.z());   // Z -> Y (up)
  out.pz = static_cast<float>(p_IinG.y());   // Y -> Z (forward)

  // Rotation conversion:
  // We need the body orientation in Unity world frame.
  // R_ItoG = R_GtoI^T gives the body-to-world rotation in VIO coords.
  // Then apply the same Y<->Z permutation to both sides:
  //   R_unity = P * R_ItoG * P^T
  // where P = [1 0 0; 0 0 1; 0 1 0] (P is symmetric, so P^T = P = P^{-1})
  Eigen::Matrix3d R_ItoG = R_GtoI.transpose();

  Eigen::Matrix3d P;
  P << 1, 0, 0,
       0, 0, 1,
       0, 1, 0;

  Eigen::Matrix3d R_unity = P * R_ItoG * P;

  // Convert to quaternion (Eigen uses Hamilton convention, same as Unity)
  Eigen::Quaterniond q(R_unity);
  q.normalize();

  out.qx = static_cast<float>(q.x());
  out.qy = static_cast<float>(q.y());
  out.qz = static_cast<float>(q.z());
  out.qw = static_cast<float>(q.w());
  out.valid = 1;
}

// ============================================================================
// Feature Access
// ============================================================================

int VOUnityBridge::getFeatureCount() const {
  std::lock_guard<std::mutex> lock(mutex_);

  if (!initialized_ || vio_manager_ == nullptr)
    return -1;

  // Get the tracked features from VioManager's internal tracker
  // VioManager has trackFEATS but it's protected; we access via get_historical_viz_image
  // Alternative: access the state's good_features_MSCKF
  auto good_feats = vio_manager_->get_good_features_MSCKF();
  return static_cast<int>(good_feats.size());
}

VOErrorCode VOUnityBridge::getFeatures(VOFeature *out, int maxCount,
                                       int *outCount) const {
  std::lock_guard<std::mutex> lock(mutex_);

  if (!initialized_ || vio_manager_ == nullptr) {
    *outCount = 0;
    return VO_ERROR_NOT_INITIALIZED;
  }

  // We can report SLAM features that are in the state
  auto state = vio_manager_->get_state();
  if (!state) {
    *outCount = 0;
    return VO_SUCCESS;
  }

  int count = 0;
  for (auto &feat_pair : state->features_SLAM) {
    if (count >= maxCount)
      break;
    auto &feat = feat_pair.second;
    auto feat_val = feat->get_xyz(false);
    out[count].id = static_cast<int>(feat_pair.first);
    // Project to 2D would require camera model; for now report 3D x,y
    out[count].x = static_cast<float>(feat_val(0));
    out[count].y = static_cast<float>(feat_val(1));
    out[count].status = 1; // Tracked
    count++;
  }

  *outCount = count;
  return VO_SUCCESS;
}

VOErrorCode VOUnityBridge::getPointCloud(float *out, int maxPoints,
                                         int *outCount) const {
  std::lock_guard<std::mutex> lock(mutex_);

  if (!initialized_ || vio_manager_ == nullptr) {
    *outCount = 0;
    return VO_ERROR_NOT_INITIALIZED;
  }

  try {
    // Get SLAM features in global frame
    auto slam_feats = vio_manager_->get_features_SLAM();
    auto msckf_feats = vio_manager_->get_good_features_MSCKF();

    int count = 0;

    // Add SLAM features
    for (auto &pt : slam_feats) {
      if (count >= maxPoints)
        break;
      // Convert to Unity coordinates (same permutation as pose)
      out[count * 3 + 0] = static_cast<float>(pt(0));      // X
      out[count * 3 + 1] = static_cast<float>(pt(2));      // Z -> Y (up)
      out[count * 3 + 2] = static_cast<float>(pt(1));      // Y -> Z (forward)
      count++;
    }

    // Add MSCKF features
    for (auto &pt : msckf_feats) {
      if (count >= maxPoints)
        break;
      out[count * 3 + 0] = static_cast<float>(pt(0));
      out[count * 3 + 1] = static_cast<float>(pt(2));
      out[count * 3 + 2] = static_cast<float>(pt(1));
      count++;
    }

    *outCount = count;
    return VO_SUCCESS;
  } catch (const std::exception &e) {
    *outCount = 0;
    return VO_ERROR_TRACKING_FAILED;
  }
}

// ============================================================================
// Runtime Configuration
// ============================================================================

VOErrorCode VOUnityBridge::setMaxFeatures(int count) {
  std::lock_guard<std::mutex> lock(mutex_);
  if (count <= 0)
    return VO_ERROR_INVALID_PARAM;
  // Runtime feature count change is not directly supported by VioManager
  // The parameter is set at construction time
  return VO_SUCCESS;
}

VOErrorCode VOUnityBridge::setFastThreshold(int threshold) {
  std::lock_guard<std::mutex> lock(mutex_);
  if (threshold <= 0)
    return VO_ERROR_INVALID_PARAM;
  // Runtime threshold change is not directly supported by VioManager
  return VO_SUCCESS;
}

// ============================================================================
// Debug Image
// ============================================================================

VOErrorCode VOUnityBridge::getDebugImage(uint8_t *out, int width, int height,
                                         bool drawPoints, bool drawFlow) const {
  std::lock_guard<std::mutex> lock(mutex_);

  if (!initialized_ || vio_manager_ == nullptr)
    return VO_ERROR_NOT_INITIALIZED;

  if (out == nullptr || width != camera_params_.width ||
      height != camera_params_.height)
    return VO_ERROR_INVALID_PARAM;

  if (current_frame_.empty())
    return VO_ERROR_INVALID_IMAGE;

  try {
    // Use VioManager's built-in visualization if available
    cv::Mat vizImg = vio_manager_->get_historical_viz_image();

    cv::Mat debugImg;
    if (!vizImg.empty()) {
      // VioManager provides a visualization with tracked features
      // Resize to match expected output if needed
      if (vizImg.rows != height || vizImg.cols != width) {
        cv::resize(vizImg, vizImg, cv::Size(width, height));
      }
      // Convert to RGBA
      if (vizImg.channels() == 3) {
        cv::cvtColor(vizImg, debugImg, cv::COLOR_BGR2RGBA);
      } else if (vizImg.channels() == 1) {
        cv::cvtColor(vizImg, debugImg, cv::COLOR_GRAY2RGBA);
      } else if (vizImg.channels() == 4) {
        debugImg = vizImg;
      } else {
        cv::cvtColor(current_frame_, debugImg, cv::COLOR_GRAY2RGBA);
      }
    } else {
      // Fallback: draw on current grayscale frame
      cv::cvtColor(current_frame_, debugImg, cv::COLOR_GRAY2RGBA);

      // Draw good MSCKF features (projected would require camera model)
      // For now, just show the grayscale frame
    }

    std::memcpy(out, debugImg.data, width * height * 4);
    return VO_SUCCESS;
  } catch (const std::exception &e) {
    return VO_ERROR_TRACKING_FAILED;
  }
}

// ============================================================================
// Native Texture
// ============================================================================

VOErrorCode VOUnityBridge::setNativeTexture(void *ptr, int width, int height) {
  std::lock_guard<std::mutex> lock(mutex_);
  native_texture_ptr_ = ptr;
  texture_width_ = width;
  texture_height_ = height;
  return VO_SUCCESS;
}

void VOUnityBridge::onRenderEvent(int eventID) {
  if (eventID != 1)
    return;

  std::lock_guard<std::mutex> lock(mutex_);

  if (native_texture_ptr_ == nullptr || current_frame_.empty())
    return;
  if (texture_width_ == 0 || texture_height_ == 0)
    return;

  while (glGetError() != GL_NO_ERROR)
    ;

  try {
    // Get visualization from VioManager
    cv::Mat vizImg = vio_manager_ ? vio_manager_->get_historical_viz_image()
                                  : cv::Mat();

    cv::Mat debugImg;
    if (!vizImg.empty()) {
      if (vizImg.channels() == 3)
        cv::cvtColor(vizImg, debugImg, cv::COLOR_BGR2RGBA);
      else if (vizImg.channels() == 1)
        cv::cvtColor(vizImg, debugImg, cv::COLOR_GRAY2RGBA);
      else
        debugImg = vizImg.clone();
    } else {
      cv::cvtColor(current_frame_, debugImg, cv::COLOR_GRAY2RGBA);
    }

    if (debugImg.rows != texture_height_ || debugImg.cols != texture_width_)
      cv::resize(debugImg, debugImg, cv::Size(texture_width_, texture_height_));

    GLuint glTexId = (GLuint)(size_t)(native_texture_ptr_);
    glBindTexture(GL_TEXTURE_2D, glTexId);
    glTexSubImage2D(GL_TEXTURE_2D, 0, 0, 0, texture_width_, texture_height_,
                    GL_RGBA, GL_UNSIGNED_BYTE, debugImg.data);
    glBindTexture(GL_TEXTURE_2D, 0);
  } catch (...) {
    // Silently ignore errors in render thread
  }
}

// ============================================================================
// Native IMU Collection
// ============================================================================

VOErrorCode VOUnityBridge::startNativeImu(int target_hz) {
#if defined(__ANDROID__) || defined(ANDROID)
  std::lock_guard<std::mutex> lock(mutex_);

  if (!initialized_ || vio_manager_ == nullptr)
    return VO_ERROR_NOT_INITIALIZED;

  if (imu_collector_ && imu_collector_->isRunning())
    return VO_SUCCESS; // Already running

  imu_collector_ = std::make_unique<AndroidImuCollector>();

  // The callback converts Android device-frame IMU to VIO frame and feeds it
  auto callback = [this](double timestamp, float ax, float ay, float az,
                         float gx, float gy, float gz) {
    // Store native sensor timestamp for camera sync
    native_sensor_ts_.store(timestamp, std::memory_order_relaxed);

    // Android device body frame -> VIO frame (right-handed, Z-up)
    // Android: X=right, Y=up, Z=out-of-screen
    // VIO:     X=right, Y=forward, Z=up
    // Mapping: VIO_x = Dev_x, VIO_y = -Dev_z, VIO_z = Dev_y
    float ax_vio = ax;
    float ay_vio = -az;
    float az_vio = ay;
    float gx_vio = gx;
    float gy_vio = -gz;
    float gz_vio = gy;

    VOImuData imu;
    imu.timestamp = timestamp;
    imu.ax = ax_vio;
    imu.ay = ay_vio;
    imu.az = az_vio;
    imu.gx = gx_vio;
    imu.gy = gy_vio;
    imu.gz = gz_vio;

    // Feed directly (feedImu acquires its own lock)
    // We must NOT hold mutex_ here to avoid deadlock
    // So we release the lock before calling feedImu
    this->feedImu(imu);
  };

  if (!imu_collector_->start(callback, target_hz)) {
    imu_collector_.reset();
    return VO_ERROR_TRACKING_FAILED;
  }

  imu_use_native_ = true;
  PRINT_INFO("[VOBridge] Native IMU started at %d Hz\n", target_hz);
  return VO_SUCCESS;
#else
  (void)target_hz;
  return VO_ERROR_INVALID_PARAM; // Not supported on non-Android
#endif
}

VOErrorCode VOUnityBridge::stopNativeImu() {
#if defined(__ANDROID__) || defined(ANDROID)
  std::lock_guard<std::mutex> lock(mutex_);

  if (imu_collector_) {
    imu_collector_->stop();
    imu_collector_.reset();
    imu_use_native_ = false;
    PRINT_INFO("[VOBridge] Native IMU stopped\n");
  }
  return VO_SUCCESS;
#else
  return VO_SUCCESS;
#endif
}

bool VOUnityBridge::isNativeImuRunning() const {
#if defined(__ANDROID__) || defined(ANDROID)
  return imu_collector_ && imu_collector_->isRunning();
#else
  return false;
#endif
}

double VOUnityBridge::getNativeSensorTimestamp() const {
  return native_sensor_ts_.load(std::memory_order_relaxed);
}

// ============================================================================
// Reset
// ============================================================================

VOErrorCode VOUnityBridge::resetImu() {
  std::lock_guard<std::mutex> lock(mutex_);
  // VioManager handles IMU state internally; a full reset would require
  // re-initialization. For now, this is a no-op.
  return VO_SUCCESS;
}
