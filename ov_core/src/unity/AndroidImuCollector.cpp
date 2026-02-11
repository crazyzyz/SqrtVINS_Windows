/**
 * @file AndroidImuCollector.cpp
 * @brief Implementation of native Android IMU sensor collector
 */

#if defined(__ANDROID__) || defined(ANDROID)

#include "AndroidImuCollector.h"
#include "utils/print.h"

#include <android/log.h>
#include <cmath>
#include <unistd.h>

#define LOOPER_ID_SENSOR 1

AndroidImuCollector::AndroidImuCollector() = default;

AndroidImuCollector::~AndroidImuCollector() { stop(); }

bool AndroidImuCollector::start(ImuCallback callback, int target_hz) {
  if (running_.load())
    return true;

  callback_ = std::move(callback);
  target_hz_ = target_hz;

  // Launch sensor thread
  running_.store(true);
  sensor_thread_ = std::thread(&AndroidImuCollector::sensorThreadFunc, this);

  return true;
}

void AndroidImuCollector::stop() {
  if (!running_.load())
    return;

  running_.store(false);

  // Wake up the looper so the thread can exit
  if (looper_) {
    ALooper_wake(looper_);
  }

  if (sensor_thread_.joinable()) {
    sensor_thread_.join();
  }
}

int AndroidImuCollector::sensorCallback(int fd, int events, void *data) {
  auto *self = static_cast<AndroidImuCollector *>(data);
  if (!self || !self->running_.load())
    return 0; // Returning 0 unregisters the callback

  ASensorEvent event;
  while (ASensorEventQueue_getEvents(self->event_queue_, &event, 1) > 0) {
    // Android sensor timestamp is in nanoseconds (CLOCK_BOOTTIME)
    double ts = event.timestamp * 1e-9;

    if (event.type == ASENSOR_TYPE_ACCELEROMETER) {
      std::lock_guard<std::mutex> lock(self->data_mutex_);
      // Android accelerometer includes gravity, which is what VIO expects
      self->last_ax_ = event.acceleration.x;
      self->last_ay_ = event.acceleration.y;
      self->last_az_ = event.acceleration.z;
      self->last_accel_ts_ = ts;
      self->has_accel_ = true;
    } else if (event.type == ASENSOR_TYPE_GYROSCOPE) {
      std::lock_guard<std::mutex> lock(self->data_mutex_);
      self->last_gx_ = event.gyro.x;
      self->last_gy_ = event.gyro.y;
      self->last_gz_ = event.gyro.z;
      self->last_gyro_ts_ = ts;
      self->has_gyro_ = true;
    }

    // When we have both readings, emit a synchronized sample
    // Use gyroscope timestamp as the reference (typically higher priority)
    {
      std::lock_guard<std::mutex> lock(self->data_mutex_);
      if (self->has_accel_ && self->has_gyro_ && self->callback_) {
        double imu_ts = self->last_gyro_ts_;
        self->callback_(imu_ts, self->last_ax_, self->last_ay_, self->last_az_,
                        self->last_gx_, self->last_gy_, self->last_gz_);
        // Reset flags so we wait for next pair
        self->has_accel_ = false;
        self->has_gyro_ = false;
      }
    }
  }

  return 1; // Continue receiving events
}

void AndroidImuCollector::sensorThreadFunc() {
  PRINT_INFO("[ImuCollector] Sensor thread started, target %d Hz\n",
             target_hz_);

  // Create looper for this thread
  looper_ = ALooper_prepare(ALOOPER_PREPARE_ALLOW_NON_CALLBACKS);
  if (!looper_) {
    PRINT_ERROR("[ImuCollector] Failed to create ALooper\n");
    running_.store(false);
    return;
  }

  // Get sensor manager
  sensor_manager_ = ASensorManager_getInstance();
  if (!sensor_manager_) {
    PRINT_ERROR("[ImuCollector] Failed to get ASensorManager\n");
    running_.store(false);
    return;
  }

  // Get sensors
  accelerometer_ = ASensorManager_getDefaultSensor(sensor_manager_,
                                                    ASENSOR_TYPE_ACCELEROMETER);
  gyroscope_ =
      ASensorManager_getDefaultSensor(sensor_manager_, ASENSOR_TYPE_GYROSCOPE);

  if (!accelerometer_ || !gyroscope_) {
    PRINT_ERROR("[ImuCollector] Accelerometer or gyroscope not available\n");
    running_.store(false);
    return;
  }

  // Create event queue
  event_queue_ = ASensorManager_createEventQueue(
      sensor_manager_, looper_, LOOPER_ID_SENSOR, sensorCallback, this);

  if (!event_queue_) {
    PRINT_ERROR("[ImuCollector] Failed to create sensor event queue\n");
    running_.store(false);
    return;
  }

  // Enable sensors at target rate
  int period_us = 1000000 / target_hz_;
  ASensorEventQueue_enableSensor(event_queue_, accelerometer_);
  ASensorEventQueue_setEventRate(event_queue_, accelerometer_, period_us);
  ASensorEventQueue_enableSensor(event_queue_, gyroscope_);
  ASensorEventQueue_setEventRate(event_queue_, gyroscope_, period_us);

  PRINT_INFO("[ImuCollector] Sensors enabled: accel + gyro at %d us period\n",
             period_us);

  // Event loop
  while (running_.load()) {
    // Poll with timeout (100ms) so we can check running_ flag
    int result = ALooper_pollOnce(100, nullptr, nullptr, nullptr);
    if (result == ALOOPER_POLL_ERROR) {
      PRINT_ERROR("[ImuCollector] ALooper_pollOnce error\n");
      break;
    }
  }

  // Cleanup
  ASensorEventQueue_disableSensor(event_queue_, accelerometer_);
  ASensorEventQueue_disableSensor(event_queue_, gyroscope_);
  ASensorManager_destroyEventQueue(sensor_manager_, event_queue_);
  event_queue_ = nullptr;
  looper_ = nullptr;

  PRINT_INFO("[ImuCollector] Sensor thread stopped\n");
}

#endif // __ANDROID__
