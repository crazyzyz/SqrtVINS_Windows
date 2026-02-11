/**
 * @file AndroidImuCollector.h
 * @brief Native Android IMU sensor collector using ASensorManager
 *
 * Collects accelerometer and gyroscope data at high frequency (200Hz)
 * directly from Android Sensor API, bypassing Unity's low-quality
 * Input.gyro/Input.acceleration.
 */

#ifndef ANDROID_IMU_COLLECTOR_H
#define ANDROID_IMU_COLLECTOR_H

#if defined(__ANDROID__) || defined(ANDROID)

#include <android/looper.h>
#include <android/sensor.h>

#include <atomic>
#include <functional>
#include <mutex>
#include <thread>

/**
 * @brief Callback type for IMU data
 * @param timestamp Timestamp in seconds (CLOCK_BOOTTIME)
 * @param ax,ay,az Accelerometer (m/s^2) in device body frame
 * @param gx,gy,gz Gyroscope (rad/s) in device body frame
 */
using ImuCallback = std::function<void(double timestamp, float ax, float ay,
                                       float az, float gx, float gy,
                                       float gz)>;

class AndroidImuCollector {
public:
  AndroidImuCollector();
  ~AndroidImuCollector();

  /**
   * @brief Start collecting IMU data
   * @param callback Function called for each synchronized IMU sample
   * @param target_hz Target sampling rate in Hz (default 200)
   * @return true if started successfully
   */
  bool start(ImuCallback callback, int target_hz = 200);

  /**
   * @brief Stop collecting IMU data
   */
  void stop();

  bool isRunning() const { return running_.load(); }

private:
  void sensorThreadFunc();

  static int sensorCallback(int fd, int events, void *data);

  ImuCallback callback_;
  int target_hz_ = 200;

  std::atomic<bool> running_{false};
  std::thread sensor_thread_;

  ASensorManager *sensor_manager_ = nullptr;
  ASensorEventQueue *event_queue_ = nullptr;
  const ASensor *accelerometer_ = nullptr;
  const ASensor *gyroscope_ = nullptr;
  ALooper *looper_ = nullptr;

  // Latest readings for synchronization
  std::mutex data_mutex_;
  float last_ax_ = 0, last_ay_ = 0, last_az_ = 0;
  float last_gx_ = 0, last_gy_ = 0, last_gz_ = 0;
  double last_accel_ts_ = 0;
  double last_gyro_ts_ = 0;
  bool has_accel_ = false;
  bool has_gyro_ = false;
};

#endif // __ANDROID__
#endif // ANDROID_IMU_COLLECTOR_H
