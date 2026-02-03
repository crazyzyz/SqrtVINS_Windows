/**
 * @file MapManager.cpp
 * @brief Implementation of thread-safe map management for 3D points and camera trajectory
 */

#include "MapManager.h"

namespace ov_core {

void MapManager::addPoints(const std::vector<Eigen::Vector3d> &points) {
  std::lock_guard<std::mutex> lock(mutex_);
  for (const auto &pt : points) {
    MapPoint mp;
    mp.position = pt;
    mp.color = Eigen::Vector3f(1.0f, 1.0f, 1.0f); // Default white color
    mp.id = next_point_id_++;
    mp.observations = 1;
    points_.push_back(mp);
  }
}

void MapManager::addCameraPose(const Eigen::Matrix4d &pose) {
  std::lock_guard<std::mutex> lock(mutex_);
  trajectory_.push_back(pose);
}

std::vector<MapManager::MapPoint> MapManager::getPointCloud() const {
  std::lock_guard<std::mutex> lock(mutex_);
  return points_;
}

std::vector<Eigen::Matrix4d> MapManager::getTrajectory() const {
  std::lock_guard<std::mutex> lock(mutex_);
  return trajectory_;
}

Eigen::Matrix4d MapManager::getCurrentPose() const {
  std::lock_guard<std::mutex> lock(mutex_);
  if (trajectory_.empty()) {
    return Eigen::Matrix4d::Identity();
  }
  return trajectory_.back();
}

void MapManager::clear() {
  std::lock_guard<std::mutex> lock(mutex_);
  points_.clear();
  trajectory_.clear();
  next_point_id_ = 0;
}

size_t MapManager::getNumPoints() const {
  std::lock_guard<std::mutex> lock(mutex_);
  return points_.size();
}

} // namespace ov_core
