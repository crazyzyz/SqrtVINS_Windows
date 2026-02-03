/**
 * @file CalibrationParser.cpp
 * @brief Implementation of camera calibration file parser
 */

#include "CalibrationParser.h"
#include "utils/print.h"

#include <fstream>
#include <opencv2/core.hpp>

namespace ov_core {

CameraCalibration CalibrationParser::loadFromFile(const std::string &filepath,
                                                   const std::string &camera_name) {
  CameraCalibration calib;

  // Check if file exists
  if (!fileExists(filepath)) {
    PRINT_WARNING("Calibration file not found: %s\n", filepath.c_str());
    PRINT_WARNING("Using default calibration values.\n");
    return getDefaultCalibration();
  }

  // Open the YAML file
  cv::FileStorage fs(filepath, cv::FileStorage::READ);
  if (!fs.isOpened()) {
    PRINT_WARNING("Failed to open calibration file: %s\n", filepath.c_str());
    PRINT_WARNING("Using default calibration values.\n");
    return getDefaultCalibration();
  }

  // Get the camera node
  cv::FileNode cam_node = fs[camera_name];
  if (cam_node.empty()) {
    PRINT_WARNING("Camera '%s' not found in calibration file.\n", camera_name.c_str());
    PRINT_WARNING("Using default calibration values.\n");
    fs.release();
    return getDefaultCalibration();
  }

  // Parse camera model
  cv::FileNode model_node = cam_node["camera_model"];
  if (!model_node.empty()) {
    calib.camera_model = static_cast<std::string>(model_node);
  }

  // Parse distortion model
  cv::FileNode dist_model_node = cam_node["distortion_model"];
  if (!dist_model_node.empty()) {
    calib.distortion_model = static_cast<std::string>(dist_model_node);
  }

  // Parse intrinsics [fx, fy, cx, cy]
  cv::FileNode intrinsics_node = cam_node["intrinsics"];
  if (!parseIntrinsics(intrinsics_node, calib)) {
    PRINT_WARNING("Failed to parse intrinsics for '%s'.\n", camera_name.c_str());
    PRINT_WARNING("Using default intrinsic values.\n");
  }

  // Parse distortion coefficients
  cv::FileNode distortion_node = cam_node["distortion_coeffs"];
  if (!parseDistortion(distortion_node, calib)) {
    PRINT_WARNING("Failed to parse distortion for '%s'.\n", camera_name.c_str());
    PRINT_WARNING("Using zero distortion.\n");
    calib.distortion = {0.0, 0.0, 0.0, 0.0, 0.0};
  }

  // Parse resolution [width, height]
  cv::FileNode resolution_node = cam_node["resolution"];
  if (!parseResolution(resolution_node, calib)) {
    PRINT_WARNING("Failed to parse resolution for '%s'.\n", camera_name.c_str());
    PRINT_WARNING("Using default resolution.\n");
  }

  fs.release();

  PRINT_INFO("Loaded calibration for '%s':\n", camera_name.c_str());
  PRINT_INFO("  fx=%.3f, fy=%.3f, cx=%.3f, cy=%.3f\n", calib.fx, calib.fy, calib.cx, calib.cy);
  PRINT_INFO("  distortion: [%.6f, %.6f, %.6f, %.6f",
             calib.distortion.size() > 0 ? calib.distortion[0] : 0.0,
             calib.distortion.size() > 1 ? calib.distortion[1] : 0.0,
             calib.distortion.size() > 2 ? calib.distortion[2] : 0.0,
             calib.distortion.size() > 3 ? calib.distortion[3] : 0.0);
  if (calib.distortion.size() > 4) {
    PRINT_INFO(", %.6f]\n", calib.distortion[4]);
  } else {
    PRINT_INFO("]\n");
  }
  PRINT_INFO("  resolution: %dx%d\n", calib.image_width, calib.image_height);

  return calib;
}

CameraCalibration CalibrationParser::getDefaultCalibration(int image_width, int image_height) {
  CameraCalibration calib;

  // Set resolution
  calib.image_width = image_width;
  calib.image_height = image_height;

  // Estimate focal length from image size (typical webcam FOV ~60-70 degrees)
  // Using FOV = 2 * atan(sensor_size / (2 * focal_length))
  // For ~65 degree horizontal FOV: f ≈ width / (2 * tan(32.5°)) ≈ width * 0.78
  double focal_factor = 0.78;
  calib.fx = image_width * focal_factor;
  calib.fy = image_width * focal_factor;  // Assume square pixels

  // Principal point at image center
  calib.cx = image_width / 2.0;
  calib.cy = image_height / 2.0;

  // Zero distortion for uncalibrated camera
  calib.distortion = {0.0, 0.0, 0.0, 0.0, 0.0};

  calib.camera_model = "pinhole";
  calib.distortion_model = "radtan";

  PRINT_INFO("Using default calibration:\n");
  PRINT_INFO("  fx=%.3f, fy=%.3f, cx=%.3f, cy=%.3f\n", calib.fx, calib.fy, calib.cx, calib.cy);
  PRINT_INFO("  resolution: %dx%d\n", calib.image_width, calib.image_height);

  return calib;
}

bool CalibrationParser::fileExists(const std::string &filepath) {
  std::ifstream file(filepath);
  return file.good();
}

bool CalibrationParser::parseIntrinsics(const cv::FileNode &node, CameraCalibration &calib) {
  if (node.empty() || node.type() != cv::FileNode::SEQ) {
    return false;
  }

  std::vector<double> intrinsics;
  for (const auto &val : node) {
    intrinsics.push_back(static_cast<double>(val));
  }

  if (intrinsics.size() < 4) {
    return false;
  }

  // Kalibr format: [fx, fy, cx, cy]
  calib.fx = intrinsics[0];
  calib.fy = intrinsics[1];
  calib.cx = intrinsics[2];
  calib.cy = intrinsics[3];

  return true;
}

bool CalibrationParser::parseDistortion(const cv::FileNode &node, CameraCalibration &calib) {
  if (node.empty() || node.type() != cv::FileNode::SEQ) {
    return false;
  }

  calib.distortion.clear();
  for (const auto &val : node) {
    calib.distortion.push_back(static_cast<double>(val));
  }

  // Ensure at least 4 coefficients (radtan model: k1, k2, p1, p2)
  while (calib.distortion.size() < 4) {
    calib.distortion.push_back(0.0);
  }

  return true;
}

bool CalibrationParser::parseResolution(const cv::FileNode &node, CameraCalibration &calib) {
  if (node.empty() || node.type() != cv::FileNode::SEQ) {
    return false;
  }

  std::vector<int> resolution;
  for (const auto &val : node) {
    resolution.push_back(static_cast<int>(val));
  }

  if (resolution.size() < 2) {
    return false;
  }

  calib.image_width = resolution[0];
  calib.image_height = resolution[1];

  return true;
}

} // namespace ov_core
