/**
 * @file CalibrationParser.h
 * @brief Camera calibration file parser for visual odometry
 */

#ifndef OV_CORE_CALIBRATION_PARSER_H
#define OV_CORE_CALIBRATION_PARSER_H

#include <Eigen/Dense>
#include <opencv2/core.hpp>
#include <string>
#include <vector>

namespace ov_core {

/**
 * @brief Camera calibration data structure
 *
 * Contains camera intrinsic parameters and distortion coefficients.
 */
struct CameraCalibration {
  // Intrinsic parameters
  double fx = 500.0;  ///< Focal length x (pixels)
  double fy = 500.0;  ///< Focal length y (pixels)
  double cx = 320.0;  ///< Principal point x (pixels)
  double cy = 240.0;  ///< Principal point y (pixels)

  // Distortion coefficients (radtan model: k1, k2, p1, p2, k3)
  std::vector<double> distortion = {0.0, 0.0, 0.0, 0.0, 0.0};

  // Image resolution
  int image_width = 640;
  int image_height = 480;

  // Camera model type
  std::string camera_model = "pinhole";
  std::string distortion_model = "radtan";

  /**
   * @brief Get the camera intrinsic matrix K
   * @return 3x3 camera intrinsic matrix
   */
  Eigen::Matrix3d getK() const {
    Eigen::Matrix3d K = Eigen::Matrix3d::Identity();
    K(0, 0) = fx;
    K(1, 1) = fy;
    K(0, 2) = cx;
    K(1, 2) = cy;
    return K;
  }

  /**
   * @brief Get OpenCV camera matrix
   * @return OpenCV 3x3 camera matrix
   */
  cv::Mat getCameraMatrix() const {
    cv::Mat K = cv::Mat::eye(3, 3, CV_64F);
    K.at<double>(0, 0) = fx;
    K.at<double>(1, 1) = fy;
    K.at<double>(0, 2) = cx;
    K.at<double>(1, 2) = cy;
    return K;
  }

  /**
   * @brief Get OpenCV distortion coefficients
   * @return OpenCV distortion coefficient vector
   */
  cv::Mat getDistCoeffs() const {
    cv::Mat dist(static_cast<int>(distortion.size()), 1, CV_64F);
    for (size_t i = 0; i < distortion.size(); ++i) {
      dist.at<double>(static_cast<int>(i), 0) = distortion[i];
    }
    return dist;
  }

  /**
   * @brief Check if calibration has valid intrinsics
   * @return True if intrinsics are valid (positive focal lengths)
   */
  bool isValid() const { return fx > 0 && fy > 0 && cx > 0 && cy > 0; }
};

/**
 * @brief Parser for camera calibration YAML files
 *
 * Supports parsing calibration files in the Kalibr format used by OpenVINS.
 * Provides default values for uncalibrated cameras.
 */
class CalibrationParser {
public:
  /**
   * @brief Load calibration from a YAML file
   * @param filepath Path to the YAML calibration file
   * @param camera_name Camera name in the file (e.g., "cam0", "cam1")
   * @return CameraCalibration structure with loaded or default values
   */
  static CameraCalibration loadFromFile(const std::string &filepath,
                                        const std::string &camera_name = "cam0");

  /**
   * @brief Get default calibration for a typical webcam
   * @param image_width Image width in pixels
   * @param image_height Image height in pixels
   * @return CameraCalibration with reasonable default values
   */
  static CameraCalibration getDefaultCalibration(int image_width = 640,
                                                  int image_height = 480);

  /**
   * @brief Check if a calibration file exists and is readable
   * @param filepath Path to the calibration file
   * @return True if file exists and can be opened
   */
  static bool fileExists(const std::string &filepath);

private:
  /**
   * @brief Parse intrinsics array [fx, fy, cx, cy]
   * @param node OpenCV FileNode containing intrinsics
   * @param calib Output calibration structure
   * @return True if parsing succeeded
   */
  static bool parseIntrinsics(const cv::FileNode &node, CameraCalibration &calib);

  /**
   * @brief Parse distortion coefficients array
   * @param node OpenCV FileNode containing distortion_coeffs
   * @param calib Output calibration structure
   * @return True if parsing succeeded
   */
  static bool parseDistortion(const cv::FileNode &node, CameraCalibration &calib);

  /**
   * @brief Parse image resolution [width, height]
   * @param node OpenCV FileNode containing resolution
   * @param calib Output calibration structure
   * @return True if parsing succeeded
   */
  static bool parseResolution(const cv::FileNode &node, CameraCalibration &calib);
};

} // namespace ov_core

#endif // OV_CORE_CALIBRATION_PARSER_H
