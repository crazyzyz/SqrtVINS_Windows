/**
 * @file PangolinViewer.cpp
 * @brief Implementation of 3D visualization using Pangolin library
 */

#include "PangolinViewer.h"

#if defined(HAVE_PANGOLIN) && HAVE_PANGOLIN

#include <pangolin/pangolin.h>
#include <pangolin/gl/gl.h>

namespace ov_core {

PangolinViewer::PangolinViewer(const Config &config)
    : config_(config), running_(false), quit_requested_(false) {
  // Initialize current pose to identity
  current_pose_ = Eigen::Matrix4d::Identity();
}

PangolinViewer::~PangolinViewer() {
  stop();
}

void PangolinViewer::start() {
  if (running_.load()) {
    return; // Already running
  }
  running_.store(true);
  quit_requested_.store(false);
  render_thread_ = std::thread(&PangolinViewer::renderLoop, this);
}

void PangolinViewer::stop() {
  if (!running_.load()) {
    return; // Not running
  }
  running_.store(false);
  if (render_thread_.joinable()) {
    render_thread_.join();
  }
}

void PangolinViewer::updatePointCloud(const std::vector<Eigen::Vector3d> &points) {
  std::lock_guard<std::mutex> lock(data_mutex_);
  points_ = points;
}

void PangolinViewer::updateTrajectory(const std::vector<Eigen::Matrix4d> &poses) {
  std::lock_guard<std::mutex> lock(data_mutex_);
  trajectory_ = poses;
}

void PangolinViewer::updateCurrentPose(const Eigen::Matrix4d &pose) {
  std::lock_guard<std::mutex> lock(data_mutex_);
  current_pose_ = pose;
}

void PangolinViewer::updateImage(const cv::Mat &image) {
  std::lock_guard<std::mutex> lock(data_mutex_);
  current_image_ = image.clone();
}

bool PangolinViewer::shouldQuit() const {
  return quit_requested_.load();
}


void PangolinViewer::renderLoop() {
  // Create Pangolin window with OpenGL context
  pangolin::CreateWindowAndBind(config_.window_name, config_.window_width, config_.window_height);

  // Enable depth testing for proper 3D rendering
  glEnable(GL_DEPTH_TEST);

  // Enable blending for transparency
  glEnable(GL_BLEND);
  glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

  // Define projection and initial model-view matrix
  // Set up camera with reasonable defaults for SLAM visualization
  pangolin::OpenGlRenderState s_cam(
      pangolin::ProjectionMatrix(config_.window_width, config_.window_height, 500, 500,
                                  config_.window_width / 2, config_.window_height / 2, 0.1, 1000),
      pangolin::ModelViewLookAt(0, -2, -5, 0, 0, 0, pangolin::AxisNegY));

  // Create interactive view with mouse handlers
  pangolin::Handler3D handler(s_cam);
  pangolin::View &d_cam = pangolin::CreateDisplay()
                              .SetBounds(0.0, 1.0, 0.0, 1.0, -(float)config_.window_width / config_.window_height)
                              .SetHandler(&handler);

  // Main rendering loop
  while (!pangolin::ShouldQuit() && running_.load()) {
    // Clear screen and depth buffer
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    // Set background color (dark gray)
    glClearColor(0.1f, 0.1f, 0.1f, 1.0f);

    // Activate the 3D view
    d_cam.Activate(s_cam);

    // Draw coordinate axes at origin
    glLineWidth(2.0f);
    glBegin(GL_LINES);
    // X axis - Red
    glColor3f(1.0f, 0.0f, 0.0f);
    glVertex3f(0.0f, 0.0f, 0.0f);
    glVertex3f(0.5f, 0.0f, 0.0f);
    // Y axis - Green
    glColor3f(0.0f, 1.0f, 0.0f);
    glVertex3f(0.0f, 0.0f, 0.0f);
    glVertex3f(0.0f, 0.5f, 0.0f);
    // Z axis - Blue
    glColor3f(0.0f, 0.0f, 1.0f);
    glVertex3f(0.0f, 0.0f, 0.0f);
    glVertex3f(0.0f, 0.0f, 0.5f);
    glEnd();

    // Draw all visualization elements
    drawPointCloud();
    drawTrajectory();
    drawCurrentCamera();

    // Swap buffers and process events
    pangolin::FinishFrame();
  }

  // Signal quit was requested
  quit_requested_.store(true);

  // Destroy the window
  pangolin::DestroyWindow(config_.window_name);
}


void PangolinViewer::drawPointCloud() {
  std::lock_guard<std::mutex> lock(data_mutex_);

  if (points_.empty()) {
    return;
  }

  // Set point size
  glPointSize(config_.point_size);

  // Draw points
  glBegin(GL_POINTS);
  for (const auto &pt : points_) {
    // Use a gradient color based on height (Y coordinate)
    // This gives visual depth perception
    float height_normalized = static_cast<float>((pt.y() + 5.0) / 10.0);
    height_normalized = std::max(0.0f, std::min(1.0f, height_normalized));

    // Color gradient from blue (low) to red (high)
    glColor3f(height_normalized, 0.5f, 1.0f - height_normalized);
    glVertex3d(pt.x(), pt.y(), pt.z());
  }
  glEnd();
}

void PangolinViewer::drawTrajectory() {
  std::lock_guard<std::mutex> lock(data_mutex_);

  if (trajectory_.size() < 2) {
    return;
  }

  // Draw trajectory as connected line strip
  glLineWidth(2.0f);
  glColor3f(0.0f, 1.0f, 0.0f); // Green trajectory

  glBegin(GL_LINE_STRIP);
  for (const auto &pose : trajectory_) {
    // Extract camera position (translation from world to camera)
    // The camera position in world frame is -R^T * t
    Eigen::Matrix3d R = pose.block<3, 3>(0, 0);
    Eigen::Vector3d t = pose.block<3, 1>(0, 3);
    Eigen::Vector3d cam_pos = -R.transpose() * t;

    glVertex3d(cam_pos.x(), cam_pos.y(), cam_pos.z());
  }
  glEnd();
}

void PangolinViewer::drawCurrentCamera() {
  std::lock_guard<std::mutex> lock(data_mutex_);
  drawCameraFrustum(current_pose_, config_.camera_size);
}


void PangolinViewer::drawCameraFrustum(const Eigen::Matrix4d &pose, float size) {
  // Extract rotation and translation
  Eigen::Matrix3d R = pose.block<3, 3>(0, 0);
  Eigen::Vector3d t = pose.block<3, 1>(0, 3);

  // Camera position in world frame: -R^T * t
  Eigen::Vector3d cam_pos = -R.transpose() * t;

  // Camera axes in world frame
  Eigen::Vector3d x_axis = R.transpose().col(0) * size;
  Eigen::Vector3d y_axis = R.transpose().col(1) * size;
  Eigen::Vector3d z_axis = R.transpose().col(2) * size;

  // Frustum corners (in camera frame, then transformed to world)
  float w = size * 0.6f;  // Width
  float h = size * 0.4f;  // Height
  float z = size * 0.8f;  // Depth

  // Four corners of the image plane in camera frame
  Eigen::Vector3d p1(-w, -h, z);
  Eigen::Vector3d p2(w, -h, z);
  Eigen::Vector3d p3(w, h, z);
  Eigen::Vector3d p4(-w, h, z);

  // Transform to world frame
  p1 = cam_pos + R.transpose() * p1;
  p2 = cam_pos + R.transpose() * p2;
  p3 = cam_pos + R.transpose() * p3;
  p4 = cam_pos + R.transpose() * p4;

  // Draw camera frustum
  glLineWidth(2.0f);
  glColor3f(1.0f, 0.0f, 0.0f); // Red for current camera

  glBegin(GL_LINES);
  // Lines from camera center to corners
  glVertex3d(cam_pos.x(), cam_pos.y(), cam_pos.z());
  glVertex3d(p1.x(), p1.y(), p1.z());

  glVertex3d(cam_pos.x(), cam_pos.y(), cam_pos.z());
  glVertex3d(p2.x(), p2.y(), p2.z());

  glVertex3d(cam_pos.x(), cam_pos.y(), cam_pos.z());
  glVertex3d(p3.x(), p3.y(), p3.z());

  glVertex3d(cam_pos.x(), cam_pos.y(), cam_pos.z());
  glVertex3d(p4.x(), p4.y(), p4.z());

  // Image plane rectangle
  glVertex3d(p1.x(), p1.y(), p1.z());
  glVertex3d(p2.x(), p2.y(), p2.z());

  glVertex3d(p2.x(), p2.y(), p2.z());
  glVertex3d(p3.x(), p3.y(), p3.z());

  glVertex3d(p3.x(), p3.y(), p3.z());
  glVertex3d(p4.x(), p4.y(), p4.z());

  glVertex3d(p4.x(), p4.y(), p4.z());
  glVertex3d(p1.x(), p1.y(), p1.z());
  glEnd();

  // Draw camera coordinate axes
  glLineWidth(3.0f);
  glBegin(GL_LINES);
  // X axis - Red
  glColor3f(1.0f, 0.0f, 0.0f);
  glVertex3d(cam_pos.x(), cam_pos.y(), cam_pos.z());
  glVertex3d(cam_pos.x() + x_axis.x(), cam_pos.y() + x_axis.y(), cam_pos.z() + x_axis.z());

  // Y axis - Green
  glColor3f(0.0f, 1.0f, 0.0f);
  glVertex3d(cam_pos.x(), cam_pos.y(), cam_pos.z());
  glVertex3d(cam_pos.x() + y_axis.x(), cam_pos.y() + y_axis.y(), cam_pos.z() + y_axis.z());

  // Z axis - Blue (optical axis)
  glColor3f(0.0f, 0.0f, 1.0f);
  glVertex3d(cam_pos.x(), cam_pos.y(), cam_pos.z());
  glVertex3d(cam_pos.x() + z_axis.x(), cam_pos.y() + z_axis.y(), cam_pos.z() + z_axis.z());
  glEnd();
}

} // namespace ov_core

#else // !HAVE_PANGOLIN

namespace ov_core {

PangolinViewer::PangolinViewer(const Config &config)
    : config_(config), running_(false), quit_requested_(false) {
  current_pose_ = Eigen::Matrix4d::Identity();
}

PangolinViewer::~PangolinViewer() {
  stop();
}

void PangolinViewer::start() {
  // No-op when Pangolin is not available
  running_.store(true);
}

void PangolinViewer::stop() {
  running_.store(false);
}

void PangolinViewer::updatePointCloud(const std::vector<Eigen::Vector3d> &points) {
  std::lock_guard<std::mutex> lock(data_mutex_);
  points_ = points;
}

void PangolinViewer::updateTrajectory(const std::vector<Eigen::Matrix4d> &poses) {
  std::lock_guard<std::mutex> lock(data_mutex_);
  trajectory_ = poses;
}

void PangolinViewer::updateCurrentPose(const Eigen::Matrix4d &pose) {
  std::lock_guard<std::mutex> lock(data_mutex_);
  current_pose_ = pose;
}

void PangolinViewer::updateImage(const cv::Mat &image) {
  std::lock_guard<std::mutex> lock(data_mutex_);
  current_image_ = image.clone();
}

bool PangolinViewer::shouldQuit() const {
  return quit_requested_.load();
}

void PangolinViewer::renderLoop() {
  // No-op when Pangolin is not available
}

void PangolinViewer::drawPointCloud() {
  // No-op when Pangolin is not available
}

void PangolinViewer::drawTrajectory() {
  // No-op when Pangolin is not available
}

void PangolinViewer::drawCurrentCamera() {
  // No-op when Pangolin is not available
}

void PangolinViewer::drawCameraFrustum(const Eigen::Matrix4d &pose, float size) {
  (void)pose;
  (void)size;
  // No-op when Pangolin is not available
}

} // namespace ov_core

#endif // HAVE_PANGOLIN
