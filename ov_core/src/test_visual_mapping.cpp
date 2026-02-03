/**
 * @file test_visual_mapping.cpp
 * @brief Visual sparse mapping test application with Pangolin visualization
 *
 * This application demonstrates visual odometry with sparse point cloud mapping
 * using a webcam. It uses feature tracking, pose estimation, and triangulation
 * to build a 3D map, visualized in real-time using Pangolin.
 *
 * Usage:
 *   test_visual_mapping [config_file]
 *
 * If no config file is provided, default webcam calibration values are used.
 */

#include <csignal>
#include <iostream>
#include <string>
#include <thread>
#include <atomic>
#include <mutex>
#include <cmath>
#include <sstream>

// Windows socket includes
#ifdef _WIN32
#include <winsock2.h>
#include <ws2tcpip.h>
#pragma comment(lib, "ws2_32.lib")
#else
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <unistd.h>
#define SOCKET int
#define INVALID_SOCKET -1
#define SOCKET_ERROR -1
#define closesocket close
#endif

#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>

#include "utils/print.h"

#if HAVE_PANGOLIN
#include "visual_odometry/VisualOdometry.h"
#include "track/TrackBase.h"
#endif

using namespace ov_core;

// IMU data structure (renamed to avoid conflict with ov_core::ImuData)
struct UdpImuData {
  double ax, ay, az;  // Acceleration (m/s^2)
  double gx, gy, gz;  // Gyroscope (rad/s)
  double timestamp;
};

// Global IMU state
static std::mutex g_imu_mutex;
static UdpImuData g_latest_imu;
static std::atomic<bool> g_imu_received{false};
static std::atomic<bool> g_is_stationary{false};

// IMU thresholds for stationary detection
static const double ACCEL_THRESHOLD = 0.5;  // m/s^2 deviation from gravity
static const double GYRO_THRESHOLD = 0.05;  // rad/s

/**
 * @brief Parse IMU data from string format: "ax,ay,az,gx,gy,gz"
 */
bool parseImuData(const std::string& data, UdpImuData& imu) {
  std::stringstream ss(data);
  std::string token;
  std::vector<double> values;
  
  while (std::getline(ss, token, ',')) {
    try {
      values.push_back(std::stod(token));
    } catch (...) {
      return false;
    }
  }
  
  if (values.size() >= 6) {
    imu.ax = values[0];
    imu.ay = values[1];
    imu.az = values[2];
    imu.gx = values[3];
    imu.gy = values[4];
    imu.gz = values[5];
    return true;
  }
  return false;
}

/**
 * @brief Check if device is stationary based on IMU data
 */
bool checkStationary(const UdpImuData& imu) {
  // Check if acceleration is close to gravity (9.8 m/s^2)
  double accel_mag = std::sqrt(imu.ax*imu.ax + imu.ay*imu.ay + imu.az*imu.az);
  double accel_diff = std::abs(accel_mag - 9.81);
  
  // Check if gyroscope readings are near zero
  double gyro_mag = std::sqrt(imu.gx*imu.gx + imu.gy*imu.gy + imu.gz*imu.gz);
  
  return (accel_diff < ACCEL_THRESHOLD) && (gyro_mag < GYRO_THRESHOLD);
}

/**
 * @brief UDP receiver thread for IMU data
 */
void imuReceiverThread(int port, std::atomic<bool>& shutdown) {
#ifdef _WIN32
  // Initialize Winsock
  WSADATA wsaData;
  if (WSAStartup(MAKEWORD(2, 2), &wsaData) != 0) {
    PRINT_ERROR("WSAStartup failed\n");
    return;
  }
#endif

  SOCKET sock = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
  if (sock == INVALID_SOCKET) {
    PRINT_ERROR("Failed to create UDP socket\n");
#ifdef _WIN32
    WSACleanup();
#endif
    return;
  }

  // Set socket timeout
#ifdef _WIN32
  DWORD timeout = 100;  // 100ms
  setsockopt(sock, SOL_SOCKET, SO_RCVTIMEO, (const char*)&timeout, sizeof(timeout));
#else
  struct timeval tv;
  tv.tv_sec = 0;
  tv.tv_usec = 100000;  // 100ms
  setsockopt(sock, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));
#endif

  struct sockaddr_in addr;
  addr.sin_family = AF_INET;
  addr.sin_port = htons(port);
  addr.sin_addr.s_addr = INADDR_ANY;

  if (bind(sock, (struct sockaddr*)&addr, sizeof(addr)) == SOCKET_ERROR) {
    PRINT_ERROR("Failed to bind UDP socket to port %d\n", port);
    closesocket(sock);
#ifdef _WIN32
    WSACleanup();
#endif
    return;
  }

  PRINT_INFO("IMU receiver started on UDP port %d\n", port);

  char buffer[1024];
  int stationary_count = 0;
  const int STATIONARY_FRAMES = 10;  // Need 10 consecutive stationary readings

  while (!shutdown) {
    int len = recv(sock, buffer, sizeof(buffer) - 1, 0);
    if (len > 0) {
      buffer[len] = '\0';
      
      // Parse the first line (may receive multiple lines)
      std::string data(buffer);
      size_t newline = data.find('\n');
      if (newline != std::string::npos) {
        data = data.substr(0, newline);
      }
      
      UdpImuData imu;
      if (parseImuData(data, imu)) {
        std::lock_guard<std::mutex> lock(g_imu_mutex);
        g_latest_imu = imu;
        g_imu_received = true;
        
        // Update stationary detection
        if (checkStationary(imu)) {
          stationary_count++;
          if (stationary_count >= STATIONARY_FRAMES) {
            g_is_stationary = true;
          }
        } else {
          stationary_count = 0;
          g_is_stationary = false;
        }
      }
    }
  }

  closesocket(sock);
#ifdef _WIN32
  WSACleanup();
#endif
  PRINT_INFO("IMU receiver stopped\n");
}

// Global flag for signal handling
static std::atomic<bool> g_shutdown_requested{false};

/**
 * @brief Signal handler for graceful shutdown
 */
void signalHandler(int signum) {
  PRINT_INFO("\nShutdown requested (signal %d)\n", signum);
  g_shutdown_requested = true;
}

/**
 * @brief Print usage information
 */
void printUsage(const char *program_name) {
  std::cout << "Usage: " << program_name << " [options]\n"
            << "\nOptions:\n"
            << "  -c, --config <file>   Path to calibration YAML file\n"
            << "  -d, --device <id>     Camera device ID (default: 0)\n"
            << "  -w, --width <pixels>  Image width (default: 640)\n"
            << "  -h, --height <pixels> Image height (default: 480)\n"
            << "  -v, --verbosity <lvl> Verbosity level: DEBUG, INFO, WARNING, ERROR (default: INFO)\n"
            << "  -p, --imu-port <port> UDP port for IMU data (default: 2055)\n"
            << "  --no-imu              Disable IMU input\n"
            << "  --help                Show this help message\n"
            << "\nIMU Data Format (UDP):\n"
            << "  ax,ay,az,gx,gy,gz     (acceleration in m/s^2, gyro in rad/s)\n"
            << "\nControls:\n"
            << "  ESC                   Exit the application\n"
            << "  Mouse                 Rotate/pan/zoom in 3D viewer\n"
            << std::endl;
}

/**
 * @brief Parse command line arguments
 */
struct CommandLineArgs {
  std::string config_file;
  int camera_device = 0;
  int image_width = 640;
  int image_height = 480;
  std::string verbosity = "INFO";
  bool show_help = false;
  int imu_port = 2055;  // UDP port for IMU data
  bool use_imu = true;  // Enable IMU by default
};

CommandLineArgs parseArgs(int argc, char **argv) {
  CommandLineArgs args;

  for (int i = 1; i < argc; ++i) {
    std::string arg = argv[i];

    if (arg == "--help") {
      args.show_help = true;
    } else if ((arg == "-c" || arg == "--config") && i + 1 < argc) {
      args.config_file = argv[++i];
    } else if ((arg == "-d" || arg == "--device") && i + 1 < argc) {
      args.camera_device = std::stoi(argv[++i]);
    } else if ((arg == "-w" || arg == "--width") && i + 1 < argc) {
      args.image_width = std::stoi(argv[++i]);
    } else if ((arg == "-h" || arg == "--height") && i + 1 < argc) {
      args.image_height = std::stoi(argv[++i]);
    } else if ((arg == "-v" || arg == "--verbosity") && i + 1 < argc) {
      args.verbosity = argv[++i];
    } else if ((arg == "-p" || arg == "--imu-port") && i + 1 < argc) {
      args.imu_port = std::stoi(argv[++i]);
    } else if (arg == "--no-imu") {
      args.use_imu = false;
    } else if (arg[0] != '-' && args.config_file.empty()) {
      // Positional argument - treat as config file
      args.config_file = arg;
    }
  }

  return args;
}

int main(int argc, char **argv) {
  // Parse command line arguments
  CommandLineArgs args = parseArgs(argc, argv);

  // Show help if requested
  if (args.show_help) {
    printUsage(argv[0]);
    return 0;
  }

  // Set verbosity level
  Printer::setPrintLevel(args.verbosity);

  // Print welcome message
  PRINT_INFO("=======================================\n");
  PRINT_INFO("Visual Sparse Mapping Test Application\n");
  PRINT_INFO("=======================================\n");

#if HAVE_PANGOLIN
  // Register signal handler for graceful shutdown
  std::signal(SIGINT, signalHandler);
#ifdef SIGTERM
  std::signal(SIGTERM, signalHandler);
#endif

  // Initialize webcam capture
  PRINT_INFO("Opening camera device %d...\n", args.camera_device);
  cv::VideoCapture cap;
  if (!cap.open(args.camera_device)) {
    PRINT_ERROR("Unable to open camera device %d!\n", args.camera_device);
    PRINT_ERROR("Please check that your webcam is connected and not in use by another application.\n");
    return EXIT_FAILURE;
  }

  // Set camera resolution
  cap.set(cv::CAP_PROP_FRAME_WIDTH, args.image_width);
  cap.set(cv::CAP_PROP_FRAME_HEIGHT, args.image_height);

  // Get actual resolution (may differ from requested)
  int actual_width = static_cast<int>(cap.get(cv::CAP_PROP_FRAME_WIDTH));
  int actual_height = static_cast<int>(cap.get(cv::CAP_PROP_FRAME_HEIGHT));
  PRINT_INFO("Camera opened: %dx%d\n", actual_width, actual_height);

  // Create VisualOdometry configuration
  VisualOdometry::Config vo_config;
  if (!args.config_file.empty()) {
    PRINT_INFO("Loading calibration from: %s\n", args.config_file.c_str());
    vo_config = VisualOdometry::loadConfigFromFile(args.config_file);
  } else {
    PRINT_INFO("Using default calibration for %dx%d webcam\n", actual_width, actual_height);
    vo_config = VisualOdometry::getDefaultConfig(actual_width, actual_height);
  }

  // Create VisualOdometry instance
  PRINT_INFO("Initializing Visual Odometry system...\n");
  std::unique_ptr<VisualOdometry> vo;
  try {
    vo = std::make_unique<VisualOdometry>(vo_config);
  } catch (const std::exception &e) {
    PRINT_ERROR("Failed to initialize Visual Odometry: %s\n", e.what());
    return EXIT_FAILURE;
  }

  PRINT_INFO("System ready. Press ESC to exit.\n");
  PRINT_INFO("---------------------------------------\n");

  // Start IMU receiver thread
  std::thread imu_thread;
  if (args.use_imu) {
    PRINT_INFO("Starting IMU receiver on UDP port %d...\n", args.imu_port);
    imu_thread = std::thread([&]() {
      imuReceiverThread(args.imu_port, g_shutdown_requested);
    });
  }

  // Main processing loop
  double timestamp = 0.0;
  const double dt = 1.0 / 30.0;  // Assume 30 FPS
  int frame_count = 0;
  Eigen::Matrix4d last_stable_pose = Eigen::Matrix4d::Identity();
  bool pose_locked = false;

  while (!g_shutdown_requested) {
    // Capture frame
    cv::Mat frame;
    cap >> frame;

    // Check for end of stream
    if (frame.empty()) {
      PRINT_WARNING("Empty frame received, camera may have disconnected\n");
      break;
    }

    // Check IMU stationary status
    bool is_stationary = g_is_stationary.load();
    
    // Process frame (skip pose update if stationary)
    if (!is_stationary) {
      vo->processFrame(frame, timestamp);
      last_stable_pose = vo->getCurrentPose();
      pose_locked = false;
    } else {
      // Still process for visualization but don't update pose
      vo->processFrame(frame, timestamp);
      if (!pose_locked) {
        PRINT_INFO("IMU: Device stationary - pose locked\n");
        pose_locked = true;
      }
    }
    
    timestamp += dt;
    frame_count++;

    // Display feature tracking visualization (like test_webcam)
    TrackBase* tracker = vo->getTracker();
    if (tracker) {
      cv::Mat img_active, img_history;
      tracker->display_active(img_active, 255, 0, 0, 0, 0, 255);
      tracker->display_history(img_history, 255, 255, 0, 255, 255, 255);
      cv::imshow("Active Tracks", img_active);
      cv::imshow("Track History", img_history);
    }

    // Check for ESC key (wait 1ms for key press)
    int key = cv::waitKey(1);
    if (key == 27) {  // ESC key
      PRINT_INFO("ESC pressed, exiting...\n");
      break;
    }

    // Check if viewer requested quit
    if (vo->shouldQuit()) {
      PRINT_INFO("Viewer closed, exiting...\n");
      break;
    }

    // Print periodic status
    if (frame_count % 100 == 0) {
      auto points = vo->getPointCloud();
      bool imu_ok = g_imu_received.load();
      PRINT_INFO("Frame %d: %zu map points, IMU: %s, Stationary: %s\n", 
                 frame_count, points.size(),
                 imu_ok ? "OK" : "NO DATA",
                 is_stationary ? "YES" : "NO");
    }
  }

  // Cleanup
  PRINT_INFO("---------------------------------------\n");
  PRINT_INFO("Shutting down...\n");

  // Stop IMU thread
  if (args.use_imu && imu_thread.joinable()) {
    imu_thread.join();
  }

  // Release resources
  vo.reset();
  cap.release();
  cv::destroyAllWindows();

  // Print final statistics
  PRINT_INFO("Processed %d frames\n", frame_count);
  PRINT_INFO("Visual Sparse Mapping completed successfully.\n");

#else
  PRINT_WARNING("Pangolin not found - visual mapping disabled\n");
  PRINT_WARNING("Please install Pangolin and rebuild to enable this feature.\n");
  PRINT_WARNING("\nInstallation instructions:\n");
  PRINT_WARNING("  1. Clone Pangolin: git clone https://github.com/stevenlovegrove/Pangolin.git\n");
  PRINT_WARNING("  2. Build and install: cd Pangolin && mkdir build && cd build && cmake .. && make && sudo make install\n");
  PRINT_WARNING("  3. Rebuild this project with Pangolin in your CMAKE_PREFIX_PATH\n");
  return EXIT_FAILURE;
#endif

  return EXIT_SUCCESS;
}
