/**
 * @file vo_unity_api.h
 * @brief C API for Unity Visual Odometry Library
 *
 * This header defines the C-style API for the Unity VO library,
 * enabling P/Invoke calls from Unity C# scripts.
 */

#ifndef VO_UNITY_API_H
#define VO_UNITY_API_H

#ifdef __cplusplus
extern "C" {
#endif

// Platform-specific export macros
#ifdef _WIN32
#ifdef VO_UNITY_EXPORTS
#define VO_API __declspec(dllexport)
#else
#define VO_API __declspec(dllimport)
#endif
#else
#define VO_API __attribute__((visibility("default")))
#endif

/**
 * @brief Error codes returned by VO library functions
 */
typedef enum {
  VO_SUCCESS = 0,                    ///< Operation completed successfully
  VO_ERROR_NOT_INITIALIZED = -1,     ///< Library not initialized
  VO_ERROR_ALREADY_INITIALIZED = -2, ///< Library already initialized
  VO_ERROR_INVALID_PARAM = -3,       ///< Invalid parameter provided
  VO_ERROR_INVALID_IMAGE = -4,       ///< Invalid image data or dimensions
  VO_ERROR_TRACKING_FAILED = -5,     ///< Feature tracking failed
  VO_ERROR_POSE_FAILED = -6          ///< Pose estimation failed
} VOErrorCode;

/**
 * @brief Camera intrinsic parameters
 */
typedef struct {
  float fx;   ///< Focal length X (pixels)
  float fy;   ///< Focal length Y (pixels)
  float cx;   ///< Principal point X (pixels)
  float cy;   ///< Principal point Y (pixels)
  int width;  ///< Image width (pixels)
  int height; ///< Image height (pixels)
  float k1;   ///< Radial distortion coefficient 1
  float k2;   ///< Radial distortion coefficient 2
  float p1;   ///< Tangential distortion coefficient 1
  float p2;   ///< Tangential distortion coefficient 2
} VOCameraParams;

/**
 * @brief Feature tracking parameters
 */
typedef struct {
  int max_features;   ///< Maximum number of features to track (default: 500)
  int fast_threshold; ///< FAST detector threshold (default: 20)
  int grid_x;         ///< Grid columns for feature distribution (default: 5)
  int grid_y;         ///< Grid rows for feature distribution (default: 4)
  int min_px_dist;    ///< Minimum pixel distance between features (default: 20)
} VOTrackingParams;

/**
 * @brief Tracked feature point data
 */
typedef struct {
  int id;     ///< Unique feature ID
  float x;    ///< X position in pixels
  float y;    ///< Y position in pixels
  int status; ///< Status: 0=lost, 1=tracked, 2=new
} VOFeature;

/**
 * @brief Camera pose in Unity coordinate system (left-handed, Y-up)
 */
typedef struct {
  float px;  ///< Position X
  float py;  ///< Position Y
  float pz;  ///< Position Z
  float qx;  ///< Quaternion X
  float qy;  ///< Quaternion Y
  float qz;  ///< Quaternion Z
  float qw;  ///< Quaternion W
  int valid; ///< 1 if pose is valid, 0 otherwise
} VOPose;

/**
 * @brief IMU measurement data
 */
typedef struct {
  double timestamp;  ///< Timestamp in seconds
  float ax;          ///< Accelerometer X (m/s^2)
  float ay;          ///< Accelerometer Y (m/s^2)
  float az;          ///< Accelerometer Z (m/s^2)
  float gx;          ///< Gyroscope X (rad/s)
  float gy;          ///< Gyroscope Y (rad/s)
  float gz;          ///< Gyroscope Z (rad/s)
} VOImuData;

// ============================================================================
// Initialization and Shutdown
// ============================================================================

/**
 * @brief Initialize the VO library with camera and tracking parameters
 * @param camera_params Camera intrinsic parameters
 * @param tracking_params Feature tracking parameters (can be NULL for defaults)
 * @return VO_SUCCESS on success, error code otherwise
 */
VO_API VOErrorCode vo_initialize(const VOCameraParams *camera_params,
                                 const VOTrackingParams *tracking_params);

/**
 * @brief Shutdown the VO library and release all resources
 * @return VO_SUCCESS on success, error code otherwise
 */
VO_API VOErrorCode vo_shutdown(void);

/**
 * @brief Check if the library is initialized
 * @return 1 if initialized, 0 otherwise
 */
VO_API int vo_is_initialized(void);

// ============================================================================
// Frame Processing
// ============================================================================

/**
 * @brief Process a new camera frame
 * @param image_data Pointer to image pixel data
 * @param width Image width in pixels
 * @param height Image height in pixels
 * @param channels Number of channels (1=Grayscale, 4=RGBA)
 * @param timestamp Frame timestamp in seconds
 * @return VO_SUCCESS on success, error code otherwise
 */
VO_API VOErrorCode vo_process_frame(const unsigned char *image_data, int width,
                                    int height, int channels, double timestamp);

// ============================================================================
// Data Retrieval
// ============================================================================

/**
 * @brief Get the number of currently tracked features
 * @return Number of features, or -1 if not initialized
 */
VO_API int vo_get_feature_count(void);

/**
 * @brief Get tracked feature data
 * @param features Output array for feature data
 * @param max_count Maximum number of features to retrieve
 * @param out_count Output: actual number of features retrieved
 * @return VO_SUCCESS on success, error code otherwise
 */
VO_API VOErrorCode vo_get_features(VOFeature *features, int max_count,
                                   int *out_count);

/**
 * @brief Get the current camera pose
 * @param pose Output pose structure
 * @return VO_SUCCESS on success, error code otherwise
 */
VO_API VOErrorCode vo_get_pose(VOPose *pose);

/**
 * @brief Get the 3D point cloud
 * @param points Output array for 3D points (x,y,z triplets)
 * @param max_points Maximum number of points to retrieve
 * @param out_count Output: actual number of points retrieved
 * @return VO_SUCCESS on success, error code otherwise
 */
VO_API VOErrorCode vo_get_point_cloud(float *points, int max_points,
                                      int *out_count);

/**
 * @brief Get the debug visualization image with features and optical flow drawn
 * @param output_image Output buffer for RGBA image data (must be width*height*4
 * bytes)
 * @param width Image width (must match initialized camera width)
 * @param height Image height (must match initialized camera height)
 * @param draw_points If non-zero, draw feature points (red circles)
 * @param draw_flow If non-zero, draw optical flow lines (green lines)
 * @return VO_SUCCESS on success, error code otherwise
 */
VO_API VOErrorCode vo_get_debug_image(unsigned char *output_image, int width,
                                      int height, int draw_points,
                                      int draw_flow);

// ============================================================================
// IMU Data Input
// ============================================================================

/**
 * @brief Feed IMU measurement to the VIO system
 * @param imu_data IMU measurement data
 * @return VO_SUCCESS on success, error code otherwise
 */
VO_API VOErrorCode vo_feed_imu(const VOImuData *imu_data);

/**
 * @brief Reset IMU integration state
 * @return VO_SUCCESS on success, error code otherwise
 */
VO_API VOErrorCode vo_reset_imu(void);

// ============================================================================
// Runtime Configuration
// ============================================================================

/**
 * @brief Set the maximum number of features to track
 * @param count Maximum feature count (must be > 0)
 * @return VO_SUCCESS on success, error code otherwise
 */
VO_API VOErrorCode vo_set_max_features(int count);

/**
 * @brief Set the FAST detector threshold
 * @param threshold FAST threshold value (must be > 0)
 * @return VO_SUCCESS on success, error code otherwise
 */
VO_API VOErrorCode vo_set_fast_threshold(int threshold);

// ============================================================================
// Utility Functions
// ============================================================================

/**
 * @brief Get a human-readable error string
 * @param code Error code
 * @return Static string describing the error
 */
VO_API const char *vo_get_error_string(VOErrorCode code);

/**
 * @brief Get the library version
 * @param major Output: major version number
 * @param minor Output: minor version number
 * @param patch Output: patch version number
 */
VO_API void vo_get_version(int *major, int *minor, int *patch);

// ============================================================================
// Native Rendering Plugin Interface
// ============================================================================

/**
 * @brief Function pointer type for Unity rendering events
 */
typedef void (*UnityRenderEventCallback)(int);

/**
 * @brief Get the specific rendering event callback for Unity
 * @return Function pointer to be passed to GL.IssuePluginEvent
 */
VO_API UnityRenderEventCallback vo_get_render_event_func(void);

/**
 * @brief Set the native texture pointer (e.g., OpenGL ID)
 * @param texture_ptr Pointer to the native texture (cast from intptr_t)
 * @param width Texture width
 * @param height Texture height
 * @return VO_SUCCESS on success
 */
VO_API VOErrorCode vo_set_native_texture(void *texture_ptr, int width,
                                         int height);

#ifdef __cplusplus
}
#endif

#endif // VO_UNITY_API_H
