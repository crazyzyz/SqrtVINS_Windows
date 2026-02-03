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
    float fx;           ///< Focal length X (pixels)
    float fy;           ///< Focal length Y (pixels)
    float cx;           ///< Principal point X (pixels)
    float cy;           ///< Principal point Y (pixels)
    int width;          ///< Image width (pixels)
    int height;         ///< Image height (pixels)
    float k1;           ///< Radial distortion coefficient 1
    float k2;           ///< Radial distortion coefficient 2
    float p1;           ///< Tangential distortion coefficient 1
    float p2;           ///< Tangential distortion coefficient 2
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
    int id;             ///< Unique feature ID
    float x;            ///< X position in pixels
    float y;            ///< Y position in pixels
    int status;         ///< Status: 0=lost, 1=tracked, 2=new
} VOFeature;

/**
 * @brief Camera pose in Unity coordinate system (left-handed, Y-up)
 */
typedef struct {
    float px;           ///< Position X
    float py;           ///< Position Y
    float pz;           ///< Position Z
    float qx;           ///< Quaternion X
    float qy;           ///< Quaternion Y
    float qz;           ///< Quaternion Z
    float qw;           ///< Quaternion W
    int valid;          ///< 1 if pose is valid, 0 otherwise
} VOPose;

// ============================================================================
// Initialization and Shutdown
// ============================================================================

/**
 * @brief Initialize the VO library with camera and tracking parameters
 * @param camera_params Camera intrinsic parameters
 * @param tracking_params Feature tracking parameters (can be NULL for defaults)
 * @return VO_SUCCESS on success, error code otherwise
 */
VO_API VOErrorCode vo_initialize(const VOCameraParams* camera_params,
                                  const VOTrackingParams* tracking_params);

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
VO_API VOErrorCode vo_process_frame(const unsigned char* image_data,
                                     int width, int height,
                                     int channels,
                                     double timestamp);

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
VO_API VOErrorCode vo_get_features(VOFeature* features, int max_count, int* out_count);

/**
 * @brief Get the current camera pose
 * @param pose Output pose structure
 * @return VO_SUCCESS on success, error code otherwise
 */
VO_API VOErrorCode vo_get_pose(VOPose* pose);

/**
 * @brief Get the 3D point cloud
 * @param points Output array for 3D points (x,y,z triplets)
 * @param max_points Maximum number of points to retrieve
 * @param out_count Output: actual number of points retrieved
 * @return VO_SUCCESS on success, error code otherwise
 */
VO_API VOErrorCode vo_get_point_cloud(float* points, int max_points, int* out_count);

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
VO_API const char* vo_get_error_string(VOErrorCode code);

/**
 * @brief Get the library version
 * @param major Output: major version number
 * @param minor Output: minor version number
 * @param patch Output: patch version number
 */
VO_API void vo_get_version(int* major, int* minor, int* patch);

#ifdef __cplusplus
}
#endif

#endif // VO_UNITY_API_H
