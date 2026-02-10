cmake_minimum_required(VERSION 3.10)
project(vo_unity VERSION 2.0.0 LANGUAGES CXX)

# C++17 for std::optional
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# 必须为 Android 交叉编译
if(NOT ANDROID)
    message(FATAL_ERROR "此 CMakeLists 仅用于 Android SO 构建。请使用 Android NDK toolchain。")
endif()

message(STATUS "Building vo_unity (Full VIO) for Android ${ANDROID_ABI}")

# Find dependencies
find_package(Eigen3 REQUIRED)
find_package(OpenCV REQUIRED)
# Boost is header-only, manually specify path
set(BOOST_ROOT "${CMAKE_CURRENT_SOURCE_DIR}/../../../Thirdparty/boost_1_84_0")

# Path definitions
set(OV_CORE_SRC_DIR "${CMAKE_CURRENT_SOURCE_DIR}/..")
set(OV_SRVINS_SRC_DIR "${CMAKE_CURRENT_SOURCE_DIR}/../../../ov_srvins/src")

# Include directories
include_directories(
    ${OV_CORE_SRC_DIR}
    ${OV_CORE_SRC_DIR}/..
    ${OV_SRVINS_SRC_DIR}
    ${EIGEN3_INCLUDE_DIR}
    ${OpenCV_INCLUDE_DIRS}
    ${BOOST_ROOT}
)

# ============================================================================
# Collect ov_core sources (excluding test files and old visual_odometry)
# ============================================================================
file(GLOB_RECURSE OVCORE_SOURCES
    "${OV_CORE_SRC_DIR}/cam/*.cpp"
    "${OV_CORE_SRC_DIR}/cpi/*.cpp"
    "${OV_CORE_SRC_DIR}/feat/*.cpp"
    "${OV_CORE_SRC_DIR}/init/*.cpp"
    "${OV_CORE_SRC_DIR}/sim/*.cpp"
    "${OV_CORE_SRC_DIR}/track/*.cpp"
    "${OV_CORE_SRC_DIR}/types/*.cpp"
    "${OV_CORE_SRC_DIR}/utils/*.cpp"
)

# Exclude test files
list(FILTER OVCORE_SOURCES EXCLUDE REGEX ".*test_.*\\.cpp$")

# ============================================================================
# Collect ov_srvins sources (the full MSCKF VIO pipeline)
# ============================================================================
set(SRVINS_SOURCES
    ${OV_SRVINS_SRC_DIR}/core/VioManager.cpp
    ${OV_SRVINS_SRC_DIR}/core/VioManagerOptions.cpp
    ${OV_SRVINS_SRC_DIR}/state/State.cpp
    ${OV_SRVINS_SRC_DIR}/state/StateHelper.cpp
    ${OV_SRVINS_SRC_DIR}/state/Propagator.cpp
    ${OV_SRVINS_SRC_DIR}/update/UpdaterHelper.cpp
    ${OV_SRVINS_SRC_DIR}/update/UpdaterMSCKF.cpp
    ${OV_SRVINS_SRC_DIR}/update/UpdaterSLAM.cpp
    ${OV_SRVINS_SRC_DIR}/update/UpdaterZeroVelocity.cpp
    ${OV_SRVINS_SRC_DIR}/initializer/InertialInitializer.cpp
    ${OV_SRVINS_SRC_DIR}/initializer/InertialInitializerOptions.cpp
    ${OV_SRVINS_SRC_DIR}/initializer/static/StaticInitializer.cpp
    ${OV_SRVINS_SRC_DIR}/initializer/dynamic/DynamicInitializer.cpp
    ${OV_SRVINS_SRC_DIR}/initializer/dynamic/Solver.cpp
    ${OV_SRVINS_SRC_DIR}/utils/CameraPoseBuffer.cpp
    ${OV_SRVINS_SRC_DIR}/utils/EigenMatrixBuffer.cpp
    ${OV_SRVINS_SRC_DIR}/utils/Helper.cpp
    ${OV_SRVINS_SRC_DIR}/utils/NoiseManager.cpp
    ${OV_SRVINS_SRC_DIR}/utils/Timer.cpp
)

# Unity bridge sources
set(UNITY_SOURCES
    ${CMAKE_CURRENT_SOURCE_DIR}/vo_unity_api.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/VOUnityBridge.cpp
)

# Boost.Filesystem compiled sources (needed by VioManager timing stats)
set(BOOST_FS_DIR "${BOOST_ROOT}/libs/filesystem/src")
set(BOOST_FS_SOURCES
    ${BOOST_FS_DIR}/codecvt_error_category.cpp
    ${BOOST_FS_DIR}/directory.cpp
    ${BOOST_FS_DIR}/exception.cpp
    ${BOOST_FS_DIR}/operations.cpp
    ${BOOST_FS_DIR}/path.cpp
    ${BOOST_FS_DIR}/path_traits.cpp
    ${BOOST_FS_DIR}/portability.cpp
    ${BOOST_FS_DIR}/unique_path.cpp
    ${BOOST_FS_DIR}/utf8_codecvt_facet.cpp
)

# ============================================================================
# Build shared library for Unity Android
# ============================================================================
add_library(vo_unity SHARED
    ${OVCORE_SOURCES}
    ${SRVINS_SOURCES}
    ${UNITY_SOURCES}
    ${BOOST_FS_SOURCES}
)

# Compile definitions
target_compile_definitions(vo_unity PRIVATE
    VO_UNITY_EXPORTS
    ROS_AVAILABLE=0
    ENABLE_ARUCO_TAGS=0
    BOOST_ALL_NO_LIB
    BOOST_FILESYSTEM_SOURCE
)

# Android NDK settings
target_compile_options(vo_unity PRIVATE
    -O3
    -fPIC
    -fvisibility=hidden
    -fvisibility-inlines-hidden
)

# Use c++_shared STL
set_target_properties(vo_unity PROPERTIES
    ANDROID_STL c++_shared
    OUTPUT_NAME vo_unity
    VERSION ${PROJECT_VERSION}
    SOVERSION 2
    POSITION_INDEPENDENT_CODE ON
    CXX_VISIBILITY_PRESET hidden
    VISIBILITY_INLINES_HIDDEN ON
)

# Link libraries
target_link_libraries(vo_unity PRIVATE
    ${OpenCV_LIBRARIES}
    log  # Android log library
    GLESv2 # OpenGL ES 2.0
)

# Strip debug symbols in Release mode
if(CMAKE_BUILD_TYPE STREQUAL "Release")
    add_custom_command(TARGET vo_unity POST_BUILD
        COMMAND ${CMAKE_STRIP} $<TARGET_FILE:vo_unity>
        COMMENT "Stripping debug symbols from libvo_unity.so"
    )
endif()

# ============================================================================
# Print build info
# ============================================================================
message(STATUS "")
message(STATUS "=== vo_unity Full VIO Build ===")
message(STATUS "  ABI: ${ANDROID_ABI}")
message(STATUS "  Platform: ${ANDROID_PLATFORM}")
message(STATUS "  OpenCV: ${OpenCV_VERSION}")
message(STATUS "  Eigen3: ${EIGEN3_INCLUDE_DIR}")
message(STATUS "  ov_core sources: ${OV_CORE_SRC_DIR}")
message(STATUS "  ov_srvins sources: ${OV_SRVINS_SRC_DIR}")
message(STATUS "================================")
message(STATUS "")
