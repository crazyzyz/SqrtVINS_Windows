cmake_minimum_required(VERSION 3.3)
project(ov_srvins)

# Include libraries (if we don't have opencv 4, then fallback to opencv 3)
find_package(Eigen3 REQUIRED)
find_package(OpenCV 3 QUIET)
if(NOT OpenCV_FOUND)
    find_package(OpenCV 4 REQUIRED)
endif()
find_package(Boost REQUIRED COMPONENTS system filesystem thread date_time)
message(STATUS "OPENCV: " ${OpenCV_VERSION} " | BOOST: " ${Boost_VERSION})

# If we will compile with aruco support
option(ENABLE_ARUCO_TAGS "Enable or disable aruco tag (disable if no contrib modules)" ON)
if(NOT ENABLE_ARUCO_TAGS)
    add_definitions(-DENABLE_ARUCO_TAGS=0)
    message(WARNING "DISABLING ARUCOTAG TRACKING!")
else()
    add_definitions(-DENABLE_ARUCO_TAGS=1)
endif()

# We need c++17 for std::optional
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# Default build as release
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE "Release" CACHE STRING "Choose the type of build." FORCE)
endif()

# Enable compile optimizations
if(MSVC)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} /O2 /W3")
    set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} /O2")
    set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} /Zi /Od")
    add_definitions(-D_USE_MATH_DEFINES -DNOMINMAX)
else()
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3 -fsee -fomit-frame-pointer -fno-signed-zeros -fno-math-errno -funroll-loops")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -g3 -Wall -Wuninitialized -fno-omit-frame-pointer")
endif()

# No ROS available
add_definitions(-DROS_AVAILABLE=0)
message(STATUS "Building without ROS support")

# Installation paths
include(GNUInstallDirs)
set(CATKIN_PACKAGE_LIB_DESTINATION "${CMAKE_INSTALL_LIBDIR}")
set(CATKIN_PACKAGE_BIN_DESTINATION "${CMAKE_INSTALL_BINDIR}")
set(CATKIN_GLOBAL_INCLUDE_DESTINATION "${CMAKE_INSTALL_INCLUDEDIR}")

option(USE_FLOAT "Use float version when built" ON)
if (USE_FLOAT)
    add_definitions(-DUSE_FLOAT=1)
endif ()

# Include our header files
include_directories(
        src
        ${EIGEN3_INCLUDE_DIR}
        ${Boost_INCLUDE_DIRS}
)

# Set link libraries used by all binaries
list(APPEND thirdparty_libraries
        ${Boost_LIBRARIES}
        ${OpenCV_LIBRARIES}
)

# Manually link to ov_core sources (no ROS catkin)
message(STATUS "MANUALLY LINKING TO OV_CORE LIBRARY....")
include_directories(${CMAKE_SOURCE_DIR}/../ov_core/src/)
file(GLOB_RECURSE OVCORE_LIBRARY_SOURCES "${CMAKE_SOURCE_DIR}/../ov_core/src/*.cpp")
list(FILTER OVCORE_LIBRARY_SOURCES EXCLUDE REGEX ".*test_webcam\\.cpp$")
list(FILTER OVCORE_LIBRARY_SOURCES EXCLUDE REGEX ".*test_tracking\\.cpp$")
list(FILTER OVCORE_LIBRARY_SOURCES EXCLUDE REGEX ".*test_profile\\.cpp$")
list(FILTER OVCORE_LIBRARY_SOURCES EXCLUDE REGEX ".*test_visual_mapping\\.cpp$")
list(APPEND LIBRARY_SOURCES ${OVCORE_LIBRARY_SOURCES})
file(GLOB_RECURSE OVCORE_LIBRARY_HEADERS "${CMAKE_SOURCE_DIR}/../ov_core/src/*.h")
list(APPEND LIBRARY_HEADERS ${OVCORE_LIBRARY_HEADERS})

# #################################################
# Make the shared library
# #################################################
list(APPEND LIBRARY_SOURCES
        src/dummy.cpp
        src/sim/Simulator.cpp
        src/state/State.cpp
        src/state/StateHelper.cpp
        src/state/Propagator.cpp
        src/core/VioManager.cpp
        src/core/VioManagerOptions.cpp
        src/update/UpdaterHelper.cpp
        src/update/UpdaterMSCKF.cpp
        src/update/UpdaterSLAM.cpp
        src/update/UpdaterZeroVelocity.cpp
        src/initializer/InertialInitializer.cpp
        src/initializer/InertialInitializerOptions.cpp
        src/initializer/dynamic/Solver.cpp
        src/initializer/dynamic/OpengvHelper.cpp
        src/initializer/dynamic/DynamicInitializer.cpp
        src/initializer/static/StaticInitializer.cpp
        src/utils/Timer.cpp
        src/utils/Helper.cpp
        src/utils/NoiseManager.cpp
        src/utils/CameraPoseBuffer.cpp
        src/utils/EigenMatrixBuffer.cpp
)

file(GLOB_RECURSE LIBRARY_HEADERS "src/*.h")

# Use STATIC library on Windows to avoid DLL export issues
if(WIN32)
    add_library(ov_srvins_lib STATIC ${LIBRARY_SOURCES} ${LIBRARY_HEADERS})
else()
    add_library(ov_srvins_lib SHARED ${LIBRARY_SOURCES} ${LIBRARY_HEADERS})
endif()
target_link_libraries(ov_srvins_lib ${thirdparty_libraries})
target_include_directories(ov_srvins_lib PUBLIC src/)

install(TARGETS ov_srvins_lib
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
install(DIRECTORY src/
        DESTINATION ${CATKIN_GLOBAL_INCLUDE_DESTINATION}
        FILES_MATCHING PATTERN "*.h" PATTERN "*.hpp"
)

# #################################################
# Make binary files!
# #################################################

add_executable(run_simulation src/run_simulation.cpp)
target_link_libraries(run_simulation ov_srvins_lib ${thirdparty_libraries})
install(TARGETS run_simulation
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

add_executable(test_sim_meas src/test_sim_meas.cpp)
target_link_libraries(test_sim_meas ov_srvins_lib ${thirdparty_libraries})
install(TARGETS test_sim_meas
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

add_executable(test_sim_repeat src/test_sim_repeat.cpp)
target_link_libraries(test_sim_repeat ov_srvins_lib ${thirdparty_libraries})
install(TARGETS test_sim_repeat
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

add_executable(test_webcam ${CMAKE_SOURCE_DIR}/../ov_core/src/test_webcam.cpp)
target_link_libraries(test_webcam ov_srvins_lib ${thirdparty_libraries})
install(TARGETS test_webcam
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
