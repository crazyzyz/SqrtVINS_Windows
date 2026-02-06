# SqrtVINS 构建指南

本项目已移除 ROS 依赖，支持以下平台：
- **Windows** - Visual Studio 编译，生成 `.exe` 可执行文件
- **Android** - NDK 交叉编译，生成 `libvo_unity.so` 供 Unity 调用
- **如果你觉得这个项目对你有用，欢迎点个 Star ⭐ 支持一下！**

---

## 目录结构

```
sqrtVINS/
├── ov_core/                    # 核心算法库
│   └── src/
│       ├── visual_odometry/    # 视觉里程计
│       ├── track/              # 特征跟踪
│       └── unity/              # Unity 接口 (C API)
├── ov_srvins/                  # VIO 系统实现
├── Thirdparty/                 # 第三方依赖 (不提交到 Git)
│   ├── android-ndk-r25c-windows/
│   ├── opencv-4.8.0-android-sdk/
│   └── boost_1_84_0/
├── build_vs/                   # Windows 构建目录
└── build_android/              # Android 构建目录
```

---

## 一、Windows 构建

### 依赖项

- **Visual Studio 2022**
- **CMake 3.10+**
- **vcpkg**

### 安装依赖

```powershell
vcpkg install eigen3:x64-windows
vcpkg install opencv4:x64-windows
vcpkg install boost:x64-windows
```

### 构建步骤

```powershell
# 1. 创建构建目录
mkdir build_vs
cd build_vs

# 2. CMake 配置
cmake ../ov_srvins -G "Visual Studio 17 2022" -A x64 `
    -DCMAKE_TOOLCHAIN_FILE="C:/vcpkg/scripts/buildsystems/vcpkg.cmake" `
    -DENABLE_ROS=OFF `
    -DENABLE_ARUCO_TAGS=OFF `
    -DUSE_FLOAT=OFF

# 3. 编译
cmake --build . --config Release -j8
```

### 运行

```powershell
# 仿真 Demo
.\Release\run_simulation.exe ..\config\rpng_sim\estimator_config.yaml

# 摄像头测试
.\Release\test_webcam.exe
```

### 生成文件

| 文件名 | 用途 |
|--------|------|
| `run_simulation.exe` | 运行仿真 Demo，验证 VIO 算法 |
| `test_webcam.exe` | 测试 USB 摄像头的前端特征跟踪 |

---

## 二、Android 构建 (Unity SO 库)

### 依赖项

| 依赖 | 版本 | 下载地址 |
|------|------|----------|
| Android NDK | r25c | https://dl.google.com/android/repository/android-ndk-r25c-windows.zip |
| OpenCV Android | 4.8.0 | https://opencv.org/releases/ |
| Boost | 1.84.0 | https://archives.boost.org/release/1.84.0/source/boost_1_84_0.zip |
| Ninja | 最新 | `winget install Kitware.Ninja` |

### 安装依赖

将下载的依赖解压到 `Thirdparty/` 目录：

```
Thirdparty/
├── android-ndk-r25c-windows/android-ndk-r25c/
├── opencv-4.8.0-android-sdk/OpenCV-android-sdk/
└── boost_1_84_0/
```

### 构建步骤

```powershell
# 1. 创建构建目录
mkdir build_android
cd build_android

# 2. CMake 配置
cmake ../ov_core/src/unity -G "Ninja" `
    -DCMAKE_TOOLCHAIN_FILE="../Thirdparty/android-ndk-r25c-windows/android-ndk-r25c/build/cmake/android.toolchain.cmake" `
    -DANDROID_ABI=arm64-v8a `
    -DANDROID_PLATFORM=android-24 `
    -DANDROID_STL=c++_shared `
    -DOpenCV_DIR="D:/A3D/sqrtVINS/Thirdparty/opencv-4.8.0-android-sdk/OpenCV-android-sdk/sdk/native/jni" `
    -DEigen3_DIR="C:/vcpkg/installed/x64-windows/share/eigen3" `
    -DCMAKE_BUILD_TYPE=Release

# 3. 编译
cmake --build . -j8
```

### 输出文件

```
build_android/libvo_unity.so    # 约 6MB
```

### 部署到 Unity

将以下文件复制到 Unity 项目：

```
Assets/Plugins/Android/libs/arm64-v8a/
├── libvo_unity.so              # 构建生成
└── libopencv_java4.so          # 从 OpenCV SDK 复制
```

OpenCV 依赖位置：
```
Thirdparty/opencv-4.8.0-android-sdk/OpenCV-android-sdk/sdk/native/libs/arm64-v8a/libopencv_java4.so
```

### Unity C# 调用示例

```csharp
using System.Runtime.InteropServices;

public class VONative
{
    private const string DLL_NAME = "vo_unity";
    
    [DllImport(DLL_NAME)] 
    public static extern int vo_initialize(ref VOCameraParams camera, System.IntPtr tracking);
    
    [DllImport(DLL_NAME)] 
    public static extern int vo_shutdown();
    
    [DllImport(DLL_NAME)] 
    public static extern int vo_process_frame(byte[] data, int w, int h, int ch, double ts);
    
    [DllImport(DLL_NAME)] 
    public static extern int vo_get_pose(ref VOPose pose);
    
    [StructLayout(LayoutKind.Sequential)]
    public struct VOCameraParams 
    { 
        public float fx, fy, cx, cy; 
        public int width, height; 
        public float k1, k2, p1, p2; 
    }
    
    [StructLayout(LayoutKind.Sequential)]
    public struct VOPose 
    { 
        public float px, py, pz, qx, qy, qz, qw; 
        public int valid; 
    }
}
```

---


