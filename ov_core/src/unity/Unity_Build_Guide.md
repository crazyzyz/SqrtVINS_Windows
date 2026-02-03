# Unity Android SO 库构建指南 (Windows)

本指南说明如何在 **Windows** 系统上为 Unity Android 构建 `libvo_unity.so`。

## 前提条件

1. **Android NDK** (r21 或更高版本)
   - 下载: https://developer.android.com/ndk/downloads
   - 解压到例如 `C:\Android\android-ndk-r25c`

2. **OpenCV for Android**
   - 下载: https://opencv.org/releases/
   - 解压到例如 `C:\Android\OpenCV-android-sdk`

3. **CMake 3.10+** (已安装)

---

## 构建步骤 (PowerShell)

```powershell
# 1. 设置环境变量
$env:ANDROID_NDK = "C:\Android\android-ndk-r25c"
$env:OPENCV_ANDROID = "C:\Android\OpenCV-android-sdk"

# 2. 创建构建目录
cd D:\A3D\sqrtVINS
mkdir build_android
cd build_android

# 3. 配置 CMake (ARM64-v8a)
cmake ../ov_core/src/unity `
    -DCMAKE_TOOLCHAIN_FILE="$env:ANDROID_NDK/build/cmake/android.toolchain.cmake" `
    -DANDROID_ABI=arm64-v8a `
    -DANDROID_PLATFORM=android-24 `
    -DANDROID_STL=c++_shared `
    -DOpenCV_DIR="$env:OPENCV_ANDROID/sdk/native/jni" `
    -DCMAKE_BUILD_TYPE=Release `
    -G "Ninja"

# 4. 编译
cmake --build . -j8

# 5. 输出文件: libvo_unity.so
```

> **注意**: 需要安装 Ninja 构建工具，或者改用 `"Unix Makefiles"` 生成器。
> 如果没有 Ninja，可以通过 `winget install Kitware.Ninja` 安装。

---

## 部署到 Unity

将生成的文件复制到 Unity 项目：

```
Assets/
└── Plugins/
    └── Android/
        └── libs/
            └── arm64-v8a/
                ├── libvo_unity.so              # 你构建的库
                └── libopencv_java4.so          # 从 OpenCV SDK 复制
```

OpenCV 依赖文件位置：
```
C:\Android\OpenCV-android-sdk\sdk\native\libs\arm64-v8a\libopencv_java4.so
```

---

## Unity C# 调用示例

```csharp
using System.Runtime.InteropServices;

public class VONative
{
    private const string DLL_NAME = "vo_unity";
    
    [DllImport(DLL_NAME)] public static extern int vo_initialize(ref VOCameraParams camera, System.IntPtr tracking);
    [DllImport(DLL_NAME)] public static extern int vo_shutdown();
    [DllImport(DLL_NAME)] public static extern int vo_process_frame(byte[] data, int w, int h, int ch, double ts);
    [DllImport(DLL_NAME)] public static extern int vo_get_pose(ref VOPose pose);
    [DllImport(DLL_NAME)] public static extern int vo_get_feature_count();
    
    [StructLayout(LayoutKind.Sequential)]
    public struct VOCameraParams { public float fx, fy, cx, cy; public int width, height; public float k1, k2, p1, p2; }
    
    [StructLayout(LayoutKind.Sequential)]
    public struct VOPose { public float px, py, pz, qx, qy, qz, qw; public int valid; }
}
```

---

## 常见问题

### Q: CMake 报错 "Could not find toolchain file"
A: 确认 `ANDROID_NDK` 路径正确，且使用正斜杠 `/` 或转义反斜杠。

### Q: 找不到 OpenCV
A: 确保 `OpenCV_DIR` 指向 `sdk/native/jni` 目录。

### Q: 没有 Ninja
A: 安装方法：
```powershell
winget install Kitware.Ninja
```
或者使用 Make（需要 MSYS2）。
