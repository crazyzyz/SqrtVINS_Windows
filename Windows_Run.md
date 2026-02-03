# sqrtVINS Windows 构建指南

本项目已移除 ROS 依赖，可以在纯 Windows 环境下使用 Visual Studio 进行编译。

## 依赖项

- **Visual Studio 2022** (或更高版本)
- **CMake 3.3+**
- **vcpkg** (用于管理依赖库)

### 通过 vcpkg 安装依赖

```powershell
vcpkg install eigen3:x64-windows
vcpkg install opencv4:x64-windows
vcpkg install boost:x64-windows
```

## 构建步骤

### 1. 创建并进入构建目录

```powershell
mkdir build_vs
cd build_vs
```

### 2. 运行 CMake 配置

```powershell
cmake ../ov_srvins -G "Visual Studio 17 2022" -A x64 -DCMAKE_TOOLCHAIN_FILE="C:/vcpkg/scripts/buildsystems/vcpkg.cmake" -DENABLE_ROS=OFF -DENABLE_ARUCO_TAGS=OFF -DUSE_FLOAT=OFF
```

> 注意：请根据你实际的 vcpkg 安装路径修改 `-DCMAKE_TOOLCHAIN_FILE`

### 3. 编译

```powershell
cmake --build . --config Release
```

## 运行

编译成功后，可执行文件位于 `build_vs/Release/` 目录下。

### 运行仿真 Demo

```powershell
.\build_vs\Release\run_simulation.exe config\rpng_sim\estimator_config.yaml
```

### 运行摄像头测试

```powershell
.\build_vs\Release\test_webcam.exe config\rpng_sim\estimator_config.yaml
```

## 生成的可执行文件

| 文件名 | 用途 |
|--------|------|
| `run_simulation.exe` | 运行仿真 Demo，验证 VIO 算法 |
| `test_webcam.exe` | 测试 USB 摄像头的前端特征跟踪 |
| `test_sim_meas.exe` | 测试仿真测量数据模块 |
| `test_sim_repeat.exe` | 测试仿真重复性 |

