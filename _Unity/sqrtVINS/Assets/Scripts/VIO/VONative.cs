using System;
using System.Runtime.InteropServices;
using UnityEngine;

namespace SqrtVINS
{
    /// <summary>
    /// VIO 原生接口包装类 - 封装 libvo_unity.so 的 P/Invoke 调用
    /// 版本 2.0: 使用完整 MSCKF VIO (VioManager)
    /// </summary>
    public static class VONative
    {
#if UNITY_ANDROID && !UNITY_EDITOR
        private const string DLL_NAME = "vo_unity";
#else
        private const string DLL_NAME = "vo_unity";
#endif

        #region 数据结构

        /// <summary>
        /// 相机内参
        /// </summary>
        [StructLayout(LayoutKind.Sequential)]
        public struct VOCameraParams
        {
            public float fx;          // 焦距 X
            public float fy;          // 焦距 Y
            public float cx;          // 主点 X
            public float cy;          // 主点 Y
            public int width;         // 图像宽度
            public int height;        // 图像高度
            public float k1;          // 畸变系数 k1
            public float k2;          // 畸变系数 k2
            public float p1;          // 畸变系数 p1
            public float p2;          // 畸变系数 p2
        }

        /// <summary>
        /// 跟踪配置参数
        /// </summary>
        [StructLayout(LayoutKind.Sequential)]
        public struct VOTrackingParams
        {
            public int maxFeatures;       // 最大特征点数
            public int fastThreshold;     // FAST 阈值
            public int gridX;             // 特征提取网格列数
            public int gridY;             // 特征提取网格行数
            public int minPxDist;         // 特征点最小像素间距
        }

        /// <summary>
        /// IMU 噪声参数 (来自标定)
        /// </summary>
        [StructLayout(LayoutKind.Sequential)]
        public struct VOImuParams
        {
            public float noiseGyro;       // 陀螺仪白噪声密度 (rad/s/sqrt(Hz))
            public float gyroWalk;        // 陀螺仪随机游走 (rad/s^2/sqrt(Hz))
            public float noiseAcc;        // 加速度计白噪声密度 (m/s^2/sqrt(Hz))
            public float accWalk;         // 加速度计随机游走 (m/s^3/sqrt(Hz))
            public float frequency;       // IMU 采样频率 (Hz)
        }

        /// <summary>
        /// 相机-IMU 外参
        /// </summary>
        [StructLayout(LayoutKind.Sequential)]
        public struct VOExtrinsics
        {
            [MarshalAs(UnmanagedType.ByValArray, SizeConst = 16)]
            public float[] T_cam_imu;     // 4x4 变换矩阵 (行优先): 相机到IMU
        }

        /// <summary>
        /// 位姿数据
        /// </summary>
        [StructLayout(LayoutKind.Sequential)]
        public struct VOPose
        {
            public float px, py, pz;      // 位置 (米)
            public float qx, qy, qz, qw;  // 四元数旋转
            public int valid;             // 是否有效
        }

        /// <summary>
        /// 特征点数据
        /// </summary>
        [StructLayout(LayoutKind.Sequential)]
        public struct VOFeature
        {
            public int id;          // 特征点 ID
            public float x;         // 像素坐标 X
            public float y;         // 像素坐标 Y
            public int status;      // 状态: 0=lost, 1=tracked, 2=new
        }

        /// <summary>
        /// IMU 测量数据
        /// </summary>
        [StructLayout(LayoutKind.Sequential)]
        public struct VOImuData
        {
            public double timestamp;  // 时间戳 (秒)
            public float ax;          // 加速度计 X (m/s^2)
            public float ay;          // 加速度计 Y (m/s^2)
            public float az;          // 加速度计 Z (m/s^2)
            public float gx;          // 陀螺仪 X (rad/s)
            public float gy;          // 陀螺仪 Y (rad/s)
            public float gz;          // 陀螺仪 Z (rad/s)
        }

        #endregion

        #region Native Functions

        /// <summary>
        /// 初始化完整 VIO 系统 (MSCKF)
        /// </summary>
        [DllImport(DLL_NAME, CallingConvention = CallingConvention.Cdecl)]
        public static extern int vo_initialize(
            ref VOCameraParams camera,
            ref VOImuParams imuParams,
            ref VOExtrinsics extrinsics,
            ref VOTrackingParams tracking);

        /// <summary>
        /// 初始化 VIO 系统 (仅相机参数，其他使用默认值)
        /// </summary>
        [DllImport(DLL_NAME, CallingConvention = CallingConvention.Cdecl)]
        public static extern int vo_initialize(
            ref VOCameraParams camera,
            IntPtr imuParams,
            IntPtr extrinsics,
            IntPtr tracking);

        /// <summary>
        /// 关闭 VIO 系统
        /// </summary>
        [DllImport(DLL_NAME, CallingConvention = CallingConvention.Cdecl)]
        public static extern int vo_shutdown();

        /// <summary>
        /// 处理一帧图像
        /// </summary>
        [DllImport(DLL_NAME, CallingConvention = CallingConvention.Cdecl)]
        public static extern int vo_process_frame(IntPtr imageData, int width, int height, int channels, double timestamp);

        /// <summary>
        /// 获取当前位姿
        /// </summary>
        [DllImport(DLL_NAME, CallingConvention = CallingConvention.Cdecl)]
        public static extern int vo_get_pose(ref VOPose pose);

        /// <summary>
        /// 获取特征点数量
        /// </summary>
        [DllImport(DLL_NAME, CallingConvention = CallingConvention.Cdecl)]
        public static extern int vo_get_feature_count();

        /// <summary>
        /// 获取特征点数据
        /// </summary>
        [DllImport(DLL_NAME, CallingConvention = CallingConvention.Cdecl)]
        public static extern int vo_get_features(IntPtr features, int maxCount, ref int outCount);

        /// <summary>
        /// 获取点云数据
        /// </summary>
        [DllImport(DLL_NAME, CallingConvention = CallingConvention.Cdecl)]
        public static extern int vo_get_point_cloud(IntPtr buffer, int maxPoints, ref int actualPoints);

        /// <summary>
        /// 获取调试图像
        /// </summary>
        [DllImport(DLL_NAME, CallingConvention = CallingConvention.Cdecl)]
        public static extern int vo_get_debug_image(IntPtr outputImage, int width, int height, int drawPoints, int drawFlow);

        /// <summary>
        /// 输入 IMU 测量数据
        /// </summary>
        [DllImport(DLL_NAME, CallingConvention = CallingConvention.Cdecl)]
        public static extern int vo_feed_imu(ref VOImuData imuData);

        /// <summary>
        /// 重置 IMU 积分状态
        /// </summary>
        [DllImport(DLL_NAME, CallingConvention = CallingConvention.Cdecl)]
        public static extern int vo_reset_imu();

        /// <summary>
        /// 获取版本号
        /// </summary>
        [DllImport(DLL_NAME, CallingConvention = CallingConvention.Cdecl)]
        public static extern void vo_get_version(ref int major, ref int minor, ref int patch);

        /// <summary>
        /// 获取 Unity 渲染事件回调函数
        /// </summary>
        [DllImport(DLL_NAME, CallingConvention = CallingConvention.Cdecl)]
        public static extern IntPtr vo_get_render_event_func();

        /// <summary>
        /// 设置原生纹理指针 (OpenGL Texture ID)
        /// </summary>
        [DllImport(DLL_NAME, CallingConvention = CallingConvention.Cdecl)]
        public static extern int vo_set_native_texture(IntPtr texturePtr, int width, int height);

        /// <summary>
        /// 启动原生 IMU 采集 (Android Sensor API)
        /// </summary>
        [DllImport(DLL_NAME, CallingConvention = CallingConvention.Cdecl)]
        public static extern int vo_start_native_imu(int target_hz);

        /// <summary>
        /// 停止原生 IMU 采集
        /// </summary>
        [DllImport(DLL_NAME, CallingConvention = CallingConvention.Cdecl)]
        public static extern int vo_stop_native_imu();

        /// <summary>
        /// 检查原生 IMU 是否正在运行
        /// </summary>
        [DllImport(DLL_NAME, CallingConvention = CallingConvention.Cdecl)]
        public static extern int vo_is_native_imu_running();

        /// <summary>
        /// 获取原生传感器时间戳 (CLOCK_BOOTTIME 秒)
        /// </summary>
        [DllImport(DLL_NAME, CallingConvention = CallingConvention.Cdecl)]
        public static extern double vo_get_native_sensor_timestamp();

        #endregion

        #region Helper Methods

        /// <summary>
        /// 将 VOPose 转换为 Unity 的 Pose
        /// 注意: 坐标系转换已在 C++ 层完成，这里直接使用
        /// </summary>
        public static Pose ToUnityPose(VOPose voPose)
        {
            // C++ 层已经完成了 VIO->Unity 的坐标转换:
            // 位置: (VIO_X, VIO_Z, VIO_Y) 映射到 Unity (X, Y-up, Z-forward)
            // 旋转: 通过变换矩阵 T 完成了右手->左手+轴置换的转换
            Vector3 pos = new Vector3(voPose.px, voPose.py, voPose.pz);
            Quaternion rot = new Quaternion(voPose.qx, voPose.qy, voPose.qz, voPose.qw);
            return new Pose(pos, rot);
        }

        /// <summary>
        /// 创建默认相机参数
        /// </summary>
        public static VOCameraParams CreateDefaultCameraParams(int width, int height)
        {
            return new VOCameraParams
            {
                fx = width * 0.8f,
                fy = width * 0.8f,
                cx = width / 2f,
                cy = height / 2f,
                width = width,
                height = height,
                k1 = 0, k2 = 0, p1 = 0, p2 = 0
            };
        }

        /// <summary>
        /// 创建默认 IMU 噪声参数
        /// </summary>
        public static VOImuParams CreateDefaultImuParams()
        {
            return new VOImuParams
            {
                noiseGyro = 2.0897790366737560e-03f,
                gyroWalk = 3.3686475741950929e-05f,
                noiseAcc = 3.0690310195074056e-02f,
                accWalk = 9.7833628333635959e-04f,
                frequency = 100.0f
            };
        }

        /// <summary>
        /// 创建默认外参 (相机与IMU对齐)
        /// </summary>
        public static VOExtrinsics CreateIdentityExtrinsics()
        {
            var ext = new VOExtrinsics();
            ext.T_cam_imu = new float[16];
            // Identity matrix (row-major)
            ext.T_cam_imu[0] = 1; ext.T_cam_imu[5] = 1;
            ext.T_cam_imu[10] = 1; ext.T_cam_imu[15] = 1;
            return ext;
        }

        /// <summary>
        /// 获取版本字符串
        /// </summary>
        public static string GetVersionString()
        {
            int major = 0, minor = 0, patch = 0;
            try
            {
                vo_get_version(ref major, ref minor, ref patch);
                return $"{major}.{minor}.{patch}";
            }
            catch (Exception)
            {
                return "Unknown";
            }
        }

        #endregion
    }
}
