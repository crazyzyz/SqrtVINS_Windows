using System;
using System.Runtime.InteropServices;
using UnityEngine;

namespace SqrtVINS
{
    /// <summary>
    /// VO 原生接口包装类 - 封装 libvo_unity.so 的 P/Invoke 调用
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
            public int pyramidLevels;     // 金字塔层数
            public int fastThreshold;     // FAST 阈值
            public float minDistance;      // 特征点最小间距
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
            public int trackingState;     // 跟踪状态
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

        #endregion

        #region Native Functions

        /// <summary>
        /// 初始化 VO 系统
        /// </summary>
        [DllImport(DLL_NAME, CallingConvention = CallingConvention.Cdecl)]
        public static extern int vo_initialize(ref VOCameraParams camera, IntPtr tracking);

        /// <summary>
        /// 关闭 VO 系统
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
        /// 获取带有特征点和光流绘制的调试图像
        /// </summary>
        /// <param name="outputImage">输出 RGBA 图像缓冲区 (必须为 width*height*4 字节)</param>
        /// <param name="width">图像宽度</param>
        /// <param name="height">图像高度</param>
        /// <param name="drawPoints">是否绘制特征点 (红色圆点)</param>
        /// <param name="drawFlow">是否绘制光流线 (绿色线条)</param>
        [DllImport(DLL_NAME, CallingConvention = CallingConvention.Cdecl)]
        public static extern int vo_get_debug_image(IntPtr outputImage, int width, int height, int drawPoints, int drawFlow);

        /// <summary>
        /// 重置跟踪
        /// </summary>
        [DllImport(DLL_NAME, CallingConvention = CallingConvention.Cdecl)]
        public static extern int vo_reset();

        /// <summary>
        /// 获取版本号 (这里的签名可能需要修正，原生是 void vo_get_version(int*, int*, int*))
        /// 原来的 vo_get_version 是 (IntPtr, int) 可能是错误的，不过我们先加上新的函数
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


        #endregion

        #region Helper Methods

        /// <summary>
        /// 将 VOPose 转换为 Unity 的 Pose
        /// </summary>
        public static Pose ToUnityPose(VOPose voPose)
        {
            // VIO 使用右手坐标系 (通常 Z 轴朝上)，Unity 使用左手坐标系 (Y 轴朝上)
            // 1. 基础转换：右手转左手 (翻转 X 轴)
            Vector3 rawPos = new Vector3(-voPose.px, voPose.py, voPose.pz);
            // 修复：翻滚方向反了 -> 取反 Quaternion 的 Z 分量
            // 原: Quaternion rawRot = new Quaternion(-voPose.qx, voPose.qy, voPose.qz, -voPose.qw);
            // 新: Z 取反 (同时注意实部 w 的符号通常不需要变，但为了保持共轭关系，我们直接反转 Z 轴旋转分量)
            Quaternion rawRot = new Quaternion(-voPose.qx, voPose.qy, -voPose.qz, -voPose.qw);
            
            // 2. 坐标系对齐：将 VIO 的 Z 轴向上对齐到 Unity 的 Y 轴向上
            // 这通常需要绕 X 轴旋转 90 度
            Quaternion fix = Quaternion.Euler(90, 0, 0);
            
            return new Pose(fix * rawPos, fix * rawRot);
        }

        /// <summary>
        /// 创建默认相机参数
        /// </summary>
        public static VOCameraParams CreateDefaultCameraParams(int width, int height)
        {
            return new VOCameraParams
            {
                fx = width * 0.8f,   // 估计焦距
                fy = width * 0.8f,
                cx = width / 2f,
                cy = height / 2f,
                width = width,
                height = height,
                k1 = 0, k2 = 0, p1 = 0, p2 = 0  // 无畸变
            };
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
