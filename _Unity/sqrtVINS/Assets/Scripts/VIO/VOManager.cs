using System;
using System.Collections;
using System.Runtime.InteropServices;
using UnityEngine;
using UnityEngine.Events;

namespace SqrtVINS
{
    /// <summary>
    /// SqrtVINS 视觉惯性里程计管理器
    /// 负责初始化、更新和管理 VIO 系统
    /// </summary>
    public class VOManager : MonoBehaviour
    {
        
        private static VOManager _instance;
        public static VOManager Instance
        {
            get
            {
                if (_instance == null)
                {
                    _instance = FindObjectOfType<VOManager>();
                }
                return _instance;
            }
        }

        [Header("相机参数")]
        [SerializeField] private float focalLength = 500f;
        [SerializeField] private int imageWidth = 640;
        [SerializeField] private int imageHeight = 480;

        [Header("跟踪参数")]
        [SerializeField] private int maxFeatures = 200;
        [SerializeField] private int pyramidLevels = 3;

        [Header("更新设置")]
        [SerializeField] private bool autoUpdate = true;
        [SerializeField] private float updateInterval = 0.033f; // 约 30 FPS

        public enum VOState
        {
            Uninitialized,
            Initializing,
            Running,
            Lost,
            Error
        }

        private VOState _currentState = VOState.Uninitialized;
        public VOState CurrentState => _currentState;

        private VONative.VOPose _currentPose;
        public Pose CurrentPose => VONative.ToUnityPose(_currentPose);

        public bool IsTracking => _currentState == VOState.Running && _currentPose.valid != 0;

        [Header("事件")]
        public UnityEvent<Pose> OnPoseUpdated;
        public UnityEvent<VOState> OnStateChanged;
        public UnityEvent OnTrackingLost;
        public UnityEvent OnTrackingRecovered;

        private void Awake()
        {
            if (_instance != null && _instance != this)
            {
                Destroy(gameObject);
                return;
            }
            _instance = this;
            DontDestroyOnLoad(gameObject);
        }

        private void OnDestroy()
        {
            Shutdown();
        }

        private void OnApplicationPause(bool pauseStatus)
        {
            if (pauseStatus)
            {
                // 应用暂停时可以选择暂停 VIO
                Debug.Log("[VOManager] Application paused");
            }
            else
            {
                // 恢复时可能需要重置
                Debug.Log("[VOManager] Application resumed");
            }
        }


        /// <summary>
        /// 初始化 VIO 系统
        /// </summary>
        public bool Initialize()
        {
            if (_currentState != VOState.Uninitialized)
            {
                Debug.LogWarning("[VOManager] Already initialized");
                return false;
            }

            SetState(VOState.Initializing);

            try
            {
                VONative.VOCameraParams cameraParams = new VONative.VOCameraParams
                {
                    fx = focalLength,
                    fy = focalLength,
                    cx = imageWidth / 2f,
                    cy = imageHeight / 2f,
                    width = imageWidth,
                    height = imageHeight,
                    k1 = 0, k2 = 0, p1 = 0, p2 = 0
                };

                // 设置跟踪参数
                VONative.VOTrackingParams trackingParams = new VONative.VOTrackingParams
                {
                    maxFeatures = maxFeatures,
                    pyramidLevels = pyramidLevels,
                    fastThreshold = 20, // 默认值
                    minDistance = 20f   // 默认值
                };

                int result = 0;
                
                // 将结构体分配到非托管内存
                IntPtr ptr = Marshal.AllocHGlobal(Marshal.SizeOf(trackingParams));
                try
                {
                    Marshal.StructureToPtr(trackingParams, ptr, false);
                    result = VONative.vo_initialize(ref cameraParams, ptr);
                }
                finally
                {
                    Marshal.FreeHGlobal(ptr);
                }


                if (result == 0)
                {
                    SetState(VOState.Running);
                    Debug.Log("[VOManager] VIO initialized successfully");
                    
                    if (autoUpdate)
                    {
                        StartCoroutine(UpdateLoop());
                    }
                    
                    return true;
                }
                else
                {
                    SetState(VOState.Error);
                    Debug.LogError($"[VOManager] Failed to initialize VIO, error code: {result}");
                    return false;
                }
            }
            catch (Exception e)
            {
                SetState(VOState.Error);
                Debug.LogError($"[VOManager] Exception during initialization: {e.Message}");
                return false;
            }
        }

        /// <summary>
        /// 初始化 VIO 系统（使用自定义相机参数）
        /// </summary>
        public bool Initialize(VONative.VOCameraParams cameraParams)
        {
            imageWidth = cameraParams.width;
            imageHeight = cameraParams.height;
            focalLength = cameraParams.fx;

            if (_currentState != VOState.Uninitialized)
            {
                Shutdown();
            }

            SetState(VOState.Initializing);

            try
            {
                // 设置跟踪参数
                VONative.VOTrackingParams trackingParams = new VONative.VOTrackingParams
                {
                    maxFeatures = maxFeatures,
                    pyramidLevels = pyramidLevels,
                    fastThreshold = 20,
                    minDistance = 20f
                };

                int result = 0;
                IntPtr ptr = Marshal.AllocHGlobal(Marshal.SizeOf(trackingParams));
                try
                {
                    Marshal.StructureToPtr(trackingParams, ptr, false);
                    result = VONative.vo_initialize(ref cameraParams, ptr);
                }
                finally
                {
                    Marshal.FreeHGlobal(ptr);
                }


                if (result == 0)
                {
                    SetState(VOState.Running);
                    Debug.Log("[VOManager] VIO initialized with custom params");
                    
                    if (autoUpdate)
                    {
                        StartCoroutine(UpdateLoop());
                    }
                    
                    return true;
                }
                else
                {
                    SetState(VOState.Error);
                    Debug.LogError($"[VOManager] Failed to initialize VIO: {result}");
                    return false;
                }
            }
            catch (Exception e)
            {
                SetState(VOState.Error);
                Debug.LogError($"[VOManager] Exception: {e.Message}");
                return false;
            }
        }

        /// <summary>
        /// 处理一帧图像数据
        /// </summary>
        private int _frameCount = 0;
        
        public bool ProcessFrame(byte[] imageData, int width, int height, int channels, double timestamp)
        {
            if (_currentState != VOState.Running && _currentState != VOState.Lost)
            {
                Debug.Log($"[VOManager] ProcessFrame skipped: state={_currentState}");
                return false;
            }

            try
            {
                // 固定内存并传递给原生代码
                GCHandle handle = GCHandle.Alloc(imageData, GCHandleType.Pinned);
                try
                {
                    int result = VONative.vo_process_frame(
                        handle.AddrOfPinnedObject(),
                        width,
                        height,
                        channels,
                        timestamp
                    );

                    if (result == 0)
                    {
                        _frameCount++;
                        if (_frameCount % 30 == 0) // 每30帧打印一次
                        {
                            Debug.Log($"[VOManager] Processed {_frameCount} frames");
                        }
                        // 更新位姿
                        UpdatePose();
                        return true;
                    }
                    else
                    {
                        Debug.LogWarning($"[VOManager] Process frame failed: {result}");
                        return false;
                    }
                }
                finally
                {
                    handle.Free();
                }
            }
            catch (Exception e)
            {
                Debug.LogError($"[VOManager] ProcessFrame exception: {e.Message}");
                return false;
            }
        }

        /// <summary>
        /// 重置 VIO 跟踪
        /// </summary>
        public void Reset()
        {
            if (_currentState == VOState.Uninitialized)
            {
                return;
            }

            try
            {
                VONative.vo_reset();
                SetState(VOState.Running);
                Debug.Log("[VOManager] VIO reset");
            }
            catch (Exception e)
            {
                Debug.LogError($"[VOManager] Reset exception: {e.Message}");
            }
        }

        /// <summary>
        /// 关闭 VIO 系统
        /// </summary>
        public void Shutdown()
        {
            if (_currentState == VOState.Uninitialized)
            {
                return;
            }

            StopAllCoroutines();

            try
            {
                VONative.vo_shutdown();
                Debug.Log("[VOManager] VIO shutdown");
            }
            catch (Exception e)
            {
                Debug.LogError($"[VOManager] Shutdown exception: {e.Message}");
            }

            SetState(VOState.Uninitialized);
        }

        private void UpdatePose()
        {
            bool wasTracking = _currentPose.valid != 0;

            int result = VONative.vo_get_pose(ref _currentPose);

            if (result == 0 && _currentPose.valid != 0)
            {
                Pose unityPose = VONative.ToUnityPose(_currentPose);
                OnPoseUpdated?.Invoke(unityPose);

                if (!wasTracking)
                {
                    SetState(VOState.Running);
                    OnTrackingRecovered?.Invoke();
                }
            }
            else
            {
                if (wasTracking)
                {
                    SetState(VOState.Lost);
                    OnTrackingLost?.Invoke();
                }
            }
        }


        private void SetState(VOState newState)
        {
            if (_currentState != newState)
            {
                _currentState = newState;
                OnStateChanged?.Invoke(newState);
                Debug.Log($"[VOManager] State changed to: {newState}");
            }
        }

        private IEnumerator UpdateLoop()
        {
            WaitForSeconds wait = new WaitForSeconds(updateInterval);

            while (_currentState == VOState.Running || _currentState == VOState.Lost)
            {
                UpdatePose();
                yield return wait;
            }
        }

    }
}
