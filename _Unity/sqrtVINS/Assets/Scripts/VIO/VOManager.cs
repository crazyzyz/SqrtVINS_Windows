using System;
using System.Collections;
using System.Collections.Concurrent;
using System.Collections.Generic;
using System.Runtime.InteropServices;
using System.Threading;
using UnityEngine;
using UnityEngine.Events;

namespace SqrtVINS
{
    /// <summary>
    /// SqrtVINS 视觉惯性里程计管理器
    /// 负责初始化、更新和管理 VIO 系统
    /// 优化：使用独立工作线程处理 VIO 核心计算，避免阻塞 Unity 主线程
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

        [Header("IMU 参数")]
        [SerializeField] private bool useImu = true;
        [Tooltip("IMU 数据发送频率 (Hz)")]
        [SerializeField] private float imuUpdateRate = 100f;

        // 目前不使用定时更新，由相机驱动
        // [SerializeField] private bool autoUpdate = true;
        // [SerializeField] private float updateInterval = 0.033f; 

        public enum VOState
        {
            Uninitialized,
            Initializing,
            Running,
            Lost,
            Error
        }

        private volatile VOState _currentState = VOState.Uninitialized;
        public VOState CurrentState => _currentState;

        // 当前位姿 (由工作线程写入，主线程读取)
        private VONative.VOPose _currentPose;
        private object _poseLock = new object();
        private bool _hasNewPose = false;

        public Pose CurrentPose 
        {
            get 
            {
                lock (_poseLock)
                {
                    return VONative.ToUnityPose(_currentPose);
                }
            }
        }

        public bool IsTracking => _currentState == VOState.Running && _currentPose.valid != 0;

        private Thread _workerThread;
        private AutoResetEvent _workerSignal;
        private CancellationTokenSource _cancellationTokenSource;
        private ConcurrentQueue<FrameData> _frameQueue;
        private ConcurrentQueue<byte[]> _bufferPool;
        
        private const int MAX_QUEUE_SIZE = 5;

        // IMU 相关变量
        private float _lastImuTime = 0f;
        private float _imuInterval; 

        private struct FrameData
        {
            public byte[] Data;
            public int Width;
            public int Height;
            public int Channels;
            public double Timestamp;
        }

        [Header("Events")]
        public UnityEvent<Pose> OnPoseUpdated;
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

            _frameQueue = new ConcurrentQueue<FrameData>();

            // 初始化 IMU 间隔
            _imuInterval = 1f / imuUpdateRate;

            // 启用陀螺仪
            if (useImu)
            {
                Input.gyro.enabled = true;
                Debug.Log("[VOManager] Gyroscope enabled");
            }

            _bufferPool = new ConcurrentQueue<byte[]>();
            _workerSignal = new AutoResetEvent(false);
            _cancellationTokenSource = new CancellationTokenSource();
        }

        private void OnDestroy()
        {
            Shutdown();
            _workerSignal?.Close();
            _cancellationTokenSource?.Dispose();
        }

        private void OnApplicationPause(bool pauseStatus)
        {
            if (pauseStatus)
            {
                Debug.Log("[VOManager] Application paused");
            }
            else
            {
                Debug.Log("[VOManager] Application resumed");
            }
        }

        private void Update()
        {
            // 发送 IMU 数据
            if (useImu && _currentState == VOState.Running)
            {
                SendImuData();
            }

            // 在主线程分发事件
            if (_hasNewPose)
            {
                Pose pose;
                bool valid;
                
                lock (_poseLock)
                {
                    pose = VONative.ToUnityPose(_currentPose);
                    valid = _currentPose.valid != 0;
                    _hasNewPose = false;
                }

                OnPoseUpdated?.Invoke(pose);

                // 简单的状态机逻辑
                if (valid && _currentState == VOState.Lost)
                {
                    SetState(VOState.Running);
                    OnTrackingRecovered?.Invoke();
                }
                else if (!valid && _currentState == VOState.Running)
                {
                    SetState(VOState.Lost);
                    OnTrackingLost?.Invoke();
                }
            }
        }

        public bool Initialize()
        {
            return InitializeSystem(new VONative.VOCameraParams
            {
                fx = focalLength,
                fy = focalLength,
                cx = imageWidth / 2f,
                cy = imageHeight / 2f,
                width = imageWidth,
                height = imageHeight,
                k1 = 0, k2 = 0, p1 = 0, p2 = 0
            });
        }

        public bool Initialize(VONative.VOCameraParams cameraParams)
        {
            return InitializeSystem(cameraParams);
        }

        private bool InitializeSystem(VONative.VOCameraParams cameraParams)
        {
            if (_currentState != VOState.Uninitialized)
            {
                Debug.LogWarning("[VOManager] Already initialized");
                return false;
            }

            // Sync internal state
            imageWidth = cameraParams.width;
            imageHeight = cameraParams.height;
            focalLength = cameraParams.fx;

            SetState(VOState.Initializing);

            VONative.VOTrackingParams trackingParams = new VONative.VOTrackingParams
            {
                maxFeatures = maxFeatures,
                pyramidLevels = pyramidLevels,
                fastThreshold = 20,
                minDistance = 20f
            };

            int result;
            IntPtr ptr = Marshal.AllocHGlobal(Marshal.SizeOf(trackingParams));
            try
            {
                Marshal.StructureToPtr(trackingParams, ptr, false);
                result = VONative.vo_initialize(ref cameraParams, ptr);
            }
            catch (Exception e)
            {
                Debug.LogError($"[VOManager] Init exception: {e.Message}");
                SetState(VOState.Error);
                return false;
            }
            finally
            {
                Marshal.FreeHGlobal(ptr);
            }

            if (result == 0)
            {
                SetState(VOState.Running);
                Debug.Log("[VOManager] VIO initialized successfully");
                StartWorker();
                return true;
            }
            else
            {
                SetState(VOState.Error);
                Debug.LogError($"[VOManager] Failed to initialize VIO: {result}");
                return false;
            }
        }

        private int _frameCount = 0;
        private int _dropCount = 0;

        /// <summary>
        /// 接收帧数据并入队处理（非阻塞）
        /// </summary>
        public bool ProcessFrame(byte[] imageData, int width, int height, int channels, double timestamp)
        {
            if (_currentState != VOState.Running && _currentState != VOState.Lost) return false;

            // Simple backpressure
            if (_frameQueue.Count >= MAX_QUEUE_SIZE)
            {
                _dropCount++;
                if (_dropCount % 30 == 0) 
                    Debug.LogWarning($"[VOManager] Dropping frames! Queue size: {_frameQueue.Count}");
                return false;
            }

            // 1. Get buffer from pool
            if (!_bufferPool.TryDequeue(out byte[] buffer) || buffer.Length != imageData.Length)
            {
                buffer = new byte[imageData.Length];
            }

            // 2. Copy data (Main thread overhead)
            Array.Copy(imageData, buffer, imageData.Length);

            // 3. Enqueue
            _frameQueue.Enqueue(new FrameData
            {
                Data = buffer,
                Width = width,
                Height = height,
                Channels = channels,
                Timestamp = timestamp
            });

            // 4. Signal worker
            _workerSignal.Set();

            return true;
        }

        public void Reset()
        {
            if (_currentState == VOState.Uninitialized) return;
            
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

        public void Shutdown()
        {
            if (_currentState == VOState.Uninitialized) return;

            StopWorker();

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

        private void StartWorker()
        {
            if (_workerThread != null && _workerThread.IsAlive) return;

            // Renew cancellation token if needed being careful with disposal
            if (_cancellationTokenSource != null && _cancellationTokenSource.IsCancellationRequested)
            {
                _cancellationTokenSource.Dispose();
                _cancellationTokenSource = new CancellationTokenSource();
            }

            _workerThread = new Thread(WorkerLoop)
            {
                Name = "VIO_Worker_Thread",
                IsBackground = true
            };
            _workerThread.Start();
            Debug.Log("[VOManager] Worker thread started");
        }

        private void StopWorker()
        {
            if (_cancellationTokenSource != null && !_cancellationTokenSource.IsCancellationRequested)
                _cancellationTokenSource.Cancel();
            
            _workerSignal?.Set(); // Wake up to check cancellation

            if (_workerThread != null)
            {
                if (!_workerThread.Join(500)) // Wait 500ms
                {
                    Debug.LogWarning("[VOManager] Worker thread join timed out");
                }
                _workerThread = null;
            }
            
            // Clear queue
            while (_frameQueue.TryDequeue(out _)) { }
        }

        private void WorkerLoop()
        {
            try 
            {
                while (_cancellationTokenSource != null && !_cancellationTokenSource.IsCancellationRequested)
                {
                    _workerSignal.WaitOne(100);

                    if (_cancellationTokenSource.IsCancellationRequested) break;

                    while (_frameQueue.TryDequeue(out FrameData frame))
                    {
                         if (_cancellationTokenSource.IsCancellationRequested) break;
                         ProcessFrameInternal(frame);
                         if (_bufferPool.Count < 30) _bufferPool.Enqueue(frame.Data);
                    }
                }
            }
            catch (Exception e)
            {
                Debug.LogError($"[VOManager] Worker thread exception: {e.Message}");
            }
            finally
            {
                Debug.Log("[VOManager] Worker thread exited");
            }
        }

        private void ProcessFrameInternal(FrameData frame)
        {
            GCHandle handle = GCHandle.Alloc(frame.Data, GCHandleType.Pinned);
            try
            {
                int result = VONative.vo_process_frame(
                    handle.AddrOfPinnedObject(),
                    frame.Width,
                    frame.Height,
                    frame.Channels,
                    frame.Timestamp
                );

                if (result == 0)
                {
                    _frameCount++;
                    // Only log periodically to reduce spam
                    if (_frameCount % 100 == 0) Debug.Log($"[VOManager] Processed {_frameCount} frames");

                    VONative.VOPose newPose = new VONative.VOPose();
                    int poseResult = VONative.vo_get_pose(ref newPose);

                    if (poseResult == 0)
                    {
                        lock (_poseLock)
                        {
                            _currentPose = newPose;
                            _hasNewPose = true;
                        }

                        // 诊断日志：每30帧记录一次位姿和valid状态
                        if (_frameCount % 30 == 0)
                        {
                            string validStatus = newPose.valid == 0 ? "INVALID(IMU)" : "VALID(PnP)";
                            Debug.Log($"[VOManager] Frame {_frameCount}: {validStatus}, " +
                                     $"Pos=({newPose.px:F3}, {newPose.py:F3}, {newPose.pz:F3}), " +
                                     $"Quat=({newPose.qx:F2}, {newPose.qy:F2}, {newPose.qz:F2}, {newPose.qw:F2})");
                        }
                    }
                }
            }
            catch (Exception e)
            {
                 Debug.LogError($"[VOManager] Pinned memory exception: {e.Message}");
            }
            finally
            {
                handle.Free();
            }
        }

        /// <summary>
        /// 发送 IMU 数据到 SO 库
        /// </summary>
        private void SendImuData()
        {
            float currentTime = Time.time;
            if (currentTime - _lastImuTime < _imuInterval)
                return;

            _lastImuTime = currentTime;

            // 获取 Unity 的加速度计和陀螺仪数据
            Vector3 accel = Input.acceleration;
            Vector3 gyro = Input.gyro.rotationRateUnbiased;

            // 构造 IMU 数据
            VONative.VOImuData imuData = new VONative.VOImuData
            {
                timestamp = Time.timeAsDouble,
                ax = accel.x * 9.81f,
                ay = accel.y * 9.81f,
                az = accel.z * 9.81f,
                gx = gyro.x,
                gy = gyro.y,
                gz = gyro.z
            };

            // 发送到 SO 库
            VONative.vo_feed_imu(ref imuData);
        }

        private void SetState(VOState newState)
        {
            if (_currentState != newState)
            {
                _currentState = newState;
                Debug.Log($"[VOManager] State changed to: {newState}");
            }
        }
    }
}
