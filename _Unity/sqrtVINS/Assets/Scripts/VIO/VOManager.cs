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
        #region 单例模式

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

        #endregion

        #region 配置参数

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

        #endregion

        #region 状态与数据

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

        #endregion

        #region 线程与队列

        private Thread _workerThread;
        private AutoResetEvent _workerSignal;
        private volatile bool _shouldStopWorker;
        private ConcurrentQueue<FrameData> _frameQueue;
        private ConcurrentQueue<byte[]> _bufferPool;
        
        private const int MAX_QUEUE_SIZE = 5; // 如果堆积超过5帧，说明处理不过来

        private struct FrameData
        {
            public byte[] Data;
            public int Width;
            public int Height;
            public int Channels;
            public double Timestamp;
        }

        #endregion

        #region 事件

        [Header("事件")]
        public UnityEvent<Pose> OnPoseUpdated;
        public UnityEvent<VOState> OnStateChanged;
        public UnityEvent OnTrackingLost;
        public UnityEvent OnTrackingRecovered;

        #endregion

        #region Unity 生命周期

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
            _bufferPool = new ConcurrentQueue<byte[]>();
            _workerSignal = new AutoResetEvent(false);
        }

        private void OnDestroy()
        {
            Shutdown();
            if (_workerSignal != null)
            {
                _workerSignal.Close();
                _workerSignal = null;
            }
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

        #endregion

        #region 公共方法

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

            imageWidth = cameraParams.width;
            imageHeight = cameraParams.height;
            focalLength = cameraParams.fx;

            SetState(VOState.Initializing);

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
                
                // 启动工作线程
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
            if (_currentState != VOState.Running && _currentState != VOState.Lost)
            {
                return false;
            }

            // 简单的背压控制：如果队列太长，直接丢弃（避免内存爆炸和延迟累积）
            // 注意：VIO 丢帧可能导致跟丢，但处理不过来时也没办法
            if (_frameQueue.Count >= MAX_QUEUE_SIZE)
            {
                _dropCount++;
                if (_dropCount % 30 == 0) 
                    Debug.LogWarning($"[VOManager] Dropping frames! Queue size: {_frameQueue.Count}");
                return false;
            }

            // 1. 从 Pool 获取或新建 Buffer
            byte[] buffer;
            if (!_bufferPool.TryDequeue(out buffer) || buffer.Length != imageData.Length)
            {
                buffer = new byte[imageData.Length];
            }

            // 2. 拷贝数据 (主线程唯一的开销)
            Array.Copy(imageData, buffer, imageData.Length);

            // 3. 入队
            _frameQueue.Enqueue(new FrameData
            {
                Data = buffer,
                Width = width,
                Height = height,
                Channels = channels,
                Timestamp = timestamp
            });

            // 4. 唤醒工作线程
            _workerSignal.Set();

            return true;
        }

        public void Reset()
        {
            if (_currentState == VOState.Uninitialized) return;
            // Reset 操作也应该在 VIO 线程或者加锁，这里简单起见，且假设 C++ 层有锁
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

        #endregion

        #region 工作线程

        private void StartWorker()
        {
            if (_workerThread != null && _workerThread.IsAlive) return;

            _shouldStopWorker = false;
            _workerThread = new Thread(WorkerLoop);
            _workerThread.Name = "VIO_Worker_Thread";
            _workerThread.IsBackground = true;
            _workerThread.Start();
            Debug.Log("[VOManager] Worker thread started");
        }

        private void StopWorker()
        {
            _shouldStopWorker = true;
            if (_workerSignal != null) _workerSignal.Set(); // 唤醒以便退出
            
            if (_workerThread != null)
            {
                if (!_workerThread.Join(1000)) // 等待最多1秒
                {
                    Debug.LogWarning("[VOManager] Worker thread join timed out, aborting...");
                    _workerThread.Abort();
                }
                _workerThread = null;
            }
            
            // 清理队列和缓冲池
            FrameData data;
            while (_frameQueue != null && _frameQueue.TryDequeue(out data)) { }
        }

        private void WorkerLoop()
        {
            while (!_shouldStopWorker)
            {
                // 等待信号 或 超时（避免死等）
                _workerSignal.WaitOne(100);

                if (_shouldStopWorker) break;

                // 处理队列中的所有帧
                FrameData frame;
                while (_frameQueue.TryDequeue(out frame))
                {
                    ProcessFrameInternal(frame);
                    
                    // 归还 buffer 到 pool
                    if (_bufferPool.Count < 30) // 限制 Pool 大小
                    {
                        _bufferPool.Enqueue(frame.Data);
                    }
                }
            }
            Debug.Log("[VOManager] Worker thread exited");
        }

        private void ProcessFrameInternal(FrameData frame)
        {
            try
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
                        if (_frameCount % 30 == 0)
                        {
                            Debug.Log($"[VOManager] Processed {_frameCount} frames (Worker)");
                        }
                        
                        // 立即更新位姿
                        VONative.VOPose newPose = new VONative.VOPose();
                        if (VONative.vo_get_pose(ref newPose) == 0 && newPose.valid != 0)
                        {
                            lock (_poseLock)
                            {
                                _currentPose = newPose;
                                _hasNewPose = true;
                            }
                        }
                    }
                    else
                    {
                        Debug.LogWarning($"[VOManager] Worker process failed: {result}");
                    }
                }
                finally
                {
                    handle.Free();
                }
            }
            catch (Exception e)
            {
                Debug.LogError($"[VOManager] Worker exception: {e.Message}");
            }
        }

        #endregion

        private void SetState(VOState newState)
        {
            if (_currentState != newState)
            {
                _currentState = newState;
                // 注意：StateChanged 事件仍在主线程触发（如果在 Update 中检测到不一致），
                // 或者在这里触发但要小心线程安全。目前为了简单，只在主线程分发 OnPoseUpdated。
                // 如果需要严格的状态事件，建议都放到 Update 里检测。
                // 这里暂时直接 Invoke 可能有风险，但也可能没事（取决于订阅者是否操作 Unity API）
                // 安全起见，我们只打印 Log
                Debug.Log($"[VOManager] State changed to: {newState}");
            }
        }
    }
}
