using System;
using System.Collections;
using UnityEngine;
using UnityEngine.UI;

namespace SqrtVINS
{
    /// <summary>
    /// Android 相机控制器
    /// 使用 Unity WebCamTexture 获取相机图像并传递给 VIO 系统
    /// </summary>
    public class AndroidCameraController : MonoBehaviour
    {
        #region 配置

        [Header("相机设置")]
        [SerializeField] private int requestedWidth = 640;
        [SerializeField] private int requestedHeight = 480;
        [SerializeField] private int requestedFPS = 30;
        [SerializeField] private bool useFrontCamera = false;

        [Header("预览设置")]
        [SerializeField] private RawImage previewImage;
        [SerializeField] private bool showPreview = true;

        [Header("VIO 设置")]
        [SerializeField] private bool autoStartVIO = true;
        [SerializeField] private float focalLengthEstimate = 500f;

        #endregion

        #region 状态

        private WebCamTexture _webCamTexture;
        private Color32[] _pixelBuffer;
        private byte[] _grayBuffer;
        private bool _isRunning = false;
        private double _startTime;

        public bool IsRunning => _isRunning;
        public int ActualWidth => _webCamTexture != null ? _webCamTexture.width : 0;
        public int ActualHeight => _webCamTexture != null ? _webCamTexture.height : 0;

        #endregion

        #region 事件

        public event Action<byte[], int, int, double> OnFrameReady;
        public event Action OnCameraStarted;
        public event Action OnCameraStopped;
        public event Action<string> OnCameraError;

        #endregion

        #region Unity 生命周期

        private bool _hasStarted = false;

        private void Start()
        {
#if UNITY_ANDROID && !UNITY_EDITOR
            StartCoroutine(RequestCameraPermissionAndStart());
#else
            // 编辑器或其他平台直接启动
            StartCamera();
#endif
            _hasStarted = true;
        }

        private void OnDestroy()
        {
            StopCamera();
        }

        private void OnApplicationPause(bool pauseStatus)
        {
            // 只有在 Start 已经执行后才处理暂停/恢复
            if (!_hasStarted) return;
            
            if (pauseStatus)
            {
                StopCamera();
            }
            else
            {
                StartCamera();
            }
        }

        private void Update()
        {
            if (_isRunning && _webCamTexture != null && _webCamTexture.didUpdateThisFrame)
            {
                ProcessCameraFrame();
            }
        }

        #endregion

        #region 公共方法

        /// <summary>
        /// 启动相机
        /// </summary>
        public void StartCamera()
        {
            if (_isRunning)
            {
                Debug.LogWarning("[AndroidCamera] Camera already running");
                return;
            }

            StartCoroutine(StartCameraCoroutine());
        }

        /// <summary>
        /// 停止相机
        /// </summary>
        public void StopCamera()
        {
            if (_webCamTexture != null)
            {
                _webCamTexture.Stop();
                Destroy(_webCamTexture);
                _webCamTexture = null;
            }

            _isRunning = false;
            OnCameraStopped?.Invoke();
            Debug.Log("[AndroidCamera] Camera stopped");
        }

        /// <summary>
        /// 切换前后摄像头
        /// </summary>
        public void ToggleCamera()
        {
            useFrontCamera = !useFrontCamera;
            StopCamera();
            StartCamera();
        }

        #endregion

        #region 私有方法

        private IEnumerator RequestCameraPermissionAndStart()
        {
#if UNITY_ANDROID
            // 检查相机权限
            if (!UnityEngine.Android.Permission.HasUserAuthorizedPermission(UnityEngine.Android.Permission.Camera))
            {
                UnityEngine.Android.Permission.RequestUserPermission(UnityEngine.Android.Permission.Camera);
                
                // 等待用户响应
                yield return new WaitForSeconds(0.5f);
                
                while (!UnityEngine.Android.Permission.HasUserAuthorizedPermission(UnityEngine.Android.Permission.Camera))
                {
                    yield return new WaitForSeconds(0.5f);
                }
            }
#endif
            StartCamera();
            yield return null;
        }

        private IEnumerator StartCameraCoroutine()
        {
            // 获取可用相机列表
            WebCamDevice[] devices = WebCamTexture.devices;

            if (devices.Length == 0)
            {
                Debug.LogError("[AndroidCamera] No camera devices found");
                OnCameraError?.Invoke("No camera devices found");
                yield break;
            }

            // 选择相机
            string deviceName = "";
            foreach (var device in devices)
            {
                Debug.Log($"[AndroidCamera] Found camera: {device.name}, isFront: {device.isFrontFacing}");
                if (device.isFrontFacing == useFrontCamera)
                {
                    deviceName = device.name;
                    break;
                }
            }

            if (string.IsNullOrEmpty(deviceName))
            {
                deviceName = devices[0].name;
            }

            Debug.Log($"[AndroidCamera] Using camera: {deviceName}");

            // 创建 WebCamTexture
            _webCamTexture = new WebCamTexture(deviceName, requestedWidth, requestedHeight, requestedFPS);
            _webCamTexture.Play();

            // 等待相机启动
            int timeout = 50; // 5 秒超时
            while (!_webCamTexture.didUpdateThisFrame && timeout > 0)
            {
                yield return new WaitForSeconds(0.1f);
                timeout--;
            }

            if (!_webCamTexture.isPlaying)
            {
                Debug.LogError("[AndroidCamera] Failed to start camera");
                OnCameraError?.Invoke("Failed to start camera");
                yield break;
            }

            // 初始化缓冲区
            int width = _webCamTexture.width;
            int height = _webCamTexture.height;
            _pixelBuffer = new Color32[width * height];
            _grayBuffer = new byte[width * height];
            _startTime = Time.realtimeSinceStartupAsDouble;

            Debug.Log($"[AndroidCamera] Camera started: {width}x{height}");

            // 设置预览
            if (showPreview && previewImage != null)
            {
                previewImage.texture = _webCamTexture;
                previewImage.gameObject.SetActive(true);
                
                // 调整预览方向
                AdjustPreviewOrientation();
            }

            _isRunning = true;
            OnCameraStarted?.Invoke();

            // 自动启动 VIO
            if (autoStartVIO)
            {
                InitializeVIO(width, height);
            }
        }

        private void ProcessCameraFrame()
        {
            if (_webCamTexture == null || !_webCamTexture.isPlaying)
            {
                return;
            }

            int width = _webCamTexture.width;
            int height = _webCamTexture.height;

            // 获取像素数据
            _webCamTexture.GetPixels32(_pixelBuffer);

            // 转换为灰度图
            ConvertToGrayscale(_pixelBuffer, _grayBuffer, width, height);

            // 计算时间戳
            double timestamp = Time.realtimeSinceStartupAsDouble - _startTime;

            // 触发帧就绪事件
            OnFrameReady?.Invoke(_grayBuffer, width, height, timestamp);

            // 如果 VOManager 存在且已初始化，则处理帧
            if (VOManager.Instance != null && VOManager.Instance.CurrentState != VOManager.VOState.Uninitialized
                && VOManager.Instance.CurrentState != VOManager.VOState.Error)
            {
                VOManager.Instance.ProcessFrame(_grayBuffer, width, height, 1, timestamp);
            }
        }

        private void ConvertToGrayscale(Color32[] pixels, byte[] gray, int width, int height)
        {
            // 使用加权平均转换为灰度
            // Y = 0.299 R + 0.587 G + 0.114 B
            for (int i = 0; i < pixels.Length; i++)
            {
                Color32 c = pixels[i];
                gray[i] = (byte)(0.299f * c.r + 0.587f * c.g + 0.114f * c.b);
            }
        }

        private void AdjustPreviewOrientation()
        {
            if (previewImage == null) return;

            int width = _webCamTexture.width;
            int height = _webCamTexture.height;
            int videoAngle = _webCamTexture.videoRotationAngle;
            
            Debug.Log($"[AndroidCamera] AdjustPreview: texture={width}x{height}, videoRotationAngle={videoAngle}, useFrontCamera={useFrontCamera}");

            // 设置 AspectRatioFitter（如果存在）
            var aspectFitter = previewImage.GetComponent<AspectRatioFitter>();
            if (aspectFitter != null)
            {
                aspectFitter.aspectMode = AspectRatioFitter.AspectMode.FitInParent;
                aspectFitter.aspectRatio = (float)width / height;
                Debug.Log($"[AndroidCamera] AspectRatioFitter found, mode=FitInParent, ratio={aspectFitter.aspectRatio}");
            }
            else
            {
                Debug.LogWarning("[AndroidCamera] AspectRatioFitter component NOT found on previewImage!");
            }

            // 根据相机旋转调整预览
            int angle = -videoAngle;
            
            // 如果旋转了 90 或 270 度，需要交换宽高比
            if (Mathf.Abs(videoAngle) == 90 || Mathf.Abs(videoAngle) == 270)
            {
                if (aspectFitter != null)
                {
                    aspectFitter.aspectRatio = (float)height / width;
                }
            }
            
            previewImage.rectTransform.localEulerAngles = new Vector3(0, 0, angle);

            // 前置摄像头需要水平翻转
            if (useFrontCamera)
            {
                previewImage.rectTransform.localScale = new Vector3(-1, 1, 1);
            }
            else
            {
                previewImage.rectTransform.localScale = Vector3.one;
            }
        }

        private void InitializeVIO(int width, int height)
        {
            if (VOManager.Instance == null)
            {
                Debug.LogWarning("[AndroidCamera] VOManager not found, creating one");
                GameObject go = new GameObject("VOManager");
                go.AddComponent<VOManager>();
            }

            VONative.VOCameraParams cameraParams = new VONative.VOCameraParams
            {
                fx = focalLengthEstimate,
                fy = focalLengthEstimate,
                cx = width / 2f,
                cy = height / 2f,
                width = width,
                height = height,
                k1 = 0, k2 = 0, p1 = 0, p2 = 0
            };

            VOManager.Instance.Initialize(cameraParams);
        }

        #endregion
    }
}
