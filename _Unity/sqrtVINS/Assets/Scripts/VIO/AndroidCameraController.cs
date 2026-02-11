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

        [Header("调试渲染")]
        [SerializeField] private VODebugRenderer debugRenderer;
        [SerializeField] private bool useNativeDebugImage = true;

        private WebCamTexture _webCamTexture;
        private Color32[] _pixelBuffer;
        private byte[] _grayBuffer;
        private bool _isRunning = false;
        private double _startTime;

        public bool IsRunning => _isRunning;
        public int ActualWidth => _webCamTexture != null ? _webCamTexture.width : 0;
        public int ActualHeight => _webCamTexture != null ? _webCamTexture.height : 0;

        public event Action<byte[], int, int, double> OnFrameReady;
        public event Action OnCameraStarted;
        public event Action OnCameraStopped;


        private bool _hasStarted = false;

        private void Start()
        {
#if UNITY_ANDROID && !UNITY_EDITOR
            StartCoroutine(StartCameraSequence());
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
            
            UpdatePose();
        }


        public void StartCamera()
        {
            if (_isRunning) return;
            StartCoroutine(StartCameraSequence());
        }

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
            Debug.Log("[AndroidCamera] Stopped.");
        }

        public void ToggleCamera()
        {
            useFrontCamera = !useFrontCamera;
            StopCamera();
            StartCamera();
        }

        [Header("Pose Driving")]
        [SerializeField] private bool drivePose = true;
        [SerializeField] private Transform poseTarget;
        
        private void UpdatePose()
        {
            if (!drivePose || VOManager.Instance == null || !VOManager.Instance.IsTracking) return;
            
            if (poseTarget == null && Camera.main != null) 
                poseTarget = Camera.main.transform;

            if (poseTarget != null)
            {
                var pose = VOManager.Instance.CurrentPose;
                poseTarget.position = pose.position;
                poseTarget.rotation = pose.rotation;
            }
        }

        private IEnumerator StartCameraSequence()
        {
#if UNITY_ANDROID && !UNITY_EDITOR
            if (!UnityEngine.Android.Permission.HasUserAuthorizedPermission(UnityEngine.Android.Permission.Camera))
            {
                UnityEngine.Android.Permission.RequestUserPermission(UnityEngine.Android.Permission.Camera);
                yield return new WaitForSeconds(0.5f);
                while (!UnityEngine.Android.Permission.HasUserAuthorizedPermission(UnityEngine.Android.Permission.Camera))
                {
                    yield return new WaitForSeconds(0.5f);
                }
            }
#endif
            yield return StartCameraInternal();
        }

        private IEnumerator StartCameraInternal()
        {
            var devices = WebCamTexture.devices;
            if (devices.Length == 0)
            {
                Debug.LogError("[AndroidCamera] No devices found.");
                yield break;
            }

            string deviceName = devices[0].name;
            foreach (var d in devices)
            {
                if (d.isFrontFacing == useFrontCamera)
                {
                    deviceName = d.name;
                    break;
                }
            }

            _webCamTexture = new WebCamTexture(deviceName, requestedWidth, requestedHeight, requestedFPS);
            _webCamTexture.Play();

            // Timeout safety
            float timeout = 5f;
            while (!_webCamTexture.didUpdateThisFrame && timeout > 0)
            {
                yield return new WaitForSeconds(0.1f);
                timeout -= 0.1f;
            }

            if (!_webCamTexture.isPlaying)
            {
                Debug.LogError("[AndroidCamera] Failed to play webcam.");
                yield break;
            }

            // Init buffers
            int w = _webCamTexture.width;
            int h = _webCamTexture.height;
            _pixelBuffer = new Color32[w * h];
            _grayBuffer = new byte[w * h];
            _startTime = Time.realtimeSinceStartupAsDouble;

            Debug.Log($"[AndroidCamera] Started: {w}x{h}");

            // Preview Setup
            if (showPreview && previewImage != null && !useNativeDebugImage)
            {
                previewImage.texture = _webCamTexture;
                previewImage.gameObject.SetActive(true);
                AdjustPreviewOrientation();
            }

            _isRunning = true;
            OnCameraStarted?.Invoke();

            if (autoStartVIO)
            {
                InitializeVIO(w, h);
                // 同步时间基准，确保 IMU 和相机帧使用同一时钟
                if (VOManager.Instance != null)
                    VOManager.Instance.SetTimeBase(_startTime);
            }
        }

        private void ProcessCameraFrame()
        {
            if (_webCamTexture == null || !_webCamTexture.isPlaying) return;

            _webCamTexture.GetPixels32(_pixelBuffer);

            // Conversion happens here
            ProcessGrayscale(_pixelBuffer, _grayBuffer);

            double ts;
#if UNITY_ANDROID && !UNITY_EDITOR
            // 当原生 IMU 运行时，使用 CLOCK_BOOTTIME 时间戳保持与 IMU 同步
            double nativeTs = VONative.vo_get_native_sensor_timestamp();
            if (nativeTs > 0)
            {
                ts = nativeTs;
            }
            else
            {
                ts = Time.realtimeSinceStartupAsDouble - _startTime;
            }
#else
            ts = Time.realtimeSinceStartupAsDouble - _startTime;
#endif
            OnFrameReady?.Invoke(_grayBuffer, _webCamTexture.width, _webCamTexture.height, ts);

            if (VOManager.Instance != null &&
                (VOManager.Instance.CurrentState == VOManager.VOState.Running ||
                 VOManager.Instance.CurrentState == VOManager.VOState.Initializing ||
                 VOManager.Instance.CurrentState == VOManager.VOState.Lost))
            {
                VOManager.Instance.ProcessFrame(_grayBuffer, _webCamTexture.width, _webCamTexture.height, 1, ts);
            }
        }

        // Optimized conversion loop (unsafe) or standard logic? 
        // Keeping it safe but clean.
        private static void ProcessGrayscale(Color32[] input, byte[] output)
        {
            int len = input.Length;
            for (int i = 0; i < len; i++)
            {
                Color32 c = input[i];
                // Integer math is faster than float: Y = (77*R + 150*G + 29*B) >> 8
                output[i] = (byte)((77 * c.r + 150 * c.g + 29 * c.b) >> 8);
            }
        }

        private void AdjustPreviewOrientation()
        {
            if (previewImage == null) return;

            int angle = -_webCamTexture.videoRotationAngle;
            var fitter = previewImage.GetComponent<AspectRatioFitter>();
            
            if (fitter != null)
            {
                fitter.aspectMode = AspectRatioFitter.AspectMode.FitInParent;
                float ratio = (float)_webCamTexture.width / _webCamTexture.height;
                
                // Swap ratio if rotated 90 deg
                if (Mathf.Abs(angle) == 90 || Mathf.Abs(angle) == 270) ratio = 1f / ratio;
                fitter.aspectRatio = ratio;
            }
            
            previewImage.rectTransform.localEulerAngles = new Vector3(0, 0, angle);
            
            // Mirror scale if front camera
            float scaleX = useFrontCamera ? -1 : 1;
            previewImage.rectTransform.localScale = new Vector3(scaleX, 1, 1);
        }

        private void InitializeVIO(int width, int height)
        {
            // Do not create if exists!
            if (VOManager.Instance == null)
            {
                 GameObject go = new GameObject("VOManager");
                 go.AddComponent<VOManager>();
            }

            // 使用 VOManager 中的标定参数（来自 PARAconfig.yaml），不再覆盖
            VOManager.Instance.Initialize();

            if (useNativeDebugImage && debugRenderer != null)
            {
                debugRenderer.Initialize(width, height);
            }
        }
    
    }
}
