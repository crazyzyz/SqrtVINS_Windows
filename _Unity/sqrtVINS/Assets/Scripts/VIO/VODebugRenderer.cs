using System;
using System.Runtime.InteropServices;
using UnityEngine;
using UnityEngine.UI;

namespace SqrtVINS
{
    /// <summary>
    /// 显示带有特征点和光流绘制的调试图像
    /// 取代 WebCamTexture 显示，使用原生绘制的图像
    /// </summary>
    public class VODebugRenderer : MonoBehaviour
    {
        [Header("References")]
        [SerializeField] private RawImage targetImage;
        
        [Header("Settings")]
        [SerializeField] private bool drawFeaturePoints = true;
        [SerializeField] private bool drawOpticalFlow = true;
        [SerializeField] private bool useDebugImage = true;
        
        private Texture2D _debugTexture;
        private byte[] _imageBuffer;
        private GCHandle _bufferHandle;
        private bool _initialized = false;
        
        private int _imageWidth;
        private int _imageHeight;
        
        private void Start()
        {
            if (targetImage == null)
            {
                Debug.LogError("[VODebugRenderer] Target image not assigned!");
                enabled = false;
                return;
            }
        }
        
        private void OnDestroy()
        {
            if (_bufferHandle.IsAllocated)
            {
                _bufferHandle.Free();
            }
            
            if (_debugTexture != null)
            {
                Destroy(_debugTexture);
            }
        }
        
        /// <summary>
        /// 初始化调试渲染器
        /// </summary>
        public void Initialize(int width, int height)
        {
            _imageWidth = width;
            _imageHeight = height;
            
            // 创建纹理和缓冲区
            _debugTexture = new Texture2D(width, height, TextureFormat.RGBA32, false);
            _imageBuffer = new byte[width * height * 4];
            _bufferHandle = GCHandle.Alloc(_imageBuffer, GCHandleType.Pinned);
            
            _initialized = true;
            Debug.Log($"[VODebugRenderer] Initialized: {width}x{height}");
        }
        
        private void LateUpdate()
        {
            if (!_initialized || !useDebugImage) return;
            if (VOManager.Instance == null || VOManager.Instance.CurrentState != VOManager.VOState.Running) return;
            
            // 获取调试图像
            int result = VONative.vo_get_debug_image(
                _bufferHandle.AddrOfPinnedObject(),
                _imageWidth,
                _imageHeight,
                drawFeaturePoints ? 1 : 0,
                drawOpticalFlow ? 1 : 0
            );
            
            if (result == 0)
            {
                // 加载纹理
                _debugTexture.LoadRawTextureData(_imageBuffer);
                _debugTexture.Apply();
                
                // 更新显示
                if (targetImage != null && targetImage.texture != _debugTexture)
                {
                    targetImage.texture = _debugTexture;
                }
            }
        }
        
        /// <summary>
        /// 切换是否使用调试图像
        /// </summary>
        public void SetUseDebugImage(bool use)
        {
            useDebugImage = use;
        }
        
        /// <summary>
        /// 设置绘制选项
        /// </summary>
        public void SetDrawOptions(bool points, bool flow)
        {
            drawFeaturePoints = points;
            drawOpticalFlow = flow;
        }
    }
}
