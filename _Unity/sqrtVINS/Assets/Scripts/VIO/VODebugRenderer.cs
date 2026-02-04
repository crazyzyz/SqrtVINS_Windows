using System;
using System.Collections;
using UnityEngine;
using UnityEngine.UI;

namespace SqrtVINS
{
    /// <summary>
    /// VIO 调试图像渲染器
    /// 优化方案2：使用 Native Texture Rendering Plugin
    /// 直接在原生层更新 OpenGL 纹理，避免 CPU-GPU 数据传输
    /// </summary>
    public class VODebugRenderer : MonoBehaviour
    {
        [Header("UI 组件")]
        [SerializeField] private RawImage debugImage;
        [SerializeField] private AspectRatioFitter aspectRatioFitter;

        [Header("设置")]
        [SerializeField] private bool useDebugImage = true;

        private Texture2D _debugTexture;
        private bool _initialized = false;
        private int _imageWidth;
        private int _imageHeight;

        private void OnDestroy()
        {
            if (_debugTexture != null)
            {
                Destroy(_debugTexture);
                _debugTexture = null;
            }
        }

        public void Initialize(int width, int height)
        {
            if (_initialized)
            {
                // 如果尺寸变了，重新初始化
                if (_imageWidth != width || _imageHeight != height)
                {
                    if (_debugTexture != null) Destroy(_debugTexture);
                    _initialized = false;
                }
                else
                {
                    return;
                }
            }

            _imageWidth = width;
            _imageHeight = height;

            try
            {
                // 创建 Texture2D
                // TextureFormat.RGBA32 对应 GL_RGBA + GL_UNSIGNED_BYTE
                _debugTexture = new Texture2D(width, height, TextureFormat.RGBA32, false);
                _debugTexture.filterMode = FilterMode.Point; // 像素风，或者 Bilinear
                _debugTexture.wrapMode = TextureWrapMode.Clamp;
                
                // 关键：初始化颜色并 Upload 到 GPU，确保显存分配
                // 填充不透明黑色，以便区分背景
                Color32[] colors = new Color32[width * height];
                for (int i = 0; i < colors.Length; i++) colors[i] = new Color32(0, 0, 0, 255);
                _debugTexture.SetPixels32(colors);
                _debugTexture.Apply(); // 必须调用，否则从 Native update 可能无效


                if (debugImage != null)
                {
                    debugImage.texture = _debugTexture;
                    debugImage.gameObject.SetActive(true);
                }

                if (aspectRatioFitter != null)
                {
                    aspectRatioFitter.aspectRatio = (float)width / height;
                }

                // 关键：获取 Native Texture Pointer 并传递给 C++
                IntPtr nativePtr = _debugTexture.GetNativeTexturePtr();
                Debug.Log($"[VODebugRenderer] Initialized with native texture ptr: {nativePtr}");
                
                int result = VONative.vo_set_native_texture(nativePtr, width, height);
                if (result != 0)
                {
                    Debug.LogError($"[VODebugRenderer] Failed to set native texture: {result}");
                }

                _initialized = true;
                
                // 启动渲染循环
                StopAllCoroutines();
                StartCoroutine(RenderLoop());
            }
            catch (Exception e)
            {
                Debug.LogError($"[VODebugRenderer] Init failed: {e.Message}");
            }
        }

        private IEnumerator RenderLoop()
        {
            while (true)
            {
                // 必须在 WaitForEndOfFrame 之后调用 GL.IssuePluginEvent
                // 以确保在渲染线程执行
                yield return new WaitForEndOfFrame();

                if (_initialized && useDebugImage)
                {
                    // 获取回调函数指针
                    IntPtr callback = VONative.vo_get_render_event_func();
                    if (callback != IntPtr.Zero)
                    {
                        // 触发事件 ID 1 (我们在 C++ 里定义 ID 1 为更新纹理)
                        GL.IssuePluginEvent(callback, 1);
                    }
                }
            }
        }
    }
}
