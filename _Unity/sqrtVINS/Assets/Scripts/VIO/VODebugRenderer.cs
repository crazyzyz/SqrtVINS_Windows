using System;
using System.Collections;
using UnityEngine;
using UnityEngine.UI;

namespace SqrtVINS
{
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
                _debugTexture = new Texture2D(width, height, TextureFormat.RGBA32, false);
                _debugTexture.filterMode = FilterMode.Point; 
                _debugTexture.wrapMode = TextureWrapMode.Clamp;
                
               
                Color32[] colors = new Color32[width * height];
                for (int i = 0; i < colors.Length; i++) colors[i] = new Color32(0, 0, 0, 255);
                _debugTexture.SetPixels32(colors);
                _debugTexture.Apply();


                if (debugImage != null)
                {
                    debugImage.texture = _debugTexture;
                    debugImage.gameObject.SetActive(true);
                }

                if (aspectRatioFitter != null)
                {
                    aspectRatioFitter.aspectRatio = (float)width / height;
                }

                IntPtr nativePtr = _debugTexture.GetNativeTexturePtr();
                Debug.Log($"[VODebugRenderer] Initialized with native texture ptr: {nativePtr}");
                
                int result = VONative.vo_set_native_texture(nativePtr, width, height);
                if (result != 0)
                {
                    Debug.LogError($"[VODebugRenderer] Failed to set native texture: {result}");
                }

                _initialized = true;
                
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
                
                yield return new WaitForEndOfFrame();

                if (_initialized && useDebugImage)
                {
                    
                    IntPtr callback = VONative.vo_get_render_event_func();
                    if (callback != IntPtr.Zero)
                    {
                        
                        GL.IssuePluginEvent(callback, 1);
                    }
                }
            }
        }
    }
}
