using System.Collections.Generic;
using UnityEngine;
using UnityEngine.UI;

namespace SqrtVINS
{
    /// <summary>
    /// 在 UI 上绘制 VO 特征点和光流
    /// 需挂载在 Canvas 下
    /// </summary>
    public class VOFeatureVisualizer : Graphic
    {
        [Header("Visualization Settings")]
        [SerializeField] private Color featureColor = Color.red;
        [SerializeField] private float pointSize = 5f;
        [SerializeField] private bool showIds = false;
        
        [Header("References")]
        [SerializeField] private RawImage cameraPreview; // 用于计算坐标缩放

        private VONative.VOFeature[] _currentFeatures;
        private int _featureCount = 0;
        private Dictionary<int, Vector2> _prevPositions = new Dictionary<int, Vector2>();

        protected override void OnEnable()
        {
            base.OnEnable();
            if (VOManager.Instance != null)
            {
                VOManager.Instance.OnFeaturesUpdated += OnFeaturesUpdated;
            }
        }

        protected override void OnDisable()
        {
            base.OnDisable();
            if (VOManager.Instance != null)
            {
                VOManager.Instance.OnFeaturesUpdated -= OnFeaturesUpdated;
            }
        }

        private void OnFeaturesUpdated(VONative.VOFeature[] features, int count)
        {
            _currentFeatures = features;
            _featureCount = count;
            
            // 触发重绘
            SetVerticesDirty();
        }

        protected override void OnPopulateMesh(VertexHelper vh)
        {
            vh.Clear();

            if (_currentFeatures == null || _featureCount == 0 || cameraPreview == null)
            {
                return;
            }

            Rect pixelRect = GetPixelAdjustedRect();
            
            // 获取相机图像的原始分辨率
            float imgW = 640;
            float imgH = 480;
            
            if (cameraPreview.texture != null)
            {
                imgW = cameraPreview.texture.width;
                imgH = cameraPreview.texture.height;
            }

            // 计算缩放比例 (假设 RawImage 是 AspectFit 或 Fill)
            // 这里简化假设 RawImage 填满 RectTransform 且未拉伸失真
            // 实际上需要考虑 Aspect Ratio Fitter
            
            float scaleX = pixelRect.width / imgW;
            float scaleY = pixelRect.height / imgH;
            
            // 居中偏移
            float offsetX = -pixelRect.width / 2f;
            float offsetY = -pixelRect.height / 2f;

            // VO 坐标系可能是 (0,0) 在左上角，Y 向下？通常 OpenCV 是这样。
            // Unity UI 是 (0,0) 在中心，Y 向上。
            // 需要翻转 Y 轴。
            
            UIVertex vert = UIVertex.simpleVert;
            vert.color = featureColor;

            for (int i = 0; i < _featureCount; i++)
            {
                var f = _currentFeatures[i];
                
                // 坐标转换
                float uiX = f.x * scaleX + offsetX;
                float uiY = (imgH - f.y) * scaleY + offsetY; // Flip Y
                Vector2 currentPos = new Vector2(uiX, uiY);

                // 绘制光流线 (Green)
                if (_prevPositions.TryGetValue(f.id, out Vector2 prevPos))
                {
                    // 只有当位置移动距离不大时才画线（避免重置时的飞线）
                    if (Vector2.Distance(currentPos, prevPos) < pixelRect.width * 0.2f)
                    {
                        DrawLine(vh, prevPos, currentPos, 2f, Color.green);
                    }
                }
                
                // 更新历史位置
                if (f.status == 1) // Tracked
                {
                    _prevPositions[f.id] = currentPos;
                }
                else
                {
                    _prevPositions.Remove(f.id);
                }

                // 绘制特征点 (Red)
                DrawQuad(vh, currentPos, pointSize, vert);
            }
        }

        private void DrawLine(VertexHelper vh, Vector2 start, Vector2 end, float width, Color color)
        {
            UIVertex v = UIVertex.simpleVert;
            v.color = color;
            v.uv0 = Vector2.zero;

            Vector2 dir = (end - start).normalized;
            Vector2 normal = new Vector2(-dir.y, dir.x) * width * 0.5f;

            int idx = vh.currentVertCount;

            v.position = start - normal;
            vh.AddVert(v);

            v.position = start + normal;
            vh.AddVert(v);

            v.position = end + normal;
            vh.AddVert(v);

            v.position = end - normal;
            vh.AddVert(v);

            vh.AddTriangle(idx, idx + 1, idx + 2);
            vh.AddTriangle(idx + 2, idx + 3, idx);
        }

        private void DrawQuad(VertexHelper vh, Vector2 center, float size, UIVertex v)
        {
            // Reset color for point
            v.color = featureColor;
            float half = size * 0.5f;

            v.position = new Vector3(center.x - half, center.y - half);
            v.uv0 = new Vector2(0, 0);
            int idx = vh.currentVertCount;
            vh.AddVert(v);

            v.position = new Vector3(center.x - half, center.y + half);
            v.uv0 = new Vector2(0, 1);
            vh.AddVert(v);

            v.position = new Vector3(center.x + half, center.y + half);
            v.uv0 = new Vector2(1, 1);
            vh.AddVert(v);

            v.position = new Vector3(center.x + half, center.y - half);
            v.uv0 = new Vector2(1, 0);
            vh.AddVert(v);

            vh.AddTriangle(idx, idx + 1, idx + 2);
            vh.AddTriangle(idx + 2, idx + 3, idx);
        }
    }
}
