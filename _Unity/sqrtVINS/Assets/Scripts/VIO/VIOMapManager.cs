using UnityEngine;
using UnityEngine.UI;
using System.Collections.Generic;

namespace SqrtVINS
{

    public class VIOMapManager : MonoBehaviour
    {
        [Header("UI 引用")]
        public RawImage mapPreviewUI;
        
        [Header("轨迹设置")]
        public float minDistanceBetweenPoints = 0.05f; // 采样间距（米）
        public Color trajectoryColor = Color.cyan;
        public float trajectoryWidth = 0.02f;
        
        [Header("地图相机设置")]
        public float cameraHeight = 5f; // 相机俯视高度
        public float orthographicSize = 5f; // 初始视野范围
        public bool followAnchor = true; // 相机是否跟随锚点核心

        private GameObject _anchorInstance;
        private LineRenderer _pathRenderer;
        private Camera _mapCamera;

        private RenderTexture _mapTexture;
        private List<Vector3> _pathPoints = new List<Vector3>();
        private Vector3 _lastPoint;

        private void Start()
        {
            Debug.Log("[VIOMapManager] Start called. Initializing map system...");

            // 1. 初始化 AnchorObject
            GameObject prefab = Resources.Load<GameObject>("AnchorObject");
            if (prefab != null)
            {
                _anchorInstance = Instantiate(prefab);
                _anchorInstance.name = "VIO_Anchor_Follower";
                Debug.Log($"[VIOMapManager] AnchorObject instantiated successfully from Resources. Position: {_anchorInstance.transform.position}");
            }
            else
            {
                Debug.LogWarning("[VIOMapManager] Failed to load 'AnchorObject' from Resources. Creating temporary replacement.");
                _anchorInstance = GameObject.CreatePrimitive(PrimitiveType.Cube);
                _anchorInstance.name = "VIO_Anchor_Follower_Temp";
                _anchorInstance.transform.localScale = Vector3.one * 0.1f;
            }

            // 2. 初始化路径渲染器
            GameObject pathGo = new GameObject("VIO_Path_Renderer");
            _pathRenderer = pathGo.AddComponent<LineRenderer>();
            _pathRenderer.startWidth = _pathRenderer.endWidth = trajectoryWidth;
            _pathRenderer.material = new Material(Shader.Find("Sprites/Default"));
            _pathRenderer.startColor = _pathRenderer.endColor = trajectoryColor;
            _pathRenderer.positionCount = 0;
            _pathRenderer.useWorldSpace = true; // 确保使用世界坐标

            // 3. 配置地图相机
            SetupMapCamera();
            
            // 立即对齐相机一次
            if (_anchorInstance != null && _mapCamera != null)
            {
                _mapCamera.transform.position = new Vector3(
                    _anchorInstance.transform.position.x,
                    cameraHeight,
                    _anchorInstance.transform.position.z
                );
            }
            
            _lastPoint = Vector3.zero;
        }

        private void SetupMapCamera()
        {
            GameObject camGo = new GameObject("VIO_Map_Camera");
            _mapCamera = camGo.AddComponent<Camera>();
            _mapCamera.orthographic = true;
            _mapCamera.orthographicSize = orthographicSize;
            _mapCamera.transform.rotation = Quaternion.Euler(90, 0, 0); // 俯视
            _mapCamera.clearFlags = CameraClearFlags.SolidColor;
            _mapCamera.backgroundColor = new Color(0.1f, 0.1f, 0.1f, 0.5f); // 暗色透明背景
            
            // 创建 RenderTexture
            _mapTexture = new RenderTexture(512, 512, 16);
            _mapCamera.targetTexture = _mapTexture;
            
            // 关联 UI
            if (mapPreviewUI != null)
            {
                mapPreviewUI.texture = _mapTexture;
            }
            else
            {
                Debug.LogError("[VIOMapManager] 请分配 MapPreview (RawImage) 引用！");
            }
        }

        private void Update()
        {
            if (VOManager.Instance == null || !VOManager.Instance.IsTracking) return;

            // 1. 同步位姿
            Pose currentPose = VOManager.Instance.CurrentPose;
            if (_anchorInstance != null)
            {
                _anchorInstance.transform.position = currentPose.position;
                _anchorInstance.transform.rotation = currentPose.rotation;
            }

            // 2. 轨迹处理
            UpdateTrajectory(currentPose.position);

            // 3. 地图相机跟随 (XZ 平面)
            if (followAnchor && _mapCamera != null)
            {
                _mapCamera.transform.position = new Vector3(
                    currentPose.position.x, 
                    cameraHeight, 
                    currentPose.position.z
                );
            }
        }

        private void UpdateTrajectory(Vector3 currentPos)
        {
            // 只有当移动距离超过阈值时才添加点（滤波）
            if (_pathPoints.Count == 0 || Vector3.Distance(currentPos, _lastPoint) >= minDistanceBetweenPoints)
            {
                AddPoint(currentPos);
            }
        }

        private void AddPoint(Vector3 point)
        {
            _pathPoints.Add(point);
            _lastPoint = point;
            
            _pathRenderer.positionCount = _pathPoints.Count;
            _pathRenderer.SetPosition(_pathPoints.Count - 1, point);
        }

        public void ClearMap()
        {
            _pathPoints.Clear();
            _pathRenderer.positionCount = 0;
            Debug.Log("[VIOMapManager] Map Cleared");
        }

        private void OnDestroy()
        {
            if (_mapTexture != null) _mapTexture.Release();
        }
    }
}
