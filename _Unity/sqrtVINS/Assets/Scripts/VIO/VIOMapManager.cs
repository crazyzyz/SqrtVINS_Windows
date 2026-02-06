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
        public float minDistanceBetweenPoints = 0.02f;
        public int maxTrajectoryPoints = 500; // 最多保留500个点
        public Color trajectoryStartColor = new Color(0.2f, 0.5f, 0.5f, 1f); // 历史轨迹（暗青色）
        public Color trajectoryEndColor = Color.cyan; // 当前轨迹（亮青色）

        [Header("当前位置标记")]
        public Color currentPosColor = Color.yellow;
        public float markerSize = 0.15f;

        [Header("方向指示器")]
        public Color directionColor = new Color(1f, 0.5f, 0f, 1f); // 橙色
        public float arrowLength = 0.3f;

        [Header("地图相机设置")]
        public float cameraHeight = 50f;
        public float fixedOrthoSize = 2f; // 固定视野（米）
        public bool followCurrentPos = true;
        public int mapLayer = 8; // 地图专用 Layer

        private GameObject _anchorInstance;
        private LineRenderer _pathRenderer;
        private LineRenderer _currentPosMarker;
        private LineRenderer _directionArrow;
        private Camera _mapCamera;

        private RenderTexture _mapTexture;
        private List<Vector3> _pathPoints = new List<Vector3>();
        private Vector3 _lastPoint;
        private Quaternion _currentRotation;

        private void Start()
        {
            Debug.Log("[VIOMapManager] Start called. Initializing map system...");

            InitializeAnchor();
            SetupPathRenderer();
            SetupCurrentPosMarker();
            SetupDirectionArrow();
            SetupMapCamera();

            _lastPoint = Vector3.zero;
        }

        private void InitializeAnchor()
        {
            GameObject prefab = Resources.Load<GameObject>("AnchorObject");
            if (prefab != null)
            {
                _anchorInstance = Instantiate(prefab);
                _anchorInstance.name = "VIO_Anchor_Follower";
            }
            else
            {
                _anchorInstance = GameObject.CreatePrimitive(PrimitiveType.Cube);
                _anchorInstance.name = "VIO_Anchor_Follower_Temp";
                _anchorInstance.transform.localScale = Vector3.one * 0.1f;
            }
        }

        private void SetupPathRenderer()
        {
            GameObject pathGo = new GameObject("VIO_Path_Renderer");
            pathGo.layer = mapLayer; // 设置到地图专用 Layer
            _pathRenderer = pathGo.AddComponent<LineRenderer>();
            _pathRenderer.material = new Material(Shader.Find("Sprites/Default"));
            _pathRenderer.colorGradient = CreateTrajectoryGradient();
            _pathRenderer.positionCount = 0;
            _pathRenderer.useWorldSpace = true;
            _pathRenderer.startWidth = _pathRenderer.endWidth = 0.08f;
        }

        private Gradient CreateTrajectoryGradient()
        {
            Gradient gradient = new Gradient();
            gradient.SetKeys(
                new GradientColorKey[] {
                    new GradientColorKey(trajectoryStartColor, 0f),
                    new GradientColorKey(trajectoryEndColor, 1f)
                },
                new GradientAlphaKey[] {
                    new GradientAlphaKey(1f, 0f),
                    new GradientAlphaKey(1f, 1f)
                }
            );
            return gradient;
        }

        private void SetupCurrentPosMarker()
        {
            GameObject markerGo = new GameObject("VIO_CurrentPos_Marker");
            markerGo.layer = mapLayer; // 设置到地图专用 Layer
            _currentPosMarker = markerGo.AddComponent<LineRenderer>();
            _currentPosMarker.material = new Material(Shader.Find("Sprites/Default"));
            _currentPosMarker.startColor = _currentPosMarker.endColor = currentPosColor;
            _currentPosMarker.useWorldSpace = true;
            _currentPosMarker.positionCount = 5;
            _currentPosMarker.startWidth = _currentPosMarker.endWidth = 0.06f;
        }

        private void SetupDirectionArrow()
        {
            GameObject arrowGo = new GameObject("VIO_Direction_Arrow");
            arrowGo.layer = mapLayer; // 设置到地图专用 Layer
            _directionArrow = arrowGo.AddComponent<LineRenderer>();
            _directionArrow.material = new Material(Shader.Find("Sprites/Default"));
            _directionArrow.startColor = _directionArrow.endColor = directionColor;
            _directionArrow.useWorldSpace = true;
            _directionArrow.positionCount = 4; // 箭头：线段 + 两个箭头翼
            _directionArrow.startWidth = _directionArrow.endWidth = 0.08f;
        }

        private void SetupMapCamera()
        {
            GameObject camGo = new GameObject("VIO_Map_Camera");
            _mapCamera = camGo.AddComponent<Camera>();
            _mapCamera.orthographic = true;
            _mapCamera.orthographicSize = fixedOrthoSize;
            _mapCamera.transform.rotation = Quaternion.Euler(90, 0, 0);
            _mapCamera.clearFlags = CameraClearFlags.SolidColor;
            _mapCamera.backgroundColor = new Color(0.05f, 0.05f, 0.1f, 1f);
            _mapCamera.nearClipPlane = 0.1f;
            _mapCamera.farClipPlane = cameraHeight * 2f;
            _mapCamera.cullingMask = 1 << mapLayer; // 只渲染地图专用 Layer

            _mapTexture = new RenderTexture(512, 512, 16);
            _mapCamera.targetTexture = _mapTexture;

            if (mapPreviewUI != null)
            {
                mapPreviewUI.texture = _mapTexture;
            }
            else
            {
                Debug.LogError("[VIOMapManager] 请分配 MapPreview (RawImage) 引用！");
            }
        }

        private int _debugFrameCount = 0;

        private void Update()
        {
            _debugFrameCount++;

            if (_debugFrameCount % 100 == 0)
            {
                bool hasInstance = VOManager.Instance != null;
                bool isTracking = hasInstance && VOManager.Instance.IsTracking;
                Debug.Log($"[VIOMapManager] Frame {_debugFrameCount}: IsTracking={isTracking}, PathPoints={_pathPoints.Count}");
            }

            if (VOManager.Instance == null || !VOManager.Instance.IsTracking) return;

            Pose currentPose = VOManager.Instance.CurrentPose;
            _currentRotation = currentPose.rotation;

            // 1. 同步锚点
            if (_anchorInstance != null)
            {
                _anchorInstance.transform.position = currentPose.position;
                _anchorInstance.transform.rotation = currentPose.rotation;
            }

            // 2. 更新轨迹
            UpdateTrajectory(currentPose.position);

            // 3. 更新当前位置标记
            UpdateCurrentPosMarker(currentPose.position);

            // 4. 更新方向箭头
            UpdateDirectionArrow(currentPose.position, currentPose.rotation);

            // 5. 相机跟随当前位置
            if (followCurrentPos && _mapCamera != null)
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
            if (_pathPoints.Count == 0 || Vector3.Distance(currentPos, _lastPoint) >= minDistanceBetweenPoints)
            {
                AddPoint(currentPos);
            }
        }

        private void AddPoint(Vector3 point)
        {
            float distanceFromLast = _pathPoints.Count > 0 ? Vector3.Distance(point, _lastPoint) : 0f;

            _pathPoints.Add(point);
            _lastPoint = point;

            // 限制轨迹点数量
            bool removed = false;
            if (_pathPoints.Count > maxTrajectoryPoints)
            {
                _pathPoints.RemoveAt(0);
                removed = true;
            }

            // 更新 LineRenderer
            _pathRenderer.positionCount = _pathPoints.Count;
            for (int i = 0; i < _pathPoints.Count; i++)
            {
                _pathRenderer.SetPosition(i, _pathPoints[i]);
            }

            // 日志：每10个点记录一次
            if (_pathPoints.Count % 10 == 0)
            {
                Debug.Log($"[VIOMapManager] AddPoint: Pos=({point.x:F3}, {point.y:F3}, {point.z:F3}), " +
                         $"Distance={distanceFromLast:F3}m, TotalPoints={_pathPoints.Count}, Removed={removed}");
            }
        }

        private void UpdateCurrentPosMarker(Vector3 pos)
        {
            if (_currentPosMarker == null) return;

            float y = cameraHeight - 1f;

            // 画菱形标记
            _currentPosMarker.SetPosition(0, pos + new Vector3(0, y, markerSize));
            _currentPosMarker.SetPosition(1, pos + new Vector3(markerSize, y, 0));
            _currentPosMarker.SetPosition(2, pos + new Vector3(0, y, -markerSize));
            _currentPosMarker.SetPosition(3, pos + new Vector3(-markerSize, y, 0));
            _currentPosMarker.SetPosition(4, pos + new Vector3(0, y, markerSize));
        }

        private void UpdateDirectionArrow(Vector3 pos, Quaternion rotation)
        {
            if (_directionArrow == null) return;

            float y = cameraHeight - 0.5f;

            // 计算前向方向（Unity 的 forward 是 Z 轴）
            Vector3 forward = rotation * Vector3.forward;
            Vector3 arrowEnd = pos + forward * arrowLength;

            // 箭头翼的方向
            Vector3 right = rotation * Vector3.right;
            float wingLength = arrowLength * 0.3f;

            // 设置箭头线段：起点 -> 终点 -> 左翼 -> 终点 -> 右翼
            _directionArrow.positionCount = 5;
            _directionArrow.SetPosition(0, pos + new Vector3(0, y, 0));
            _directionArrow.SetPosition(1, arrowEnd + new Vector3(0, y, 0));
            _directionArrow.SetPosition(2, arrowEnd - forward * wingLength + right * wingLength + new Vector3(0, y, 0));
            _directionArrow.SetPosition(3, arrowEnd + new Vector3(0, y, 0));
            _directionArrow.SetPosition(4, arrowEnd - forward * wingLength - right * wingLength + new Vector3(0, y, 0));
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

