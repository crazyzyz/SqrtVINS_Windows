using UnityEngine;
using UnityEngine.UI;

namespace SqrtVINS
{
    public class VOPoseDisplay : MonoBehaviour
    {
        [Header("显示设置")]
        public bool showPosition = true;
        public bool showRotation = true;
        public bool showEulerAngles = true;
        public int updateFrequency = 10;

        [Header("UI 样式")]
        public int fontSize = 14;
        public Color textColor = Color.white;
        public TextAnchor alignment = TextAnchor.UpperLeft;

        private Text _poseText;
        private float _updateInterval;
        private float _lastUpdateTime;

        private void Start()
        {
            _updateInterval = 1f / updateFrequency;
            _lastUpdateTime = Time.time;

            CreatePoseTextUI();
        }

        private void CreatePoseTextUI()
        {
            // 查找 Canvas
            Canvas canvas = FindObjectOfType<Canvas>();
            if (canvas == null)
            {
                Debug.LogError("[VOPoseDisplay] 找不到 Canvas！");
                return;
            }

            // 创建 Text GameObject
            GameObject textGo = new GameObject("VOPoseText");
            textGo.transform.SetParent(canvas.transform, false);

            // 添加 RectTransform
            RectTransform rectTransform = textGo.AddComponent<RectTransform>();
            rectTransform.anchorMin = new Vector2(0, 1);
            rectTransform.anchorMax = new Vector2(0, 1);
            rectTransform.pivot = new Vector2(0, 1);
            rectTransform.anchoredPosition = new Vector2(10, -10);
            rectTransform.sizeDelta = new Vector2(300, 300);

            // 添加 Text 组件
            _poseText = textGo.AddComponent<Text>();
            _poseText.font = Resources.GetBuiltinResource<Font>("Arial.ttf");
            _poseText.fontSize = fontSize;
            _poseText.color = textColor;
            _poseText.alignment = alignment;
            _poseText.text = "Initializing...";

            Debug.Log("[VOPoseDisplay] UI Text 创建成功");
        }

        private void Update()
        {
            if (_poseText == null) return;

            if (Time.time - _lastUpdateTime < _updateInterval) return;
            _lastUpdateTime = Time.time;

            UpdatePoseDisplay();
        }

        private void UpdatePoseDisplay()
        {
            if (VOManager.Instance == null)
            {
                _poseText.text = "VOManager: Not Initialized";
                return;
            }

            bool isTracking = VOManager.Instance.IsTracking;
            Pose currentPose = VOManager.Instance.CurrentPose;

            System.Text.StringBuilder sb = new System.Text.StringBuilder();

            sb.AppendLine($"<b>VIO:</b> {(isTracking ? "<color=green>Tracking</color>" : "<color=red>Lost</color>")}");

            if (showPosition)
            {
                sb.AppendLine($"<b>Pos:</b> ({currentPose.position.x:F3}, {currentPose.position.y:F3}, {currentPose.position.z:F3})");
            }

            if (showRotation)
            {
                sb.AppendLine($"<b>Quat:</b> ({currentPose.rotation.x:F2}, {currentPose.rotation.y:F2}, {currentPose.rotation.z:F2}, {currentPose.rotation.w:F2})");
            }

            if (showEulerAngles)
            {
                Vector3 euler = currentPose.rotation.eulerAngles;
                sb.AppendLine($"<b>Euler:</b> ({euler.x:F1}°, {euler.y:F1}°, {euler.z:F1}°)");
            }

            _poseText.text = sb.ToString();
        }
    }
}
