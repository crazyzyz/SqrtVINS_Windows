using UnityEngine;
using UnityEngine.UI;

namespace SqrtVINS
{
    /// <summary>
    /// VIO 位姿可视化组件
    /// 将 VIO 输出的位姿应用到目标 Transform 上
    /// </summary>
    public class VOPoseVisualizer : MonoBehaviour
    {
        #region 配置

        [Header("目标对象")]
        [Tooltip("位姿将应用到此 Transform，留空则使用自身")]
        [SerializeField] private Transform targetTransform;

        [Header("位姿设置")]
        [SerializeField] private bool applyPosition = true;
        [SerializeField] private bool applyRotation = true;
        [SerializeField] private Vector3 positionOffset = Vector3.zero;
        [SerializeField] private Vector3 rotationOffset = Vector3.zero;

        [Header("平滑设置")]
        [SerializeField] private bool smoothing = true;
        [SerializeField] private float positionSmoothTime = 0.05f;
        [SerializeField] private float rotationSmoothTime = 0.05f;

        [Header("缩放设置")]
        [SerializeField] private float positionScale = 1f;

        [Header("调试显示")]
        [SerializeField] private bool showDebugInfo = true;
        [SerializeField] private Text debugText;

        #endregion

        #region 状态

        private Vector3 _currentPosition;
        private Quaternion _currentRotation;
        private Vector3 _velocity;
        private Pose _latestPose;
        private bool _hasValidPose = false;

        #endregion

        #region Unity 生命周期

        private void Awake()
        {
            if (targetTransform == null)
            {
                targetTransform = transform;
            }
        }

        private void OnEnable()
        {
            // 订阅位姿更新事件
            if (VOManager.Instance != null)
            {
                VOManager.Instance.OnPoseUpdated.AddListener(OnPoseUpdated);
                VOManager.Instance.OnStateChanged.AddListener(OnStateChanged);
            }
        }

        private void OnDisable()
        {
            // 取消订阅
            if (VOManager.Instance != null)
            {
                VOManager.Instance.OnPoseUpdated.RemoveListener(OnPoseUpdated);
                VOManager.Instance.OnStateChanged.RemoveListener(OnStateChanged);
            }
        }

        private void Update()
        {
            if (!_hasValidPose) return;

            // 计算目标位姿
            Vector3 targetPosition = _latestPose.position * positionScale + positionOffset;
            Quaternion targetRotation = _latestPose.rotation * Quaternion.Euler(rotationOffset);

            // 应用平滑
            if (smoothing)
            {
                if (applyPosition)
                {
                    _currentPosition = Vector3.SmoothDamp(_currentPosition, targetPosition, ref _velocity, positionSmoothTime);
                    targetTransform.localPosition = _currentPosition;
                }

                if (applyRotation)
                {
                    _currentRotation = Quaternion.Slerp(_currentRotation, targetRotation, Time.deltaTime / rotationSmoothTime);
                    targetTransform.localRotation = _currentRotation;
                }
            }
            else
            {
                if (applyPosition)
                {
                    targetTransform.localPosition = targetPosition;
                }

                if (applyRotation)
                {
                    targetTransform.localRotation = targetRotation;
                }
            }

            // 更新调试信息
            if (showDebugInfo)
            {
                UpdateDebugInfo();
            }
        }

        private void LateUpdate()
        {
            // 可以在这里添加额外的后处理
        }

        #endregion

        #region 公共方法

        /// <summary>
        /// 重置位姿到原点
        /// </summary>
        public void ResetPose()
        {
            _currentPosition = Vector3.zero;
            _currentRotation = Quaternion.identity;
            targetTransform.localPosition = _currentPosition;
            targetTransform.localRotation = _currentRotation;
            _hasValidPose = false;
        }

        /// <summary>
        /// 手动更新位姿
        /// </summary>
        public void SetPose(Pose pose)
        {
            _latestPose = pose;
            _hasValidPose = true;
        }

        #endregion

        #region 事件处理

        private void OnPoseUpdated(Pose pose)
        {
            _latestPose = pose;
            _hasValidPose = true;
        }

        private void OnStateChanged(VOManager.VOState state)
        {
            if (state == VOManager.VOState.Lost || state == VOManager.VOState.Error)
            {
                // 跟踪丢失时可以选择保持最后位姿或重置
                Debug.Log($"[VOPoseVisualizer] Tracking state: {state}");
            }
        }

        private void UpdateDebugInfo()
        {
            if (debugText == null) return;

            string stateStr = VOManager.Instance != null ? VOManager.Instance.CurrentState.ToString() : "N/A";
            
            debugText.text = $"VIO State: {stateStr}\n" +
                           $"Position: {_latestPose.position:F3}\n" +
                           $"Rotation: {_latestPose.rotation.eulerAngles:F1}\n" +
                           $"Tracking: {(VOManager.Instance?.IsTracking == true ? "Yes" : "No")}";
        }

        #endregion

        #region Gizmos

        private void OnDrawGizmos()
        {
            if (!_hasValidPose) return;

            // 绘制位姿坐标轴
            Gizmos.color = Color.red;
            Gizmos.DrawRay(transform.position, transform.right * 0.1f);
            Gizmos.color = Color.green;
            Gizmos.DrawRay(transform.position, transform.up * 0.1f);
            Gizmos.color = Color.blue;
            Gizmos.DrawRay(transform.position, transform.forward * 0.1f);
        }

        #endregion
    }
}
