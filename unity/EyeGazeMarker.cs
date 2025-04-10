using UnityEngine;
using System.Collections;
using Microsoft.MixedReality.Toolkit;
using Microsoft.MixedReality.Toolkit.Input;

public class EyeGazeMarker : MonoBehaviour
{
    [Header("注视与标记设置")]
    public float gazeThreshold = 2f;               // 注视时间阈值
    public float markerScaleDuration = 1f;         // 标记缩放动画时长
    public string targetMeshTag = "MapMesh";       // 地图网格标签
    public string targetMarkTag = "FinalMarker";   // 标记物体标签

    [Header("标记预制体")]
    public GameObject markerPrefab;                // 标记预制体

    [Header("射线设置")]
    public LayerMask targetLayers;                 // 射线检测的目标层
    public float maxRaycastDistance = 20f;         // 最大射线距离
    public bool showDebugRay = true;               // 是否显示调试射线
    public Color debugRayColor = Color.red;        // 调试射线颜色
    public float debugRayWidth = 0.001f;           // 调试射线宽度

    [Header("眼动点设置")]
    public float gazeDotDistance = 2f;             // 小白点与眼睛的距离

    [Header("调试设置")]
    public bool enableDebugLogs = true;            // 是否启用调试日志

    // 内部变量
    private float gazeTimer = 0f;
    private float destroyGazeTimer = 0f;
    private Vector3 lastHitPoint = Vector3.zero;
    private GameObject lastHitMarker;
    private LineRenderer debugLineRenderer;
    private GameObject eyeGazeDot;                // 小白点，用于显示眼动数据

    void Start()
    {
        // 如果没有指定目标层，则默认使用所有层
        if (targetLayers.value == 0)
        {
            targetLayers = ~0; // 所有层
            LogDebug("未指定目标层，使用所有层进行射线检测");
        }

        // 初始化眼动小白点
        InitializeEyeGazeDot();

        // 创建虚线渲染器（保留射线）
        if (showDebugRay)
        {
            InitializeDebugLineRenderer();
        }
    }

    // 初始化眼动小白点（始终显示在眼动数据指示的位置，无论是否击中物体）
    private void InitializeEyeGazeDot()
    {
        eyeGazeDot = GameObject.CreatePrimitive(PrimitiveType.Sphere);
        eyeGazeDot.name = "EyeGazeDot";
        eyeGazeDot.transform.localScale = new Vector3(0.02f, 0.02f, 0.02f);

        // 设置材质为白色
        Renderer renderer = eyeGazeDot.GetComponent<Renderer>();
        if (renderer != null)
        {
            Material dotMaterial = new Material(Shader.Find("Standard"));
            if (dotMaterial != null)
            {
                dotMaterial.color = Color.white;
                renderer.sharedMaterial = dotMaterial;
            }
        }

        // 禁用碰撞器，避免干扰射线检测
        Collider collider = eyeGazeDot.GetComponent<Collider>();
        if (collider != null)
        {
            collider.enabled = false;
        }
    }

    // 初始化射线调试线
    private void InitializeDebugLineRenderer()
    {
        GameObject lineObj = new GameObject("GazeDebugLine");
        lineObj.transform.parent = this.transform;

        debugLineRenderer = lineObj.AddComponent<LineRenderer>();
        debugLineRenderer.startWidth = debugRayWidth;
        debugLineRenderer.endWidth = debugRayWidth;
        debugLineRenderer.useWorldSpace = true;

        // 使用标准材质
        Material lineMaterial = new Material(Shader.Find("Standard"));
        if (lineMaterial != null)
        {
            lineMaterial.color = debugRayColor;
            debugLineRenderer.sharedMaterial = lineMaterial;
        }

        // 创建虚线效果 - 使用一系列点来模拟虚线
        UpdateDashedLine(Vector3.zero, Vector3.forward, maxRaycastDistance);
    }

    // 更新虚线
    private void UpdateDashedLine(Vector3 start, Vector3 direction, float distance)
    {
        if (debugLineRenderer == null)
            return;

        float dashLength = 0.05f;
        float gapLength = 0.05f;
        float step = dashLength + gapLength;
        int pointCount = Mathf.FloorToInt(distance / step) * 2;

        if (pointCount < 2)
            pointCount = 2; // 至少两个点

        debugLineRenderer.positionCount = pointCount;

        for (int i = 0; i < pointCount / 2; i++)
        {
            float dashStart = i * step;
            float dashEnd = dashStart + dashLength;

            if (dashStart > distance)
                dashStart = distance;
            if (dashEnd > distance)
                dashEnd = distance;

            debugLineRenderer.SetPosition(i * 2, start + direction * dashStart);
            debugLineRenderer.SetPosition(i * 2 + 1, start + direction * dashEnd);
        }
    }

    void Update()
    {
        // 获取眼动追踪数据
        var eyeGazeProvider = CoreServices.InputSystem?.EyeGazeProvider;

        if (eyeGazeProvider == null || !eyeGazeProvider.IsEyeTrackingEnabled)
        {
            LogDebug("眼动追踪不可用或未启用");
            ResetAllGaze();

            // 如果眼动不可用，则隐藏小白点
            if (eyeGazeDot != null)
            {
                eyeGazeDot.SetActive(false);
            }

            if (debugLineRenderer != null)
            {
                debugLineRenderer.enabled = false;
            }

            return;
        }

        // 获取视线原点和方向
        Vector3 gazeOrigin = eyeGazeProvider.GazeOrigin;
        Vector3 gazeDirection = eyeGazeProvider.GazeDirection;

        if (gazeOrigin == Vector3.zero || gazeDirection == Vector3.zero)
        {
            LogDebug("眼动数据无效: 原点或方向为零向量");
            ResetAllGaze();
            return;
        }

        // 始终更新小白点位置，无论是否与物体发生碰撞
        if (eyeGazeDot != null)
        {
            Vector3 dotPosition = gazeOrigin + gazeDirection * gazeDotDistance;
            eyeGazeDot.transform.position = dotPosition;
            if (!eyeGazeDot.activeSelf)
                eyeGazeDot.SetActive(true);
        }

        // 绘制调试射线（保留）
        if (showDebugRay && debugLineRenderer != null)
        {
            debugLineRenderer.enabled = true;
            UpdateDashedLine(gazeOrigin, gazeDirection, maxRaycastDistance);
        }

        // 射线检测，用于原有标记功能
        if (Physics.Raycast(gazeOrigin, gazeDirection, out RaycastHit hit, maxRaycastDistance, targetLayers))
        {
            LogDebug($"射线命中: {hit.collider.gameObject.name}, 标签: {hit.collider.tag}, 距离: {hit.distance}");

            if (hit.collider.CompareTag(targetMeshTag))
            {
                HandleMeshGaze(hit);
            }
            else if (hit.collider.CompareTag(targetMarkTag))
            {
                HandleMarkerGaze(hit.collider.gameObject);
            }
            else
            {
                LogDebug($"命中物体的标签 '{hit.collider.tag}' 不匹配目标标签");
                ResetAllGaze();
            }
        }
        else
        {
            LogDebug("射线未命中任何物体");
            ResetAllGaze();
        }
    }

    // 处理注视MapMesh的逻辑：累计注视时间并放置标记预制体（不做修改）
    void HandleMeshGaze(RaycastHit hit)
    {
        ResetDestroyGaze();

        if (Vector3.Distance(hit.point, lastHitPoint) < 0.1f)
        {
            gazeTimer += Time.deltaTime;
            LogDebug($"注视累计时间: {gazeTimer:F2}/{gazeThreshold:F2}");
        }
        else
        {
            LogDebug("注视点改变，重置计时器");
            gazeTimer = 0f;
        }

        lastHitPoint = hit.point;

        if (gazeTimer >= gazeThreshold)
        {
            LogDebug("注视时间达到阈值，放置标记物");
            PlaceMarker(hit.point, hit.normal);
            ResetGaze();
        }
    }

    // 处理注视标记物的逻辑：累计注视时间并销毁标记预制体（不做修改）
    void HandleMarkerGaze(GameObject marker)
    {
        ResetGaze();

        if (lastHitMarker == marker)
        {
            destroyGazeTimer += Time.deltaTime;
            LogDebug($"注视标记物累计时间: {destroyGazeTimer:F2}/{gazeThreshold:F2}");
        }
        else
        {
            LogDebug("注视标记物改变，重置计时器");
            destroyGazeTimer = 0f;
            lastHitMarker = marker;
        }

        if (destroyGazeTimer >= gazeThreshold)
        {
            LogDebug("注视标记物时间达到阈值，销毁标记物");
            Destroy(marker);
            ResetDestroyGaze();
        }
    }

    // 重置所有注视相关参数
    void ResetAllGaze()
    {
        ResetGaze();
        ResetDestroyGaze();
    }

    // 重置累计生成标记的计时器
    void ResetGaze()
    {
        gazeTimer = 0f;
        lastHitPoint = Vector3.zero;
    }

    // 重置累计销毁标记的计时器
    void ResetDestroyGaze()
    {
        destroyGazeTimer = 0f;
        lastHitMarker = null;
    }

    // 放置标记预制体（不做修改）
    void PlaceMarker(Vector3 position, Vector3 normal)
    {
        if (markerPrefab == null)
        {
            LogDebug("错误: 标记预制体未设置!");
            return;
        }

        Quaternion rotation = Quaternion.LookRotation(normal);
        GameObject marker = Instantiate(markerPrefab, position, rotation);
        marker.tag = targetMarkTag;
        marker.transform.localScale = Vector3.zero;
        StartCoroutine(ScaleMarker(marker, markerScaleDuration));
    }

    // 标记预制体的缩放动画（不做修改）
    IEnumerator ScaleMarker(GameObject marker, float duration)
    {
        if (marker == null)
            yield break;

        float elapsedTime = 0f;
        Vector3 targetScale = new Vector3(0.3f, 0.3f, 0.3f);

        while (elapsedTime < duration && marker != null)
        {
            marker.transform.localScale = Vector3.Lerp(Vector3.zero, targetScale, elapsedTime / duration);
            elapsedTime += Time.deltaTime;
            yield return null;
        }

        if (marker != null)
        {
            marker.transform.localScale = targetScale;
            LogDebug("标记物缩放完成");
        }
    }

    // 调试日志输出
    void LogDebug(string message)
    {
        if (enableDebugLogs)
        {
            Debug.Log($"[EyeGazeMarker] {message}");
        }
    }

    // 在编辑器中绘制调试射线和小白点（用于可视化调试）
    void OnDrawGizmos()
    {
        if (Application.isPlaying)
        {
            var eyeGazeProvider = CoreServices.InputSystem?.EyeGazeProvider;
            if (eyeGazeProvider != null && eyeGazeProvider.IsEyeTrackingEnabled)
            {
                Vector3 start = eyeGazeProvider.GazeOrigin;
                Vector3 direction = eyeGazeProvider.GazeDirection;
                float dashLength = 0.05f;
                float gapLength = 0.05f;
                float currentLength = 0f;

                while (currentLength < maxRaycastDistance)
                {
                    Vector3 dashStart = start + direction * currentLength;
                    float remainingLength = maxRaycastDistance - currentLength;
                    float currentDashLength = Mathf.Min(dashLength, remainingLength);

                    if (currentDashLength > 0)
                    {
                        Vector3 dashEnd = dashStart + direction * currentDashLength;
                        Gizmos.color = debugRayColor;
                        Gizmos.DrawLine(dashStart, dashEnd);
                    }

                    currentLength += dashLength + gapLength;
                }

                // 绘制小白点的位置（仅用于可视化调试，与实际eyeGazeDot同步）
                Gizmos.color = Color.white;
                Gizmos.DrawSphere(start + direction * gazeDotDistance, 0.01f);
            }
        }
    }

    // 清理内部创建的资源
    void OnDestroy()
    {
        if (eyeGazeDot != null)
        {
            Destroy(eyeGazeDot);
        }
    }
}
