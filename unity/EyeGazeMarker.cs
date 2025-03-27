using UnityEngine;
using System.Collections;
using Microsoft.MixedReality.Toolkit;
using Microsoft.MixedReality.Toolkit.Input;

public class EyeGazeMarker : MonoBehaviour
{
    [Header("注视与标记设置")]
    public float gazeThreshold = 2f;
    public float markerScaleDuration = 1f;
    public string targetMeshTag = "MapMesh";
    public string targetmarkTag = "FinalMarker";

    [Header("标记预制体")]
    public GameObject markerPrefab;

    private float gazeTimer = 0f;
    private float destroyGazeTimer = 0f;
    private Vector3 lastHitPoint;
    private GameObject lastHitMarker;

    void Update()
    {
        var eyeGazeProvider = CoreServices.InputSystem?.EyeGazeProvider;
        if (eyeGazeProvider == null || !eyeGazeProvider.IsEyeTrackingEnabled || eyeGazeProvider.GazeOrigin == Vector3.zero)
        {
            Debug.Log("眼动数据无效");
            ResetAllGaze();
            return;
        }

        Vector3 gazeOrigin = eyeGazeProvider.GazeOrigin;
        Vector3 gazeDirection = eyeGazeProvider.GazeDirection;

        if (Physics.Raycast(gazeOrigin, gazeDirection, out RaycastHit hit, Mathf.Infinity))
        {
            if (hit.collider.CompareTag(targetMeshTag))
            {
                HandleMeshGaze(hit);
            }
            else if (hit.collider.CompareTag(targetmarkTag))
            {
                HandleMarkerGaze(hit.collider.gameObject);
            }
            else
            {
                ResetAllGaze();
            }
        }
        else
        {
            Debug.Log("Raycast 未命中任何物体");
            ResetAllGaze();
        }
    }

    void HandleMeshGaze(RaycastHit hit)
    {
        ResetDestroyGaze(); // 重置销毁相关参数

        if (Vector3.Distance(hit.point, lastHitPoint) < 0.1f)
        {
            gazeTimer += Time.deltaTime;
        }
        else
        {
            gazeTimer = 0f;
        }
        lastHitPoint = hit.point;

        if (gazeTimer >= gazeThreshold)
        {
            PlaceMarker(hit.point);
            ResetGaze();
        }
    }

    void HandleMarkerGaze(GameObject marker)
    {
        ResetGaze(); // 重置生成相关参数

        if (lastHitMarker == marker)
        {
            destroyGazeTimer += Time.deltaTime;
        }
        else
        {
            destroyGazeTimer = 0f;
            lastHitMarker = marker;
        }

        if (destroyGazeTimer >= gazeThreshold)
        {
            Destroy(marker);
            ResetDestroyGaze();
        }
    }

    void ResetAllGaze()
    {
        ResetGaze();
        ResetDestroyGaze();
    }

    void ResetGaze()
    {
        gazeTimer = 0f;
        lastHitPoint = Vector3.zero;
    }

    void ResetDestroyGaze()
    {
        destroyGazeTimer = 0f;
        lastHitMarker = null;
    }

    void PlaceMarker(Vector3 position)
    {
        GameObject marker = Instantiate(markerPrefab, position, Quaternion.identity);
        marker.transform.localScale = Vector3.zero;
        StartCoroutine(ScaleMarker(marker, markerScaleDuration));
    }

    IEnumerator ScaleMarker(GameObject marker, float duration)
    {
        float elapsedTime = 0f;
        Vector3 targetScale = Vector3.one;
        while (elapsedTime < duration)
        {
            marker.transform.localScale = Vector3.Lerp(Vector3.zero, targetScale, elapsedTime / duration);
            elapsedTime += Time.deltaTime;
            yield return null;
        }
        marker.transform.localScale = targetScale;
    }
}