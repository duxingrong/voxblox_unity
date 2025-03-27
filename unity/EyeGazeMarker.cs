using UnityEngine;
using System.Collections;
using Microsoft.MixedReality.Toolkit;
using Microsoft.MixedReality.Toolkit.Input;

public class EyeGazeMarker : MonoBehaviour
{
    [Header("ע����������")]
    public float gazeThreshold = 2f;
    public float markerScaleDuration = 1f;
    public string targetMeshTag = "MapMesh";

    [Header("���Ԥ����")]
    public GameObject markerPrefab;

    private float gazeTimer = 0f;
    private Vector3 lastHitPoint;

    void Update()
    {
        var eyeGazeProvider = CoreServices.InputSystem?.EyeGazeProvider;
        if (eyeGazeProvider == null || !eyeGazeProvider.IsEyeTrackingEnabled || eyeGazeProvider.GazeOrigin == Vector3.zero)
        {
            Debug.Log("�۶�������Ч");
            ResetGaze();
            return;
        }

        Vector3 gazeOrigin = eyeGazeProvider.GazeOrigin;
        Vector3 gazeDirection = eyeGazeProvider.GazeDirection;
        Debug.Log($"GazeOrigin: {gazeOrigin}, GazeDirection: {gazeDirection}");

        if (Physics.Raycast(gazeOrigin, gazeDirection, out RaycastHit hit, Mathf.Infinity))
        {
            Debug.Log($"Raycast hit: {hit.collider.name} at {hit.point}");
            if (hit.collider.CompareTag(targetMeshTag))
            {
                if (Vector3.Distance(hit.point, lastHitPoint) < 0.1f)
                {
                    gazeTimer += Time.deltaTime;
                }
                else
                {
                    gazeTimer = 0f;
                }
                lastHitPoint = hit.point;

                Debug.Log($"gazeTimer: {gazeTimer}");

                if (gazeTimer >= gazeThreshold)
                {
                    PlaceMarker(hit.point);
                    Debug.Log("���ɱ��");
                    ResetGaze();
                }
            }
            else
            {
                ResetGaze();
            }
        }
        else
        {
            Debug.Log("Raycast δ�����κ�����");
            ResetGaze();
        }
    }


    void ResetGaze()
    {
        gazeTimer = 0f;
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
