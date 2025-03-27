using UnityEngine;
using System.Collections;

public class InitialCameraTransform : MonoBehaviour
{
    IEnumerator Start()
    {
        // 等待一帧，让 MRTK 完成初始化
        yield return null;

        // 设置局部位置为 (0, 5, -5)
        transform.localPosition = new Vector3(0, 3, -3);
        // 设置局部旋转，x轴45度，其余为0
        transform.localEulerAngles = new Vector3(10, 0, 0);
    }
}
