using UnityEngine;
using System.Collections;

public class InitialCameraTransform : MonoBehaviour
{
    IEnumerator Start()
    {
        // �ȴ�һ֡���� MRTK ��ɳ�ʼ��
        yield return null;

        // ���þֲ�λ��Ϊ (0, 5, -5)
        transform.localPosition = new Vector3(0, 3, -3);
        // ���þֲ���ת��x��45�ȣ�����Ϊ0
        transform.localEulerAngles = new Vector3(10, 0, 0);
    }
}
