using UnityEngine;
using System.Collections.Generic;
using System.Net;
using System.Net.Sockets;
using System.Text;
using System.Threading;
using System;

[Serializable]
public class TfData
{
    public string frame_id;
    public Translation translation;
    public Rotation rotation;
}

[Serializable]
public class Translation
{
    public float x;
    public float y;
    public float z;
}

[Serializable]
public class Rotation
{
    public float x;
    public float y;
    public float z;
    public float w;
}

public class TfReceiver : MonoBehaviour
{
    public int port = 9000;
    private TcpListener server;
    private Thread listenerThread;
    private bool isRunning = false;

    // �����̼߳䴫�ݽ��յ�����Ϣ
    private readonly List<string> messageQueue = new List<string>();
    private readonly object queueLock = new object();

    // ���ڻ��洴��������ϵ���󣬱����ظ�����
    private Dictionary<string, GameObject> tfObjects = new Dictionary<string, GameObject>();

    // ���ڻ����յ����ݵĻ�����
    private string incomingDataBuffer = "";

    void Start()
    {
        StartServer();
    }

    void StartServer()
    {
        server = new TcpListener(IPAddress.Any, port);
        server.Start();
        isRunning = true;
        listenerThread = new Thread(ListenForClients);
        listenerThread.IsBackground = true;
        listenerThread.Start();
        Debug.Log("TCP Server started on port " + port);
    }

    void ListenForClients()
    {
        try
        {
            while (isRunning)
            {
                TcpClient client = server.AcceptTcpClient();
                Debug.Log("Client connected.");
                Thread clientThread = new Thread(() => HandleClient(client));
                clientThread.IsBackground = true;
                clientThread.Start();
            }
        }
        catch (Exception e)
        {
            Debug.LogError("ListenForClients error: " + e.Message);
        }
    }

    void HandleClient(TcpClient client)
    {
        try
        {
            NetworkStream stream = client.GetStream();
            byte[] buffer = new byte[1024];
            int bytesRead = 0;
            while (isRunning && (bytesRead = stream.Read(buffer, 0, buffer.Length)) != 0)
            {
                incomingDataBuffer += Encoding.UTF8.GetString(buffer, 0, bytesRead);
                // �����з��ָ��յ�������
                string[] messages = incomingDataBuffer.Split('\n');
                // �������һ�������ܲ�������֮�⣬��������������Ϣ
                for (int i = 0; i < messages.Length - 1; i++)
                {
                    lock (queueLock)
                    {
                        messageQueue.Add(messages[i]);
                    }
                }
                // �����һ���������Ĳ��������´�ƴ��
                incomingDataBuffer = messages[messages.Length - 1];
            }
            client.Close();
        }
        catch (Exception e)
        {
            Debug.LogError("HandleClient error: " + e.Message);
        }
    }

    void Update()
    {
        lock (queueLock)
        {
            for (int i = 0; i < messageQueue.Count; i++)
            {
                ProcessMessage(messageQueue[i]);
            }
            messageQueue.Clear();
        }
    }

    void ProcessMessage(string jsonData)
    {
        try
        {
            TfData tfData = JsonUtility.FromJson<TfData>(jsonData);
            if (tfData != null)
            {
                Debug.Log("Processing TF data for frame: " + tfData.frame_id);
                // ROS ����ϵת���� Unity ����ϵ���˴��򵥲��� -y, z, x ��ת����������ʵ�����������
                Vector3 rosPos = new Vector3(tfData.translation.x, tfData.translation.y, tfData.translation.z);
                Vector3 unityPos = new Vector3(-rosPos.y, rosPos.z, rosPos.x);
                Quaternion rosRot = new Quaternion(tfData.rotation.x, tfData.rotation.y, tfData.rotation.z, tfData.rotation.w);
                Quaternion unityRot = new Quaternion(-rosRot.y, rosRot.z, rosRot.x, rosRot.w);

                // �ж��Ƿ��Ѿ����ڴ�����ϵ�Ŀ��ӻ�����
                if (tfObjects.ContainsKey(tfData.frame_id))
                {
                    GameObject existingTfObject = tfObjects[tfData.frame_id];
                    existingTfObject.transform.position = unityPos;
                    existingTfObject.transform.rotation = unityRot;
                }
                else
                {
                    // �������򴴽��µĶ��󲢼����ֵ�
                    GameObject newTfObject = CreateTfVisualization(tfData.frame_id, unityPos, unityRot);
                    tfObjects.Add(tfData.frame_id, newTfObject);
                }
            }
        }
        catch (Exception e)
        {
            Debug.LogError("ProcessMessage error: " + e.Message);
        }
    }

    // ����һ��ֱ�۵�����ϵ���ӻ�ģ�ͣ������ش����� GameObject
    GameObject CreateTfVisualization(string frameId, Vector3 pos, Quaternion rot)
    {
        // ��������������ϵ
        GameObject tfObject = new GameObject("TF_" + frameId);
        tfObject.transform.position = pos;
        tfObject.transform.rotation = rot;

        // ���������ᣬX�ᣨ��ɫ����Y�ᣨ��ɫ����Z�ᣨ��ɫ��
        CreateAxis(tfObject.transform, Vector3.right, Color.red, "X");
        CreateAxis(tfObject.transform, Vector3.up, Color.green, "Y");
        CreateAxis(tfObject.transform, Vector3.forward, Color.blue, "Z");

        // ����ı���ǩ��ʾ����ϵ����
        GameObject labelObj = new GameObject("Label");
        labelObj.transform.SetParent(tfObject.transform);
        labelObj.transform.localPosition = new Vector3(0, 0.2f, 0);
        TextMesh textMesh = labelObj.AddComponent<TextMesh>();
        textMesh.text = frameId;
        textMesh.fontSize = 10;
        textMesh.characterSize = 0.05f;
        textMesh.color = Color.white;

        return tfObject;
    }

    // ��������������һ����Ŀ��ӻ���ʹ�� Cylinder��
    void CreateAxis(Transform parent, Vector3 axis, Color color, string axisName)
    {
        // ����һ���ն�����Ϊ��ĸ�����
        GameObject axisObj = new GameObject(axisName + "-axis");
        axisObj.transform.SetParent(parent);
        axisObj.transform.localPosition = Vector3.zero;

        // ʹ�� Cylinder ��Ϊ�ᣬע�� Cylinder Ĭ���� Y ������
        GameObject cylinder = GameObject.CreatePrimitive(PrimitiveType.Cylinder);
        cylinder.transform.SetParent(axisObj.transform);
        // �趨��ĳ������ϸ
        float axisLength = 0.2f;
        float thickness = 0.02f;
        // ��Ϊ Cylinder �� pivot �����ģ����� Y ������Ҫ����Ϊ���᳤�������ư��᳤
        cylinder.transform.localScale = new Vector3(thickness, axisLength / 2f, thickness);
        cylinder.transform.localPosition = new Vector3(0, axisLength / 2f, 0);
        cylinder.GetComponent<Renderer>().material.color = color;

        // ��ת�ն���ʹ���� Y ����뵽ָ������
        axisObj.transform.rotation = Quaternion.FromToRotation(Vector3.up, axis);
    }

    void OnApplicationQuit()
    {
        isRunning = false;
        if (server != null)
            server.Stop();
        if (listenerThread != null && listenerThread.IsAlive)
            listenerThread.Abort();
    }
}
