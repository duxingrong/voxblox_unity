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

    // 用于线程间传递接收到的消息
    private readonly List<string> messageQueue = new List<string>();
    private readonly object queueLock = new object();

    // 用于缓存创建的坐标系对象，避免重复创建
    private Dictionary<string, GameObject> tfObjects = new Dictionary<string, GameObject>();

    // 用于缓存收到数据的缓冲区
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
                // 按换行符分割收到的数据
                string[] messages = incomingDataBuffer.Split('\n');
                // 除了最后一个（可能不完整）之外，其他都是完整消息
                for (int i = 0; i < messages.Length - 1; i++)
                {
                    lock (queueLock)
                    {
                        messageQueue.Add(messages[i]);
                    }
                }
                // 将最后一个不完整的部分留到下次拼接
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
                // ROS 坐标系转换到 Unity 坐标系（此处简单采用 -y, z, x 的转换方法，视实际情况调整）
                Vector3 rosPos = new Vector3(tfData.translation.x, tfData.translation.y, tfData.translation.z);
                Vector3 unityPos = new Vector3(-rosPos.y, rosPos.z, rosPos.x);
                Quaternion rosRot = new Quaternion(tfData.rotation.x, tfData.rotation.y, tfData.rotation.z, tfData.rotation.w);
                Quaternion unityRot = new Quaternion(-rosRot.y, rosRot.z, rosRot.x, rosRot.w);

                // 判断是否已经存在此坐标系的可视化对象
                if (tfObjects.ContainsKey(tfData.frame_id))
                {
                    GameObject existingTfObject = tfObjects[tfData.frame_id];
                    existingTfObject.transform.position = unityPos;
                    existingTfObject.transform.rotation = unityRot;
                }
                else
                {
                    // 不存在则创建新的对象并加入字典
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

    // 创建一个直观的坐标系可视化模型，并返回创建的 GameObject
    GameObject CreateTfVisualization(string frameId, Vector3 pos, Quaternion rot)
    {
        // 父对象代表该坐标系
        GameObject tfObject = new GameObject("TF_" + frameId);
        tfObject.transform.position = pos;
        tfObject.transform.rotation = rot;

        // 创建三个轴，X轴（红色）、Y轴（绿色）、Z轴（蓝色）
        CreateAxis(tfObject.transform, Vector3.right, Color.red, "X");
        CreateAxis(tfObject.transform, Vector3.up, Color.green, "Y");
        CreateAxis(tfObject.transform, Vector3.forward, Color.blue, "Z");

        // 添加文本标签显示坐标系名称
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

    // 辅助方法：创建一个轴的可视化（使用 Cylinder）
    void CreateAxis(Transform parent, Vector3 axis, Color color, string axisName)
    {
        // 创建一个空对象作为轴的父对象
        GameObject axisObj = new GameObject(axisName + "-axis");
        axisObj.transform.SetParent(parent);
        axisObj.transform.localPosition = Vector3.zero;

        // 使用 Cylinder 作为轴，注意 Cylinder 默认沿 Y 轴延伸
        GameObject cylinder = GameObject.CreatePrimitive(PrimitiveType.Cylinder);
        cylinder.transform.SetParent(axisObj.transform);
        // 设定轴的长度与粗细
        float axisLength = 0.2f;
        float thickness = 0.02f;
        // 因为 Cylinder 的 pivot 在中心，所以 Y 方向需要缩放为半轴长，并上移半轴长
        cylinder.transform.localScale = new Vector3(thickness, axisLength / 2f, thickness);
        cylinder.transform.localPosition = new Vector3(0, axisLength / 2f, 0);
        cylinder.GetComponent<Renderer>().material.color = color;

        // 旋转空对象使得其 Y 轴对齐到指定方向
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
