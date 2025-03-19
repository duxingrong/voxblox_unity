using System;
using System.Collections.Generic;
using System.Net;
using System.Net.Sockets;
using System.Threading;
using UnityEngine;

public class OffMeshServer : MonoBehaviour
{
    public int port = 12345; // 监听端口
    private TcpListener listener;
    private Thread listenerThread;
    private volatile bool isRunning = false;

    // 用于线程间传递 off 数据的队列
    private Queue<string> offDataQueue = new Queue<string>();
    private object queueLock = new object();

    void Start()
    {
        StartServer();
    }

    void OnDestroy()
    {
        StopServer();
    }

    /// <summary>
    /// 启动 TCP 服务器，监听来自 Python 客户端的连接
    /// </summary>
    void StartServer()
    {
        isRunning = true;
        listener = new TcpListener(IPAddress.Any, port);
        listener.Start();
        Debug.Log($"Server started on port {port}");

        listenerThread = new Thread(ListenForClients);
        listenerThread.IsBackground = true;
        listenerThread.Start();
    }

    void StopServer()
    {
        isRunning = false;
        if (listener != null)
        {
            listener.Stop();
        }
        if (listenerThread != null && listenerThread.IsAlive)
        {
            listenerThread.Join();
        }
    }

    /// <summary>
    /// 等待客户端连接，每当有客户端连接时，为其启动一个新的线程处理数据接收
    /// </summary>
    void ListenForClients()
    {
        while (isRunning)
        {
            try
            {
                TcpClient client = listener.AcceptTcpClient();
                Debug.Log("Client connected.");
                Thread clientThread = new Thread(() => HandleClient(client));
                clientThread.IsBackground = true;
                clientThread.Start();
            }
            catch (SocketException se)
            {
                Debug.Log("Socket exception: " + se.Message);
            }
        }
    }

    /// <summary>
    /// 处理单个客户端连接
    /// 协议：先接收 4 字节（大端序）数据长度，再接收 off 数据字符串
    /// </summary>
    /// <param name="client"></param>
    void HandleClient(TcpClient client)
    {
        NetworkStream stream = client.GetStream();
        try
        {
            while (isRunning && client.Connected)
            {
                // 接收前 4 字节，获取数据长度
                byte[] lengthBuffer = new byte[4];
                int readBytes = stream.Read(lengthBuffer, 0, 4);
                if (readBytes < 4)
                {
                    Debug.LogWarning("Client disconnected or error reading length.");
                    break;
                }
                // 处理大端序数据（如果系统为小端）
                if (BitConverter.IsLittleEndian)
                {
                    Array.Reverse(lengthBuffer);
                }
                int dataLength = BitConverter.ToInt32(lengthBuffer, 0);

                // 接收 off 数据内容
                byte[] dataBuffer = new byte[dataLength];
                int totalRead = 0;
                while (totalRead < dataLength)
                {
                    int bytes = stream.Read(dataBuffer, totalRead, dataLength - totalRead);
                    if (bytes == 0)
                    {
                        Debug.LogWarning("Client disconnected while reading data.");
                        break;
                    }
                    totalRead += bytes;
                }
                if (totalRead != dataLength)
                {
                    Debug.LogWarning("Did not receive the expected data length.");
                    break;
                }
                string offData = System.Text.Encoding.UTF8.GetString(dataBuffer);
                Debug.Log("Received off data.");

                // 将接收到的 off 数据放入队列，等待主线程处理更新 Mesh
                lock (queueLock)
                {
                    offDataQueue.Enqueue(offData);
                }
            }
        }
        catch (Exception ex)
        {
            Debug.LogError("Exception in client handling: " + ex.Message);
        }
        finally
        {
            client.Close();
        }
    }

    void Update()
    {
        // 主线程中检测队列是否有新 off 数据
        string offData = null;
        lock (queueLock)
        {
            if (offDataQueue.Count > 0)
            {
                offData = offDataQueue.Dequeue();
            }
        }
        if (!string.IsNullOrEmpty(offData))
        {
            Mesh mesh = LoadOffFromString(offData);
            if (mesh != null)
            {
                MeshFilter mf = GetComponent<MeshFilter>();
                if (mf == null)
                {
                    mf = gameObject.AddComponent<MeshFilter>();
                }
                mf.mesh = mesh;

                MeshRenderer mr = GetComponent<MeshRenderer>();
                if (mr == null)
                {
                    mr = gameObject.AddComponent<MeshRenderer>();
                }
                Shader shader = Shader.Find("Shader Graphs/VectorColor");
                if (shader == null)
                {
                    Debug.LogError("Shader Graphs/VectorColor not found.");
                }
                else
                {
                    mr.material = new Material(shader);
                }
            }
        }
    }

    /// <summary>
    /// 从 off 格式字符串解析 Mesh，与从文件读取逻辑类似
    /// </summary>
    /// <param name="offData"></param>
    /// <returns></returns>
    Mesh LoadOffFromString(string offData)
    {
        try
        {
            // 按行分割，去除空行
            string[] lines = offData.Split(new[] { "\r\n", "\r", "\n" }, StringSplitOptions.RemoveEmptyEntries);
            if (lines.Length < 2)
            {
                Debug.LogError("OFF data is empty or incomplete.");
                return null;
            }

            string header = lines[0].Trim();
            bool isOFF = header == "OFF";
            bool isNOFF = header == "NOFF";
            bool isNCOFF = header == "NCOFF";
            bool isCOFF = header == "COFF";

            if (!isOFF && !isNOFF && !isNCOFF && !isCOFF)
            {
                Debug.LogError("Unsupported OFF format: " + header);
                return null;
            }

            // 解析顶点数和面数
            string[] counts = lines[1].Split(new char[] { ' ', '\t' }, StringSplitOptions.RemoveEmptyEntries);
            int vertexCount = int.Parse(counts[0]);
            int faceCount = int.Parse(counts[1]);

            List<Vector3> vertices = new List<Vector3>();
            List<Vector3> vertexNormals = new List<Vector3>();
            List<Color> vertexColors = new List<Color>();

            int vertexDataStart = 2;
            for (int i = 0; i < vertexCount; i++)
            {
                string[] parts = lines[vertexDataStart + i].Split(new char[] { ' ', '\t' }, StringSplitOptions.RemoveEmptyEntries);
                if (isNCOFF)
                {
                    // NCOFF 格式：x y z r g b a nx ny nz
                    if (parts.Length < 10)
                    {
                        Debug.LogError("NCOFF vertex data error, expected 10 values, got " + parts.Length);
                        return null;
                    }
                    float x = float.Parse(parts[0]);
                    float y = float.Parse(parts[1]);
                    float z = float.Parse(parts[2]);
                    vertices.Add(ConvertRosToUnity(new Vector3(x, y, z)));

                    float r = float.Parse(parts[3]);
                    float g = float.Parse(parts[4]);
                    float b = float.Parse(parts[5]);
                    float a = float.Parse(parts[6]);
                    if (r > 1f || g > 1f || b > 1f || a > 1f)
                    {
                        r /= 255f; g /= 255f; b /= 255f; a /= 255f;
                    }
                    vertexColors.Add(new Color(r, g, b, a));

                    float nx = float.Parse(parts[7]);
                    float ny = float.Parse(parts[8]);
                    float nz = float.Parse(parts[9]);
                    vertexNormals.Add(ConvertRosToUnity(new Vector3(nx, ny, nz)));
                }
                else if (isCOFF)
                {
                    // COFF 格式：x y z r g b 
                    if (parts.Length < 6)
                    {
                        Debug.LogError("COFF vertex data error, expected 6 values, got " + parts.Length);
                        return null;
                    }
                    float x = float.Parse(parts[0]);
                    float y = float.Parse(parts[1]);
                    float z = float.Parse(parts[2]);
                    vertices.Add(ConvertRosToUnity(new Vector3(x, y, z)));

                    float r = float.Parse(parts[3]);
                    float g = float.Parse(parts[4]);
                    float b = float.Parse(parts[5]);
                    float a = 255;
                    if (r > 1f || g > 1f || b > 1f || a > 1f)
                    {
                        r /= 255f; g /= 255f; b /= 255f; a /= 255f;
                    }
                    vertexColors.Add(new Color(r, g, b, a));
                }
                else if (isNOFF)
                {
                    // NOFF 格式：x y z nx ny nz
                    if (parts.Length < 6)
                    {
                        Debug.LogError("NOFF vertex data error.");
                        return null;
                    }
                    float x = float.Parse(parts[0]);
                    float y = float.Parse(parts[1]);
                    float z = float.Parse(parts[2]);
                    vertices.Add(ConvertRosToUnity(new Vector3(x, y, z)));

                    float nx = float.Parse(parts[3]);
                    float ny = float.Parse(parts[4]);
                    float nz = float.Parse(parts[5]);
                    vertexNormals.Add(ConvertRosToUnity(new Vector3(nx, ny, nz)));
                }
                else // OFF 格式，只包含顶点坐标
                {
                    if (parts.Length < 3)
                    {
                        Debug.LogError("OFF vertex data error.");
                        return null;
                    }
                    float x = float.Parse(parts[0]);
                    float y = float.Parse(parts[1]);
                    float z = float.Parse(parts[2]);
                    vertices.Add(ConvertRosToUnity(new Vector3(x, y, z)));
                }
            }

            List<int> triangles = new List<int>();
            int faceDataStart = vertexDataStart + vertexCount;
            for (int i = 0; i < faceCount; i++)
            {
                string[] parts = lines[faceDataStart + i].Split(new char[] { ' ', '\t' }, StringSplitOptions.RemoveEmptyEntries);
                if (parts.Length < 4)
                {
                    Debug.LogWarning("Face data error, skipping face.");
                    continue;
                }
                int vertexInFace = int.Parse(parts[0]);
                if (vertexInFace == 3)
                {
                    triangles.Add(int.Parse(parts[1]));
                    triangles.Add(int.Parse(parts[2]));
                    triangles.Add(int.Parse(parts[3]));
                }
                else
                {
                    // 三角剖分：第一个顶点与后续顶点构成三角形
                    int firstIndex = int.Parse(parts[1]);
                    for (int j = 2; j < vertexInFace; j++)
                    {
                        triangles.Add(firstIndex);
                        triangles.Add(int.Parse(parts[j]));
                        triangles.Add(int.Parse(parts[j + 1]));
                    }
                }
            }

            Mesh mesh = new Mesh();
            mesh.SetVertices(vertices);
            mesh.SetTriangles(triangles, 0);

            if (isNOFF || isNCOFF)
            {
                mesh.SetNormals(vertexNormals);
            }
            else
            {
                mesh.RecalculateNormals();
            }
            mesh.RecalculateBounds();
            if (isNCOFF || isCOFF)
            {
                mesh.SetColors(vertexColors);
            }
            return mesh;
        }
        catch (Exception ex)
        {
            Debug.LogError("Error parsing OFF data: " + ex.Message);
            return null;
        }
    }

    // 坐标转换：Unity.x = -ROS.y, Unity.y = ROS.z, Unity.z = ROS.x
    Vector3 ConvertRosToUnity(Vector3 rosCoord)
    {
        return new Vector3(-rosCoord.y, rosCoord.z, rosCoord.x);
    }
}
