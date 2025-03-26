using System;
using System.Collections.Generic;
using System.Net;
using System.Net.Sockets;
using System.Threading;
using UnityEngine;

public class OffMesh_Socket : MonoBehaviour
{
    public int port = 12345; // �����˿�
    private TcpListener listener;
    private Thread listenerThread;
    private volatile bool isRunning = false;

    // �����̼߳䴫�� off ���ݵĶ���
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
    /// ���� TCP ���������������� Python �ͻ��˵�����
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
    /// �ȴ��ͻ������ӣ�ÿ���пͻ�������ʱ��Ϊ������һ���µ��̴߳������ݽ���
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
    /// ���������ͻ�������
    /// Э�飺�Ƚ��� 4 �ֽڣ���������ݳ��ȣ��ٽ��� off �����ַ���
    /// </summary>
    /// <param name="client"></param>
    void HandleClient(TcpClient client)
    {
        NetworkStream stream = client.GetStream();
        try
        {
            while (isRunning && client.Connected)
            {
                // ����ǰ 4 �ֽڣ���ȡ���ݳ���
                byte[] lengthBuffer = new byte[4];
                int readBytes = stream.Read(lengthBuffer, 0, 4);
                if (readBytes < 4)
                {
                    Debug.LogWarning("Client disconnected or error reading length.");
                    break;
                }
                // ������������ݣ����ϵͳΪС�ˣ�
                if (BitConverter.IsLittleEndian)
                {
                    Array.Reverse(lengthBuffer);
                }
                int dataLength = BitConverter.ToInt32(lengthBuffer, 0);

                // ���� off ��������
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

                // �����յ��� off ���ݷ�����У��ȴ����̴߳������� Mesh
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
        // ���߳��м������Ƿ����� off ����
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
    /// 从 OFF 格式的字符串数据加载 Mesh，并翻转三角形的顶点顺序
    /// </summary>
    /// <param name="offData">接收到的 OFF 格式数据</param>
    /// <returns>解析得到的 Mesh 对象</returns>
    Mesh LoadOffFromString(string offData)
    {
        try
        {
            // 按行拆分数据并去除空行
            string[] lines = offData.Split(new[] { "\r\n", "\r", "\n" }, StringSplitOptions.RemoveEmptyEntries);
            if (lines.Length < 2)
            {
                Debug.LogError("OFF 数据为空或不完整。");
                return null;
            }

            string header = lines[0].Trim();
            bool isOFF = header == "OFF";
            bool isNOFF = header == "NOFF";
            bool isNCOFF = header == "NCOFF";
            bool isCOFF = header == "COFF";

            if (!isOFF && !isNOFF && !isNCOFF && !isCOFF)
            {
                Debug.LogError("不支持的 OFF 格式: " + header);
                return null;
            }

            // 读取顶点数和面数
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
                    // NCOFF 格式：x y z nx ny nz r g b
                    if (parts.Length < 9)
                    {
                        Debug.LogError("NCOFF 顶点数据格式错误，预期 9 个值，但实际只有 " + parts.Length);
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

                    float r = float.Parse(parts[6]);
                    float g = float.Parse(parts[7]);
                    float b = float.Parse(parts[8]);
                    float a = 1;
                    vertexColors.Add(new Color(r, g, b,a));
                }
                else if (isCOFF)
                {
                    // COFF 格式：x y z r g b
                    if (parts.Length < 6)
                    {
                        Debug.LogError("COFF 顶点数据格式错误，预期 6 个值，但实际只有 " + parts.Length);
                        return null;
                    }
                    float x = float.Parse(parts[0]);
                    float y = float.Parse(parts[1]);
                    float z = float.Parse(parts[2]);
                    vertices.Add(ConvertRosToUnity(new Vector3(x, y, z)));

                    float r = float.Parse(parts[3]);
                    float g = float.Parse(parts[4]);
                    float b = float.Parse(parts[5]);
                    float a = 1;
                    vertexColors.Add(new Color(r, g, b,a));
                }
                else if (isNOFF)
                {
                    // NOFF 格式：x y z nx ny nz
                    if (parts.Length < 6)
                    {
                        Debug.LogError("NOFF 顶点数据格式错误，预期 6 个值，但实际只有 " + parts.Length);
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
                else // OFF 格式：x y z
                {
                    if (parts.Length < 3)
                    {
                        Debug.LogError("OFF 顶点数据格式错误，预期 3 个值，但实际只有 " + parts.Length);
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
                    Debug.LogWarning("面数据格式错误，跳过该面。");
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
                    // 多边形面，采用扇形分割
                    int firstIndex = int.Parse(parts[1]);
                    for (int j = 2; j < vertexInFace; j++)
                    {
                        triangles.Add(firstIndex);
                        triangles.Add(int.Parse(parts[j]));
                        triangles.Add(int.Parse(parts[j + 1]));
                    }
                }
            }

            // 翻转三角形顶点顺序，以修正坐标转换后可能导致的背面剔除问题
            for (int i = 0; i < triangles.Count; i += 3)
            {
                int temp = triangles[i + 1];
                triangles[i + 1] = triangles[i + 2];
                triangles[i + 2] = temp;
            }

            Mesh mesh = new Mesh();
            mesh.indexFormat = UnityEngine.Rendering.IndexFormat.UInt32; // 设置索引格式为 32 位
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
            Debug.LogError("解析 OFF 数据时出错: " + ex.Message);
            return null;
        }
    }


    // ����ת����Unity.x = -ROS.y, Unity.y = ROS.z, Unity.z = ROS.x
    Vector3 ConvertRosToUnity(Vector3 rosCoord)
    {
        return new Vector3(-rosCoord.y, rosCoord.z, rosCoord.x);
    }
}
