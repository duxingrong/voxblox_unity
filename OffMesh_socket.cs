using System;
using System.Collections.Generic;
using System.Net;
using System.Net.Sockets;
using System.Threading;
using UnityEngine;

public class OffMeshServer : MonoBehaviour
{
    public int port = 12345; // ¼àÌý¶Ë¿Ú
    private TcpListener listener;
    private Thread listenerThread;
    private volatile bool isRunning = false;

    // ÓÃÓÚÏß³Ì¼ä´«µÝ off Êý¾ÝµÄ¶ÓÁÐ
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
    /// Æô¶¯ TCP ·þÎñÆ÷£¬¼àÌýÀ´×Ô Python ¿Í»§¶ËµÄÁ¬½Ó
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
    /// µÈ´ý¿Í»§¶ËÁ¬½Ó£¬Ã¿µ±ÓÐ¿Í»§¶ËÁ¬½ÓÊ±£¬ÎªÆäÆô¶¯Ò»¸öÐÂµÄÏß³Ì´¦ÀíÊý¾Ý½ÓÊÕ
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
    /// ´¦Àíµ¥¸ö¿Í»§¶ËÁ¬½Ó
    /// Ð­Òé£ºÏÈ½ÓÊÕ 4 ×Ö½Ú£¨´ó¶ËÐò£©Êý¾Ý³¤¶È£¬ÔÙ½ÓÊÕ off Êý¾Ý×Ö·û´®
    /// </summary>
    /// <param name="client"></param>
    void HandleClient(TcpClient client)
    {
        NetworkStream stream = client.GetStream();
        try
        {
            while (isRunning && client.Connected)
            {
                // ½ÓÊÕÇ° 4 ×Ö½Ú£¬»ñÈ¡Êý¾Ý³¤¶È
                byte[] lengthBuffer = new byte[4];
                int readBytes = stream.Read(lengthBuffer, 0, 4);
                if (readBytes < 4)
                {
                    Debug.LogWarning("Client disconnected or error reading length.");
                    break;
                }
                // ´¦Àí´ó¶ËÐòÊý¾Ý£¨Èç¹ûÏµÍ³ÎªÐ¡¶Ë£©
                if (BitConverter.IsLittleEndian)
                {
                    Array.Reverse(lengthBuffer);
                }
                int dataLength = BitConverter.ToInt32(lengthBuffer, 0);

                // ½ÓÊÕ off Êý¾ÝÄÚÈÝ
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

                // ½«½ÓÊÕµ½µÄ off Êý¾Ý·ÅÈë¶ÓÁÐ£¬µÈ´ýÖ÷Ïß³Ì´¦Àí¸üÐÂ Mesh
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
        // Ö÷Ïß³ÌÖÐ¼ì²â¶ÓÁÐÊÇ·ñÓÐÐÂ off Êý¾Ý
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
    /// ´Ó off ¸ñÊ½×Ö·û´®½âÎö Mesh£¬Óë´ÓÎÄ¼þ¶ÁÈ¡Âß¼­ÀàËÆ
    /// </summary>
    /// <param name="offData"></param>
    /// <returns></returns>
    Mesh LoadOffFromString(string offData)
    {
        try
        {
            // °´ÐÐ·Ö¸î£¬È¥³ý¿ÕÐÐ
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

            // ½âÎö¶¥µãÊýºÍÃæÊý
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
                    // NCOFF ¸ñÊ½£ºx y z  nx ny nz r g b 
                    if (parts.Length < 9)
                    {
                        Debug.LogError("NCOFF vertex data error, expected 9 values, got " + parts.Length);
                        return null;
                    }
                    float x = float.Parse(parts[0]);
                    float y = float.Parse(parts[1]);
                    float z = float.Parse(parts[2]);
                    vertices.Add(ConvertRosToUnity(new Vector3(x, y, z)));

                    float r = float.Parse(parts[6]);
                    float g = float.Parse(parts[7]);
                    float b = float.Parse(parts[8]);
                    float a = 255;
                    if (r > 1f || g > 1f || b > 1f || a > 1f)
                    {
                        r /= 255f; g /= 255f; b /= 255f; a /= 255f;
                    }
                    vertexColors.Add(new Color(r, g, b, a));

                    float nx = float.Parse(parts[3]);
                    float ny = float.Parse(parts[4]);
                    float nz = float.Parse(parts[5]);
                    vertexNormals.Add(ConvertRosToUnity(new Vector3(nx, ny, nz)));
                }
                else if (isCOFF)
                {
                    // COFF ¸ñÊ½£ºx y z r g b 
                    if (parts.Length < 6)
                    {
                        Debug.LogError("COFF vertex data error, expected 6 values, got " + parts.Length);
                        return null;
                    }
                    float x = float.Parse(parts[0]);
                    float y = float.Parse(parts[1]);
                    float z = float.Parse(parts[2]);
                    vertices.Add(ConvertRosToUnity(new Vector3(x, y, z)));
                    // vertices.Add(new Vector3(x,y,z));

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
                    // NOFF ¸ñÊ½£ºx y z nx ny nz
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
                else // OFF ¸ñÊ½£¬Ö»°üº¬¶¥µã×ø±ê
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
                    // Èý½ÇÆÊ·Ö£ºµÚÒ»¸ö¶¥µãÓëºóÐø¶¥µã¹¹³ÉÈý½ÇÐÎ
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
            mesh.indexFormat = UnityEngine.Rendering.IndexFormat.UInt32; // ¿¿¿¿¿¿¿ 32 ¿
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

    // ×ø±ê×ª»»£ºUnity.x = -ROS.y, Unity.y = ROS.z, Unity.z = ROS.x
    Vector3 ConvertRosToUnity(Vector3 rosCoord)
    {
        return new Vector3(-rosCoord.y, rosCoord.z, rosCoord.x);
    }
}
