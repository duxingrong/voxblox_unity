using UnityEngine;
using System;
using System.Net;
using System.Net.Sockets;
using System.Text;
using System.Threading;

public class OdomReceiver : MonoBehaviour
{
    // 监听端口，跟ROS端参数一致（默认8888）
    public int serverPort = 8888;
    private TcpListener listener;
    private Thread listenerThread;

    // 标记是否已接收到odom数据（只接收一次，接收到后关闭服务器）
    private bool receivedOdom = false;

    // 存储接收到的odom位姿（已转换为Unity坐标系）
    private Vector3 odomPos;
    private Quaternion odomRot;
    // 标记有新的odom数据（在Update中使用）
    private bool newOdomData = false;

    void Start()
    {
        // 开启服务器线程
        listenerThread = new Thread(new ThreadStart(ServerThread));
        listenerThread.IsBackground = true;
        listenerThread.Start();
    }

    /// <summary>
    /// 服务器线程：等待ROS端通过socket发送odom JSON数据
    /// </summary>
    void ServerThread()
    {
        try
        {
            listener = new TcpListener(IPAddress.Any, serverPort);
            listener.Start();
            Debug.Log("TCP服务器已启动，等待连接...");
            // 等待客户端连接（阻塞调用）
            TcpClient client = listener.AcceptTcpClient();
            Debug.Log("收到客户端连接：" + client.Client.RemoteEndPoint.ToString());

            // 读取客户端发来的数据
            NetworkStream stream = client.GetStream();
            byte[] buffer = new byte[1024];
            int bytesRead = stream.Read(buffer, 0, buffer.Length);
            if (bytesRead > 0)
            {
                string jsonString = Encoding.UTF8.GetString(buffer, 0, bytesRead);
                Debug.Log("接收到JSON数据：" + jsonString);
                ParseOdomJson(jsonString);
                receivedOdom = true;
            }
            // 接收完数据后关闭连接和服务器
            stream.Close();
            client.Close();
            listener.Stop();
            Debug.Log("服务器关闭。");
        }
        catch (Exception ex)
        {
            Debug.LogError("服务器异常：" + ex.Message);
        }
    }

    /// <summary>
    /// 解析JSON字符串，提取odom位姿数据，并进行ROS→Unity的坐标转换
    /// </summary>
    /// <param name="jsonString">来自ROS端的JSON字符串</param>
    void ParseOdomJson(string jsonString)
    {
        try
        {
            OdomData data = JsonUtility.FromJson<OdomData>(jsonString);
            if (data != null)
            {
                // 将ROS坐标转换为Unity坐标（转换函数需根据具体坐标系定义修改）
                odomPos = ConvertRosPosToUnity(new Vector3(data.position.x, data.position.y, data.position.z));
                odomRot = ConvertRosOriToUnity(new Quaternion(data.orientation.x, data.orientation.y, data.orientation.z, data.orientation.w));
                newOdomData = true;
            }
        }
        catch (Exception ex)
        {
            Debug.LogError("解析JSON数据出错：" + ex.Message);
        }
    }

    /// <summary>
    /// ROS→Unity坐标系位置转换函数（示例转换，根据实际情况调整）
    /// 假设ROS中采用(x: forward, y: left, z: up)，而Unity中采用(x: right, y: up, z: forward)
    /// 此处转换示例：unity_x = -ros_y, unity_y = ros_z, unity_z = ros_x
    /// </summary>
    Vector3 ConvertRosPosToUnity(Vector3 rosPos)
    {
        return new Vector3(-rosPos.y, rosPos.z, rosPos.x);
    }

    /// <summary>
    /// ROS→Unity四元数转换函数（示例转换，根据实际情况调整）
    /// 这里只是一个占位示例，请根据实际需要进行修正
    /// </summary>
    Quaternion ConvertRosOriToUnity(Quaternion rosOri)
    {
        // 示例：简单交换分量，实际中可能需要乘以额外旋转
        return new Quaternion(rosOri.y, rosOri.z, rosOri.x, rosOri.w);
    }

    void Update()
    {
        // 如果有新的odom数据，则进行坐标变换更新主摄像机
        if (newOdomData)
        {
            // 获取当前HoloLens（摄像机）的位姿
            Vector3 humanPos = Camera.main.transform.position;
            Quaternion humanRot = Camera.main.transform.rotation;

            // 计算相对位姿：相对变换 = inverse(odom) * human
            Vector3 relativePos = Quaternion.Inverse(odomRot) * (humanPos - odomPos);
            Quaternion relativeRot = Quaternion.Inverse(odomRot) * humanRot;

            // 更新主摄像机变换，使得摄像机到世界原点的变换等于计算得到的相对变换
            Camera.main.transform.position = relativePos;
            Camera.main.transform.rotation = relativeRot;

            Debug.Log("基于接收到的odom数据更新了摄像机变换。");

            // 更新完成后清除标记，防止重复更新（如果只需要一次）
            newOdomData = false;
        }
    }

    // 用于JSON解析的辅助数据类
    [Serializable]
    public class OdomData
    {
        public PositionData position;
        public OrientationData orientation;
    }

    [Serializable]
    public class PositionData
    {
        public float x;
        public float y;
        public float z;
    }

    [Serializable]
    public class OrientationData
    {
        public float x;
        public float y;
        public float z;
        public float w;
    }

    void OnApplicationQuit()
    {
        // 退出时关闭服务器和线程
        if (listener != null)
        {
            listener.Stop();
        }
        if (listenerThread != null && listenerThread.IsAlive)
        {
            listenerThread.Abort();
        }
    }
}
