using UnityEngine;
using System;
using System.Net;
using System.Net.Sockets;
using System.Text;
using System.Threading;

public class OdomReceiver : MonoBehaviour
{
    // �����˿ڣ���ROS�˲���һ�£�Ĭ��8888��
    public int serverPort = 8888;
    private TcpListener listener;
    private Thread listenerThread;

    // ����Ƿ��ѽ��յ�odom���ݣ�ֻ����һ�Σ����յ���رշ�������
    private bool receivedOdom = false;

    // �洢���յ���odomλ�ˣ���ת��ΪUnity����ϵ��
    private Vector3 odomPos;
    private Quaternion odomRot;
    // ������µ�odom���ݣ���Update��ʹ�ã�
    private bool newOdomData = false;

    void Start()
    {
        // �����������߳�
        listenerThread = new Thread(new ThreadStart(ServerThread));
        listenerThread.IsBackground = true;
        listenerThread.Start();
    }

    /// <summary>
    /// �������̣߳��ȴ�ROS��ͨ��socket����odom JSON����
    /// </summary>
    void ServerThread()
    {
        try
        {
            listener = new TcpListener(IPAddress.Any, serverPort);
            listener.Start();
            Debug.Log("TCP���������������ȴ�����...");
            // �ȴ��ͻ������ӣ��������ã�
            TcpClient client = listener.AcceptTcpClient();
            Debug.Log("�յ��ͻ������ӣ�" + client.Client.RemoteEndPoint.ToString());

            // ��ȡ�ͻ��˷���������
            NetworkStream stream = client.GetStream();
            byte[] buffer = new byte[1024];
            int bytesRead = stream.Read(buffer, 0, buffer.Length);
            if (bytesRead > 0)
            {
                string jsonString = Encoding.UTF8.GetString(buffer, 0, bytesRead);
                Debug.Log("���յ�JSON���ݣ�" + jsonString);
                ParseOdomJson(jsonString);
                receivedOdom = true;
            }
            // ���������ݺ�ر����Ӻͷ�����
            stream.Close();
            client.Close();
            listener.Stop();
            Debug.Log("�������رա�");
        }
        catch (Exception ex)
        {
            Debug.LogError("�������쳣��" + ex.Message);
        }
    }

    /// <summary>
    /// ����JSON�ַ�������ȡodomλ�����ݣ�������ROS��Unity������ת��
    /// </summary>
    /// <param name="jsonString">����ROS�˵�JSON�ַ���</param>
    void ParseOdomJson(string jsonString)
    {
        try
        {
            OdomData data = JsonUtility.FromJson<OdomData>(jsonString);
            if (data != null)
            {
                // ��ROS����ת��ΪUnity���꣨ת����������ݾ�������ϵ�����޸ģ�
                odomPos = ConvertRosPosToUnity(new Vector3(data.position.x, data.position.y, data.position.z));
                odomRot = ConvertRosOriToUnity(new Quaternion(data.orientation.x, data.orientation.y, data.orientation.z, data.orientation.w));
                newOdomData = true;
            }
        }
        catch (Exception ex)
        {
            Debug.LogError("����JSON���ݳ���" + ex.Message);
        }
    }

    /// <summary>
    /// ROS��Unity����ϵλ��ת��������ʾ��ת��������ʵ�����������
    /// ����ROS�в���(x: forward, y: left, z: up)����Unity�в���(x: right, y: up, z: forward)
    /// �˴�ת��ʾ����unity_x = -ros_y, unity_y = ros_z, unity_z = ros_x
    /// </summary>
    Vector3 ConvertRosPosToUnity(Vector3 rosPos)
    {
        return new Vector3(-rosPos.y, rosPos.z, rosPos.x);
    }

    /// <summary>
    /// ROS��Unity��Ԫ��ת��������ʾ��ת��������ʵ�����������
    /// ����ֻ��һ��ռλʾ���������ʵ����Ҫ��������
    /// </summary>
    Quaternion ConvertRosOriToUnity(Quaternion rosOri)
    {
        // ʾ�����򵥽���������ʵ���п�����Ҫ���Զ�����ת
        return new Quaternion(rosOri.y, rosOri.z, rosOri.x, rosOri.w);
    }

    void Update()
    {
        // ������µ�odom���ݣ����������任�����������
        if (newOdomData)
        {
            // ��ȡ��ǰHoloLens�����������λ��
            Vector3 humanPos = Camera.main.transform.position;
            Quaternion humanRot = Camera.main.transform.rotation;

            // �������λ�ˣ���Ա任 = inverse(odom) * human
            Vector3 relativePos = Quaternion.Inverse(odomRot) * (humanPos - odomPos);
            Quaternion relativeRot = Quaternion.Inverse(odomRot) * humanRot;

            // ������������任��ʹ�������������ԭ��ı任���ڼ���õ�����Ա任
            Camera.main.transform.position = relativePos;
            Camera.main.transform.rotation = relativeRot;

            Debug.Log("���ڽ��յ���odom���ݸ�����������任��");

            // ������ɺ������ǣ���ֹ�ظ����£����ֻ��Ҫһ�Σ�
            newOdomData = false;
        }
    }

    // ����JSON�����ĸ���������
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
        // �˳�ʱ�رշ��������߳�
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
