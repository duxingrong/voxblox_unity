using System;
using System.Collections.Generic;
using System.IO;
using UnityEngine;

public class OffMeshLoader : MonoBehaviour
{
    public string offFilePath; // 例如：Application.dataPath + "/off.off"

    void Start()
    {
        Mesh mesh = LoadOffFile(offFilePath);
        if (mesh != null)
        {
            MeshFilter mf = gameObject.AddComponent<MeshFilter>();
            mf.mesh = mesh;

            MeshRenderer mr = gameObject.GetComponent<MeshRenderer>();
            if (mr == null)
            {
                mr = gameObject.AddComponent<MeshRenderer>();
            }

            Shader shader = Shader.Find("Shader Graphs/VectorColor");
            if (shader == null)
            {
                Debug.LogError("未找到着色器 Shader Graphs/VectorColor，请检查着色器名称或确保它已经包含在项目中。");
            }
            else
            {
                Debug.Log("找到着色器: " + shader.name);
                Material material = new Material(shader);
                mr.material = material; // 直接应用材质
            }
        }
    }

    // 将 ROS 坐标转换为 Unity 坐标
    // 假设 ROS 坐标系为：x 前方，y 向左，z 向上
    // Unity 坐标系为：x 向右，y 向上，z 向前
    // 转换公式：Unity.x = -ROS.y, Unity.y = ROS.z, Unity.z = ROS.x
    Vector3 ConvertRosToUnity(Vector3 rosCoord)
    {
        return new Vector3(-rosCoord.y, rosCoord.z, rosCoord.x);
    }

    Mesh LoadOffFile(string path)
    {
        try
        {
            string[] lines = File.ReadAllLines(path);
            if (lines.Length < 2)
            {
                Debug.LogError("文件内容不足");
                return null;
            }

            string header = lines[0].Trim();
            bool isOFF = header == "OFF";
            bool isNOFF = header == "NOFF";
            bool isNCOFF = header == "NCOFF";
            bool isCOFF = header == "COFF";

            if (!isOFF && !isNOFF && !isNCOFF && !isCOFF)
            {
                Debug.LogError("不支持的文件格式");
                return null;
            }

            // 读取顶点数、面数（边数通常忽略）
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
                    // 每行需要 10 个数字：x y z r g b a nx ny nz
                    if (parts.Length < 9)
                    {
                        Debug.LogError("NCOFF 顶点数据格式错误, 预期 9 个数字，但实际只有 " + parts.Length);
                        return null;
                    }
                    float x = float.Parse(parts[0]);
                    float y = float.Parse(parts[1]);
                    float z = float.Parse(parts[2]);
                    Vector3 rosVertex = new Vector3(x, y, z);
                    vertices.Add(ConvertRosToUnity(rosVertex));

                    float r = float.Parse(parts[6]);
                    float g = float.Parse(parts[7]);
                    float b = float.Parse(parts[8]);
                    float a = 1;
                    vertexColors.Add(new Color(r, g, b, a));

                    float nx = float.Parse(parts[3]);
                    float ny = float.Parse(parts[4]);
                    float nz = float.Parse(parts[5]);
                    Vector3 rosNormal = new Vector3(nx, ny, nz);
                    vertexNormals.Add(ConvertRosToUnity(rosNormal));
                }
                else if (isCOFF)
                {
                    // 每行需要 7 个数字：x y z r g b a 
                    if (parts.Length < 6)
                    {
                        Debug.LogError("COFF 顶点数据格式错误, 预期6 个数字，但实际只有 " + parts.Length);
                        return null;
                    }
                    float x = float.Parse(parts[0]);
                    float y = float.Parse(parts[1]);
                    float z = float.Parse(parts[2]);
                    Vector3 rosVertex = new Vector3(x, y, z);
                    vertices.Add(ConvertRosToUnity(rosVertex));

                    float r = float.Parse(parts[3]);
                    float g = float.Parse(parts[4]);
                    float b = float.Parse(parts[5]);
                    float a = 1;
                    vertexColors.Add(new Color(r, g, b, a));
                }
                else if (isNOFF)
                {
                    // NOFF：每行 6 个数字：x y z nx ny nz
                    if (parts.Length < 6)
                    {
                        Debug.LogError("NOFF 顶点数据格式错误");
                        return null;
                    }
                    float x = float.Parse(parts[0]);
                    float y = float.Parse(parts[1]);
                    float z = float.Parse(parts[2]);
                    Vector3 rosVertex = new Vector3(x, y, z);
                    vertices.Add(ConvertRosToUnity(rosVertex));

                    float nx = float.Parse(parts[3]);
                    float ny = float.Parse(parts[4]);
                    float nz = float.Parse(parts[5]);
                    Vector3 rosNormal = new Vector3(nx, ny, nz);
                    vertexNormals.Add(ConvertRosToUnity(rosNormal));
                }
                else // OFF 格式，只包含顶点坐标
                {
                    if (parts.Length < 3)
                    {
                        Debug.LogError("OFF 顶点数据格式错误");
                        return null;
                    }
                    float x = float.Parse(parts[0]);
                    float y = float.Parse(parts[1]);
                    float z = float.Parse(parts[2]);
                    Vector3 rosVertex = new Vector3(x, y, z);
                    vertices.Add(ConvertRosToUnity(rosVertex));
                }
            }

            List<int> triangles = new List<int>();
            int faceDataStart = vertexDataStart + vertexCount;
            for (int i = 0; i < faceCount; i++)
            {
                string[] parts = lines[faceDataStart + i].Split(new char[] { ' ', '\t' }, StringSplitOptions.RemoveEmptyEntries);
                if (parts.Length < 4)
                {
                    Debug.LogWarning("面数据格式可能有误");
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
                    int firstIndex = int.Parse(parts[1]);
                    for (int j = 2; j < vertexInFace; j++)
                    {
                        triangles.Add(firstIndex);
                        triangles.Add(int.Parse(parts[j]));
                        triangles.Add(int.Parse(parts[j + 1]));
                    }
                }
            }

            // 翻转每个三角形的顶点顺序，以修正坐标转换后可能导致的背面剔除问题
            for (int i = 0; i < triangles.Count; i += 3)
            {
                int temp = triangles[i + 1];
                triangles[i + 1] = triangles[i + 2];
                triangles[i + 2] = temp;
            }

            Mesh mesh = new Mesh();
            mesh.indexFormat = UnityEngine.Rendering.IndexFormat.UInt32;
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
            Debug.LogError("读取 OFF 文件失败: " + ex.Message);
            return null;
        }
    }
}
