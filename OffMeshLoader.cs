using System;
using System.Collections.Generic;
using System.IO;
using UnityEngine;

public class OffMeshLoader : MonoBehaviour
{
    public string offFilePath; // ÀýÈç£ºApplication.dataPath + "/off.off"

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
                Debug.LogError("Î´ÕÒµ½×ÅÉ«Æ÷ Shader Graphs/VectorColor£¬Çë¼ì²é×ÅÉ«Æ÷Ãû³Æ»òÈ·±£ËüÒÑ¾­°üº¬ÔÚÏîÄ¿ÖÐ¡£");
            }
            else
            {
                Debug.Log("ÕÒµ½×ÅÉ«Æ÷: " + shader.name);
                Material material = new Material(shader);
                mr.material = material; // Ö±½ÓÓ¦ÓÃ²ÄÖÊ
            }
        }
    }



    // ½« ROS ×ø±ê×ª»»Îª Unity ×ø±ê
    // ¼ÙÉè ROS ×ø±êÏµÎª£ºx Ç°·½£¬y Ïò×ó£¬z ÏòÉÏ
    // Unity ×ø±êÏµÎª£ºx ÏòÓÒ£¬y ÏòÉÏ£¬z ÏòÇ°
    // ×ª»»¹«Ê½£ºUnity.x = -ROS.y, Unity.y = ROS.z, Unity.z = ROS.x
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
                Debug.LogError("ÎÄ¼þÄÚÈÝ²»×ã");
                return null;
            }

            string header = lines[0].Trim();
            bool isOFF = header == "OFF";
            bool isNOFF = header == "NOFF";
            bool isNCOFF = header == "NCOFF";
            bool isCOFF = header == "COFF";

            if (!isOFF && !isNOFF && !isNCOFF && !isCOFF)
            {
                Debug.LogError("²»Ö§³ÖµÄÎÄ¼þ¸ñÊ½");
                return null;
            }

            // ¶ÁÈ¡¶¥µãÊý¡¢ÃæÊý£¨±ßÊýÍ¨³£ºöÂÔ£©
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
                    // Ã¿ÐÐÐèÒª 10 ¸öÊý×Ö£ºx y z r g b a nx ny nz
                    if (parts.Length < 9)
                    {
                        Debug.LogError("NCOFF ¶¥µãÊý¾Ý¸ñÊ½´íÎó, Ô¤ÆÚ 10 ¸öÊý×Ö£¬µ«Êµ¼ÊÖ»ÓÐ " + parts.Length);
                        return null;
                    }
                    // ¶ÁÈ¡¶¥µã×ø±ê²¢×ª»»
                    float x = float.Parse(parts[0]);
                    float y = float.Parse(parts[1]);
                    float z = float.Parse(parts[2]);
                    Vector3 rosVertex = new Vector3(x, y, z);
                    vertices.Add(ConvertRosToUnity(rosVertex));

                    // ¶ÁÈ¡ÑÕÉ«Êý¾Ý
                    float r = float.Parse(parts[3]);
                    float g = float.Parse(parts[4]);
                    float b = float.Parse(parts[5]);
                    float a = 255;
                    // Èç¹ûÑÕÉ«Öµ´óÓÚ 1£¬ÔòÈÏÎªÊÇ 0¡«255 ÐèÒª¹éÒ»»¯
                    if (r > 1f || g > 1f || b > 1f || a > 1f)
                    {
                        r /= 255f;
                        g /= 255f;
                        b /= 255f;
                        a /= 255f;
                    }
                    vertexColors.Add(new Color(r, g, b, a));

                    // ¶ÁÈ¡·¨ÏòÁ¿²¢×ª»»
                    float nx = float.Parse(parts[7]);
                    float ny = float.Parse(parts[8]);
                    float nz = float.Parse(parts[9]);
                    Vector3 rosNormal = new Vector3(nx, ny, nz);
                    vertexNormals.Add(ConvertRosToUnity(rosNormal));
                }
                else if (isCOFF)
                {
                    // Ã¿ÐÐÐèÒª 7¸öÊý×Ö£ºx y z r g b 
                    if (parts.Length < 6)
                    {
                        Debug.LogError("NCOFF ¶¥µãÊý¾Ý¸ñÊ½´íÎó, Ô¤ÆÚ7 ¸öÊý×Ö£¬µ«Êµ¼ÊÖ»ÓÐ " + parts.Length);
                        return null;
                    }
                    // ¶ÁÈ¡¶¥µã×ø±ê²¢×ª»»
                    float x = float.Parse(parts[0]);
                    float y = float.Parse(parts[1]);
                    float z = float.Parse(parts[2]);
                    Vector3 rosVertex = new Vector3(x, y, z);
                    vertices.Add(ConvertRosToUnity(rosVertex));

                    // ¶ÁÈ¡ÑÕÉ«Êý¾Ý
                    float r = float.Parse(parts[3]);
                    float g = float.Parse(parts[4]);
                    float b = float.Parse(parts[5]);
                    float a = 255;
                    // Èç¹ûÑÕÉ«Öµ´óÓÚ 1£¬ÔòÈÏÎªÊÇ 0¡«255 ÐèÒª¹éÒ»»¯
                    if (r > 1f || g > 1f || b > 1f || a > 1f)
                    {
                        r /= 255f;
                        g /= 255f;
                        b /= 255f;
                        a /= 255f;
                    }
                    vertexColors.Add(new Color(r, g, b, a));

                }
                else if (isNOFF)
                {
                    // NOFF£ºÃ¿ÐÐ 6 ¸öÊý×Ö£ºx y z nx ny nz
                    if (parts.Length < 6)
                    {
                        Debug.LogError("NOFF ¶¥µãÊý¾Ý¸ñÊ½´íÎó");
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
                else // OFF ¸ñÊ½£¬Ö»°üº¬¶¥µã×ø±ê
                {
                    if (parts.Length < 3)
                    {
                        Debug.LogError("OFF ¶¥µãÊý¾Ý¸ñÊ½´íÎó");
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
                    Debug.LogWarning("ÃæÊý¾Ý¸ñÊ½¿ÉÄÜÓÐÎó");
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
                    // ¶à±ßÐÎ²ÉÓÃÉÈÐÎÆÊ·Ö
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

            // Èç¹ûÌá¹©ÁË·¨ÏòÁ¿£¬ÔòÖ±½ÓÉèÖÃ£¬·ñÔòÖØËã·¨ÏòÁ¿
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
            Debug.LogError("¶ÁÈ¡ OFF ÎÄ¼þÊ§°Ü: " + ex.Message);
            return null;
        }
    }
}
