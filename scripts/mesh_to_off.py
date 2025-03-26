#!/usr/bin/env python3
import rospy
from voxblox_msgs.msg import Mesh
import numpy as np
import socket
import threading
import queue
import struct

# 全局字典，保存每个 mesh block 的最新数据，key 为 block.index 的 tuple
global_mesh_map = {}

# 用于 socket 发送的队列和事件
send_queue = queue.Queue()  
send_event = threading.Event()  
sock = None  # socket 连接对象

def create_connected_mesh(vertices, faces, colors=None, normals=None,tol=1e-10):
    """
    模拟 voxblox::createConnectedMesh 的功能：
      - 对顶点进行去冗余（合并接近的顶点），
      - 根据合并情况重新生成面索引，
      - 如果提供了颜色和法线数据，则合并它们（法线会累加后归一化）。
    
    参数:
      vertices: list[np.array]，每个元素为3D坐标
      faces: list[[i, j, k]]，每个三角面由顶点索引构成
      colors: list[tuple(r, g, b)]，与 vertices 对应的颜色；若无可传 None
      normals: list[np.array]，与 vertices 对应的法线；若无可传 None
      tol: 浮点数容差，决定合并精度（建议值接近 voxel 尺寸的倒数）
    
    返回:
      new_vertices, new_faces, new_colors, new_normals
    """
    threshold_inv = 1.0/tol
    vertex_map = {}  # 记录经容差处理后的顶点对应的新索引
    new_vertices = []
    new_colors = [] if colors is not None else None
    new_normals= [] if normals is not None else None
    index_map = {}  # 原始顶点索引到新索引的映射

    for i, v in enumerate(vertices):
        # 先缩放再四舍五入，获得离散化键
        key = tuple(np.round(v * threshold_inv).astype(np.int64))
        if key not in vertex_map:
            new_index = len(new_vertices)
            vertex_map[key] = new_index
            new_vertices.append(v)
            if colors is not None:
                new_colors.append(colors[i])
            if normals is not None:
                new_normals.append(normals[i].copy())
        else:
            new_index = vertex_map[key]
            if normals is not None:
                # 若重复，则累加法线
                new_normals[new_index] += normals[i]
        index_map[i] = vertex_map[key]    


    new_faces = []
    for face in faces:
        new_face = [index_map[face[0]], index_map[face[1]], index_map[face[2]]]
        # 若三角形退化（重复顶点），则舍弃
        if len(set(new_face)) == 3:
            new_faces.append(new_face)
    # 若有法线，则归一化每个顶点法线
    if normals is not None:
        for i, n in enumerate(new_normals):
            norm = np.linalg.norm(n)
            if norm > 1e-8:
                new_normals[i] = n / norm
            else:
                new_normals[i] = np.array([0.0, 0.0, 1.0])
    return new_vertices, new_faces, new_colors, new_normals if normals is not None else None



def compute_vertex_normals(vertices, faces):
    """
    通过累加相邻面的法线计算每个顶点的法线，
    算法：对每个面计算法线，再将其平均到对应顶点上，最后归一化。
    """
    vertex_normals = [np.zeros(3, dtype=np.float32) for _ in vertices]
    for face in faces:
        i0, i1, i2 = face
        v0, v1, v2 = vertices[i0], vertices[i1], vertices[i2]
        face_normal = np.cross(v1 - v0, v2 - v0)
        norm = np.linalg.norm(face_normal)
        if norm > 1e-8:
            face_normal /= norm
        for i in face:
            vertex_normals[i] += face_normal
    for i in range(len(vertex_normals)):
        norm = np.linalg.norm(vertex_normals[i])
        if norm > 1e-8:
            vertex_normals[i] /= norm
        else:
            vertex_normals[i] = np.array([0.0, 0.0, 1.0])
    return vertex_normals


def save_global_map(include_normals=False):
    """
    将全局累计的所有 mesh block 数据融合后生成 OFF 文件。
    如果 include_normals 为 True，则输出带法线信息的扩展 OFF 格式（NCOFF）。
    """
    global global_mesh_map
    global_vertices = []
    global_faces = []
    global_colors = []
    offset = 0
    for key in sorted(global_mesh_map.keys()):
        block = global_mesh_map[key]
        v = block['vertices']
        f = block['faces']
        c = block['colors']
        for face in f:
            global_faces.append([face[0] + offset, face[1] + offset, face[2] + offset])
        global_vertices.extend(v)
        global_colors.extend(c)
        offset += len(v)
    
    if len(global_vertices) == 0:
        rospy.logwarn("全局地图数据为空！")
        return

    normals = None
    header = "COFF"
    if include_normals:
        normals = compute_vertex_normals(global_vertices, global_faces)
        header = "NCOFF"  # 自定义头部，表示顶点含法线和颜色

    merged = create_connected_mesh(global_vertices, global_faces, global_colors, normals, tol=1e-10)
    if normals is not None:
        merged_vertices, merged_faces, merged_colors, merged_normals = merged
    else:
        merged_vertices, merged_faces, merged_colors, _ = merged

    lines = [header, f"{len(merged_vertices)} {len(merged_faces)} 0"]
    for i, v in enumerate(merged_vertices):
        if include_normals:
            n = merged_normals[i]
            c = merged_colors[i]
            # 每个顶点行：x y z nx ny nz r g b
            vertex_line = " ".join(f"{coord:.6f}" for coord in v) + " " + \
                          " ".join(f"{coord:.6f}" for coord in n) + " " + \
                          " ".join(f"{col:.6f}" for col in c)
        else:
            c = merged_colors[i]
            # 每个顶点行：x y z r g b
            vertex_line = " ".join(f"{coord:.6f}" for coord in v) + " " + \
                          " ".join(f"{col:.6f}" for col in c)
        lines.append(vertex_line)
    for face in merged_faces:
        lines.append(f"3 {face[0]} {face[1]} {face[2]}")
    off_str = "\n".join(lines)
    with open("global_map.off", "w") as f:
        f.write(off_str)
    rospy.loginfo("全局地图已保存为 global_map.off")



def send_data_thread():
    global sock 
    while not rospy.is_shutdown():
        send_event.wait()  # 等待数据到达
        send_event.clear()  # 清除事件标志
        try:
            while not send_queue.empty():
                data_str = send_queue.get_nowait()
                # 先发送4字节数据长度（大端序）
                sock.sendall(struct.pack(">I", len(data_str)))
                # 再发送数据内容
                sock.sendall(data_str.encode('utf-8'))
                rospy.loginfo("数据发送成功")
        except Exception as e:
            rospy.logerr(f"发送异常: {e}")


def mesh_callback(msg):
    """
    接收到 /mesh 消息后：
      - 对每个 mesh block 数据进行处理，
      - 更新全局字典 global_mesh_map，
      - 累计所有 block 后生成全局网格，
      - 对网格进行顶点合并与法线计算，
      - 生成带法线与颜色信息的 OFF 字符串并通过 socket 发送出去。
    """
    global global_mesh_map
    point_conv_factor = 2.0 / 65535.0

    for block in msg.mesh_blocks:
        block_key = tuple(block.index)
        num_vertices = len(block.x)
        if num_vertices == 0:
            if block_key in global_mesh_map:
                del global_mesh_map[block_key]
            continue

        vertices = []
        colors = []
        faces = []
        block_offset = np.array(block.index, dtype=np.float32) * msg.block_edge_length
        for i in range(num_vertices):
            q = np.array([block.x[i], block.y[i], block.z[i]], dtype=np.float32)
            vertex = block_offset + (q * point_conv_factor) * msg.block_edge_length
            vertices.append(vertex)
            if len(block.r) > i:
                colors.append((block.r[i], block.g[i], block.b[i]))
            else:
                colors.append((0.0, 0.0, 0.0))
        num_triangles = num_vertices // 3
        for i in range(num_triangles):
            base = i * 3
            faces.append([base, base + 1, base + 2])
        global_mesh_map[block_key] = {'vertices': vertices, 'colors': colors, 'faces': faces}

    # 累计全局数据
    global_vertices = []
    global_faces = []
    global_colors = []
    offset = 0
    for key in sorted(global_mesh_map.keys()):
        block = global_mesh_map[key]
        v = block['vertices']
        f = block['faces']
        c = block['colors']
        for face in f:
            global_faces.append([face[0] + offset, face[1] + offset, face[2] + offset])
        global_vertices.extend(v)
        global_colors.extend(c)
        offset += len(v)
    
    # 计算全局顶点法线
    normals = compute_vertex_normals(global_vertices, global_faces)
    # 使用改进后的合并算法（tol 选用 1e-10）
    merged_vertices, merged_faces, merged_colors, merged_normals = create_connected_mesh(
        global_vertices, global_faces, global_colors, normals, tol=1e-10)
    # 输出带法线和颜色的 OFF 格式（NCOFF）
    header = "NCOFF"
    lines = [header, f"{len(merged_vertices)} {len(merged_faces)} 0"]
    for i, v in enumerate(merged_vertices):
        n = merged_normals[i]
        c = merged_colors[i]
        vertex_line = " ".join(f"{coord:.6f}" for coord in v) + " " + \
                      " ".join(f"{coord:.6f}" for coord in n) + " " + \
                      " ".join(f"{col:.6f}" for col in c)
        lines.append(vertex_line)
    for face in merged_faces:
        lines.append(f"3 {face[0]} {face[1]} {face[2]}")
    off_str = "\n".join(lines)
    send_queue.put(off_str)
    send_event.set()



def main():
    global sock
    rospy.init_node('accumulate_mesh_to_off')
    rospy.Subscriber("/voxblox_node/mesh", Mesh, mesh_callback)
    # 程序退出前保存全局地图到文件
    # rospy.on_shutdown(save_global_map)

    # 初始化 socket 连接（根据需要修改 IP 与端口）
    sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    sock.connect(("127.0.0.1", 12345))
    # sock.connect(("10.192.42.21", 12345))
    send_thread = threading.Thread(target=send_data_thread, daemon=True)
    send_thread.start()
    rospy.spin()

if __name__ == '__main__':
    main()
