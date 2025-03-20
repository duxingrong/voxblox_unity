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

def create_connected_mesh(vertices, faces, colors, tol=1e-6):
    """
    模拟 voxblox::createConnectedMesh 的功能：
    对顶点进行去重（剔除冗余顶点）并重新生成面索引，
    使得网格拓扑更加规整。
    
    参数:
      vertices: list[np.array]，每个元素为3D坐标
      faces: list[[i, j, k]]，每个三角面由顶点索引构成
      colors: list[tuple(r, g, b)]，与 vertices 对应的颜色；如果无颜色可传 None
      tol: 浮点数容差，决定去重精度
      
    返回:
      new_vertices, new_faces, new_colors
    """
    vertex_map = {}  # 记录经容差处理后的顶点对应的新索引
    new_vertices = []
    new_colors = [] if colors is not None else None
    index_map = {}  # 原始顶点索引到新索引的映射

    for i, v in enumerate(vertices):
        key = tuple(np.round(v / tol).astype(np.int64))
        if key not in vertex_map:
            new_index = len(new_vertices)
            vertex_map[key] = new_index
            new_vertices.append(v)
            if colors is not None:
                new_colors.append(colors[i])
        index_map[i] = vertex_map[key]
    
    new_faces = []
    for face in faces:
        new_face = [index_map[face[0]], index_map[face[1]], index_map[face[2]]]
        if len(set(new_face)) == 3:
            new_faces.append(new_face)
    return new_vertices, new_faces, new_colors

def compute_normals(vertices, faces):
    """
    对每个三角面计算法线，返回各面的法线列表
    """
    normals = []
    for face in faces:
        v0 = vertices[face[0]]
        v1 = vertices[face[1]]
        v2 = vertices[face[2]]
        dir0 = v0 - v1
        dir1 = v0 - v2
        normal = np.cross(dir0, dir1)
        norm = np.linalg.norm(normal)
        if norm > 0:
            normal /= norm
        else:
            normal = np.array([0.0, 0.0, 0.0])
        normals.append(normal)
    return normals

def save_global_map():
    """
    将累积的所有 mesh block 数据融合后生成全局 OFF 文件，保存到 local 文件
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

    # 去冗余和整理拓扑
    global_vertices, global_faces, global_colors = create_connected_mesh(global_vertices, global_faces, global_colors, tol=1e-6)
    header = "COFF"  # 这里统一使用带颜色格式
    lines = [header, f"{len(global_vertices)} {len(global_faces)} 0"]
    for i, v in enumerate(global_vertices):
        c = global_colors[i]
        vertex_line = " ".join(f"{coord:.6f}" for coord in v) + " " + " ".join(f"{col:.6f}" for col in c)
        lines.append(vertex_line)
    for face in global_faces:
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
    收到 /mesh 消息后，对每个 mesh block 进行处理，
    更新全局字典 global_mesh_map 中对应块的数据。
    同时，累积所有块数据生成全局 OFF 格式字符串，并通过 socket 发送出去。
    """
    global global_mesh_map
    point_conv_factor = 2.0 / 65535.0

    for block in msg.mesh_blocks:
        block_key = tuple(block.index)
        num_vertices = len(block.x)
        # 若数据为空则删除对应的块
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
        # 更新或添加当前 block 数据
        global_mesh_map[block_key] = {'vertices': vertices, 'colors': colors, 'faces': faces}

    # 累积全局数据
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

    # 进行去冗余与拓扑整理
    global_vertices, global_faces, global_colors = create_connected_mesh(global_vertices, global_faces, global_colors, tol=1e-6)
    header = "COFF"
    lines = [header, f"{len(global_vertices)} {len(global_faces)} 0"]
    for i, v in enumerate(global_vertices):
        c = global_colors[i]
        vertex_line = " ".join(f"{coord:.6f}" for coord in v) + " " + " ".join(f"{col:.6f}" for col in c)
        lines.append(vertex_line)
    for face in global_faces:
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
    send_thread = threading.Thread(target=send_data_thread, daemon=True)
    send_thread.start()
    rospy.spin()

if __name__ == '__main__':
    main()
