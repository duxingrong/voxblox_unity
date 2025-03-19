#!/usr/bin/env python3

import rospy
from voxblox_msgs.msg import Mesh
import numpy as np
import socket 
import threading
import queue
import struct

## 全局变量
send_queue = queue.Queue() # 线程安全队列存储待发送数据
send_event = threading.Event() #用于线程同步的事件标志
sock = None # Socket连接对象

def send_data_thread():
    global sock 
    while not rospy.is_shutdown():
        send_event.wait() # 阻塞等待数据到达
        send_event.clear() #清除事件标志

        try:
            while not send_queue.empty():
                data_str = send_queue.get_nowait() # 取出队列头部数据
                #先发送4字节数据长度(大端序)
                sock.sendall(struct.pack(">I",len(data_str)))
                #再发送数据内容
                sock.sendall(data_str.encode('utf-8'))
                rospy.loginfo("发送成功")
        except Exception as e:
            rospy.logerr(f"发送异常:{e}")

def mesh_callback(msg):
    vertices = []  # 存储全局顶点坐标
    colors = []    # 存储对应的顶点颜色 (r, g, b)
    faces = []     # 存储三角面顶点索引
    vertex_index = 0

    # 每个块内的体素数量（根据 tsdf_voxels_per_side 参数，这里固定为16）
    voxels_per_side = 16
    # 计算每个体素的实际尺寸
    voxel_scale = msg.block_edge_length / float(voxels_per_side)

    # 转换因子，用于将16位无符号整数转换为浮点数
    point_conv_factor = 2.0 / 65535.0

    # 遍历每个 MeshBlock
    for block in msg.mesh_blocks:
        block_offset = np.array(block.index, dtype=np.float32) * msg.block_edge_length

        num_vertices = len(block.x)
        num_triangles = num_vertices // 3

        for i in range(num_triangles):
            v1 = block_offset + (((np.array([block.x[3*i],   block.y[3*i],   block.z[3*i]],   dtype=np.float32) * point_conv_factor) - 1.0) * msg.block_edge_length)
            v2 = block_offset + (((np.array([block.x[3*i+1], block.y[3*i+1], block.z[3*i+1]], dtype=np.float32) * point_conv_factor) - 1.0) * msg.block_edge_length)
            v3 = block_offset + (((np.array([block.x[3*i+2], block.y[3*i+2], block.z[3*i+2]], dtype=np.float32) * point_conv_factor) - 1.0) * msg.block_edge_length)
            vertices.extend([v1, v2, v3])

            c1 = (block.r[3*i],   block.g[3*i],   block.b[3*i])
            c2 = (block.r[3*i+1], block.g[3*i+1], block.b[3*i+2])
            c3 = (block.r[3*i+2], block.g[3*i+2], block.b[3*i+2])
            colors.extend([c1, c2, c3])

            faces.append([vertex_index, vertex_index+1, vertex_index+2])
            vertex_index += 3

    if len(vertices) == 0:
        rospy.logwarn("没有数据，无法保存")
        return 

    data_str = []
    data_str.append("COFF")  
    data_str.append(f"{len(vertices)} {len(faces)} 0")  

    for v, c in zip(vertices, colors):
        vertex_line = " ".join([f"{v[i]:.6f}" for i in range(3)]) + " " + " ".join([f"{c[i]:.6f}" for i in range(3)])
        data_str.append(vertex_line)

    for face in faces:
        data_str.append(f"3 {face[0]} {face[1]} {face[2]}")

    send_queue.put('\n'.join(data_str))  
    send_event.set()  

def main():
    global sock
    rospy.init_node('mesh_to_coff_converter')
    rospy.Subscriber("/voxblox_node/mesh", Mesh, mesh_callback)
    
    # 初始化 socket 连接
    sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    sock.connect(("127.0.0.1", 12345))  

    
    # 启动发送线程
    send_thread = threading.Thread(target=send_data_thread, daemon=True)
    send_thread.start()
    
    rospy.spin()

if __name__ == '__main__':
    main()
