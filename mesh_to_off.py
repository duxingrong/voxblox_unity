#!/usr/bin/env python3

import rospy
from voxblox_msgs.msg import Mesh
import numpy as np

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
        # 计算当前块在全局坐标系中的偏移量（假定 block.index 表示块的三维索引）
        block_offset = np.array(block.index, dtype=np.float32) * msg.block_edge_length

        # 假设各数组长度相同，每三个点构成一个三角面
        num_vertices = len(block.x)
        num_triangles = num_vertices // 3

        for i in range(num_triangles):
            # 解码三个顶点的坐标
            v1 = block_offset + (((np.array([block.x[3*i],   block.y[3*i],   block.z[3*i]],   dtype=np.float32) * point_conv_factor) - 1.0) * msg.block_edge_length)
            v2 = block_offset + (((np.array([block.x[3*i+1], block.y[3*i+1], block.z[3*i+1]], dtype=np.float32) * point_conv_factor) - 1.0) * msg.block_edge_length)
            v3 = block_offset + (((np.array([block.x[3*i+2], block.y[3*i+2], block.z[3*i+2]], dtype=np.float32) * point_conv_factor) - 1.0) * msg.block_edge_length)
            vertices.extend([v1, v2, v3])

            # 提取每个顶点的颜色信息 (颜色范围通常为 0-255)
            c1 = (block.r[3*i],   block.g[3*i],   block.b[3*i])
            c2 = (block.r[3*i+1], block.g[3*i+1], block.b[3*i+2])
            c3 = (block.r[3*i+2], block.g[3*i+2], block.b[3*i+2])
            colors.extend([c1, c2, c3])

            # 保存三角面索引（每三个顶点构成一个三角形）
            faces.append([vertex_index, vertex_index+1, vertex_index+2])
            vertex_index += 3


    if len(vertices)==0:
        rospy.logwarn("没有数据，无法保存")
        return 

    # 将提取的网格信息写入 COFF 文件（文件保存为 output.off）
    with open('output.off', 'w') as f:
        # COFF 格式第一行
        f.write("COFF\n")
        # 第二行：顶点数、面数、边数（边数可以填 0）
        f.write("{} {} 0\n".format(len(vertices), len(faces)))
        # 每个顶点一行：x y z r g b
        for v, c in zip(vertices, colors):
            f.write("{} {} {} {} {} {}\n".format(v[0], v[1], v[2], c[0], c[1], c[2]))
        # 接下来写入每个面：先写顶点数（这里固定为3），再写对应的顶点索引
        for face in faces:
            f.write("3 {} {} {}\n".format(face[0], face[1], face[2]))

    rospy.loginfo("COFF 文件已保存为 output.off")

def main():
    rospy.init_node('mesh_to_coff_converter')
    rospy.Subscriber("/voxblox_node/mesh", Mesh, mesh_callback)
    rospy.spin()

if __name__ == '__main__':
    main()
