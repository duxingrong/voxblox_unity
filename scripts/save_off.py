#!/usr/bin/env python
import rospy
from nvblox_msgs.msg import Mesh
import os

def callback(msg):
    """
    接受数据,保存为off格式
    """
    vertices = []
    colors = []
    normals = []
    triangles = []
    block_size = msg.block_size
    
    # 记录顶点的总数，用于构建三角形索引
    vertex_count = 0
    
    for block in msg.blocks:
        if len(block.vertices) > 0:
            # 将顶点，法线，颜色加入
            block_vertices = [(v.x, v.y, v.z) for v in block.vertices]
            block_normals = [(n.x, n.y, n.z) for n in block.normals]
            
            # 添加顶点和法线
            vertices.extend(block_vertices)
            normals.extend(block_normals)
            
            # 添加颜色（如果有）
            if len(block.colors) > 0:
                block_colors = [(c.r, c.g, c.b) for c in block.colors]
                colors.extend(block_colors)
            
            # 处理三角形索引
            for i in range(len(block.triangles) // 3):
                base_idx = i * 3
                # 添加三角形，注意调整索引以适应全局顶点列表
                triangles.append((
                    block.triangles[base_idx] + vertex_count,
                    block.triangles[base_idx + 1] + vertex_count,
                    block.triangles[base_idx + 2] + vertex_count
                ))
            
            # 更新顶点计数
            vertex_count += len(block.vertices)
    
    # 保存为OFF格式
    save_to_off(vertices, triangles, colors)

def save_to_off(vertices, triangles, colors=None):
    """
    将网格数据保存为OFF格式
    """
#   filename = f"mesh_{rospy.Time.now().to_sec()}.off"
    filename = f"mesh.off"
    
    with open(filename, "w") as f:
        # 文件头
        if colors and len(colors) == len(vertices):
            f.write("COFF\n")
        else:
            f.write("OFF\n")
        
        # 写入顶点数，面数，边数（边数通常为0，由软件计算）
        f.write(f"{len(vertices)} {len(triangles)} 0\n")
        
        # 写入顶点
        for i, (x, y, z) in enumerate(vertices):
            if colors and i < len(colors):
                r, g, b = colors[i]
                f.write(f"{x} {y} {z} {r} {g} {b}\n")
            else:
                f.write(f"{x} {y} {z}\n")
        
        # 写入面
        for t1, t2, t3 in triangles:
            f.write(f"3 {t1} {t2} {t3}\n")
    
    rospy.loginfo(f"Mesh saved to {filename}")

if __name__ == "__main__":
    rospy.init_node("save_off")
    nvblox_sub = rospy.Subscriber("/nvblox_node/mesh", Mesh, callback)
    rospy.loginfo("Subscribed to /nvblox_node/mesh, waiting for messages...")
    rospy.spin()
