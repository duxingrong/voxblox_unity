#!/usr/bin/env python3
import socket
import struct
import threading

def parse_coff_data(data_str):
    """解析 COFF/OFF/NCOFF 格式数据并返回顶点、法线、颜色及面信息"""
    lines = data_str.strip().split('\n')
    if not lines:
        raise ValueError("Empty data")
    header = lines[0].strip()
    if header not in ("OFF", "COFF", "NCOFF"):
        raise ValueError("Invalid header: " + header)
    
    try:
        v_count, f_count, _ = map(int, lines[1].split())
    except Exception as e:
        raise ValueError("Invalid header line format") from e

    vertices = []
    normals = []   # 仅 NCOFF 时有效
    colors = []
    faces = []
    
    # 根据格式确定每个顶点行应包含的数值数量
    if header == "OFF":
        expected_vertex_values = 3
    elif header == "COFF":
        expected_vertex_values = 6
    elif header == "NCOFF":
        expected_vertex_values = 9

    for line in lines[2:2+v_count]:
        parts = list(map(float, line.split()))
        if len(parts) != expected_vertex_values:
            continue
        # 提取顶点数据
        vertex = (parts[0], parts[1], parts[2])
        vertices.append(vertex)
        if header == "COFF":
            # 后面三个为颜色
            colors.append((parts[3], parts[4], parts[5]))
        elif header == "NCOFF":
            # 中间三个为法线，后面三个为颜色
            normals.append((parts[3], parts[4], parts[5]))
            colors.append((parts[6], parts[7], parts[8]))
    for line in lines[2+v_count:2+v_count+f_count]:
        parts = list(map(int, line.split()))
        # 检查面数据格式：首个数值应为 3，后面为顶点索引
        if len(parts) < 4 or parts[0] != 3:
            continue
        faces.append((parts[1], parts[2], parts[3]))
    
    return header, vertices, normals, colors, faces, v_count, f_count

def handle_client(conn, addr):
    """处理单个客户端连接（持久连接）"""
    print(f"[INFO] Connection established from {addr}")
    
    try:
        while True:
            # 读取前 4 字节，获取数据长度
            header_bytes = conn.recv(4)
            if not header_bytes:
                print("[INFO] 客户端已关闭连接。")
                break
            data_len = struct.unpack('>I', header_bytes)[0]
            print(f"[INFO] 预期数据长度: {data_len}")
            
            # 持续读取数据，直到读取到完整的消息
            data_buffer = b''
            while len(data_buffer) < data_len:
                chunk = conn.recv(data_len - len(data_buffer))
                if not chunk:
                    print("[ERROR] 连接意外关闭。")
                    break
                data_buffer += chunk
            
            if len(data_buffer) < data_len:
                print("[ERROR] 数据不完整，关闭连接。")
                break

            print("[INFO] 数据接收成功。")
            
            # 解析 COFF/OFF/NCOFF 数据并保存到文件
            try:
                header, vertices, normals, colors, faces, v_count, f_count = parse_coff_data(data_buffer.decode('utf-8'))
                print(f"[INFO] 解析 Mesh 成功 - 顶点数: {v_count}, 面数: {f_count}, 格式: {header}")
                
                # 保存为 OFF 文件（覆盖原有文件）
                with open('received_mesh.off', 'w') as f:
                    f.write(f"{header}\n")
                    f.write(f"{v_count} {f_count} 0\n")
                    if header == "NCOFF":
                        # 每个顶点输出： x y z nx ny nz r g b
                        for v, n, c in zip(vertices, normals, colors):
                            f.write(f"{v[0]} {v[1]} {v[2]} {n[0]} {n[1]} {n[2]} {c[0]} {c[1]} {c[2]}\n")
                    elif header == "COFF":
                        # 每个顶点输出： x y z r g b
                        for v, c in zip(vertices, colors):
                            f.write(f"{v[0]} {v[1]} {v[2]} {c[0]} {c[1]} {c[2]}\n")
                    else:  # OFF 格式：仅输出 xyz
                        for v in vertices:
                            f.write(f"{v[0]} {v[1]} {v[2]}\n")
                    for face in faces:
                        f.write(f"3 {face[0]} {face[1]} {face[2]}\n")
                print("[INFO] Mesh 已保存为 received_mesh.off")
            except Exception as e:
                print(f"[ERROR] 解析 COFF 数据失败: {e}")
    except Exception as e:
        print(f"[ERROR] 客户端处理过程中异常: {e}")
    finally:
        conn.close()
        print("[INFO] 连接关闭")

def start_server(host='0.0.0.0', port=12345):
    """启动 TCP 服务器"""
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
        s.bind((host, port))
        s.listen()
        print(f"[INFO] 服务器监听 {host}:{port}")
        
        while True:
            conn, addr = s.accept()
            # 为每个客户端连接启动一个独立的线程
            client_thread = threading.Thread(target=handle_client, args=(conn, addr), daemon=True)
            client_thread.start()

if __name__ == '__main__':
    start_server()
