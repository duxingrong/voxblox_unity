#!/usr/bin/env python3
import socket
import struct
import sys

def parse_coff_data(data_str):
    """解析 COFF 格式数据并返回顶点/面信息"""
    lines = data_str.strip().split('\n')
    if not lines or lines[0] != "COFF":
        raise ValueError("Invalid COFF header")
    
    try:
        v_count, f_count, _ = map(int, lines[1].split())
    except:
        raise ValueError("Invalid header line format")

    vertices, colors, faces = [], [], []
    for line in lines[2:2+v_count]:
        parts = list(map(float, line.split()))
        if len(parts) != 6:
            continue
        vertices.append((parts[0], parts[1], parts[2]))
        colors.append((parts[3], parts[4], parts[5]))
    
    for line in lines[2+v_count:2+v_count+f_count]:
        parts = list(map(int, line.split()))
        if len(parts) != 4 or parts[0] != 3:
            continue
        faces.append((parts[1], parts[2], parts[3]))
    
    return vertices, colors, faces, v_count, f_count

def handle_client(conn, addr):
    """处理单个客户端连接"""
    print(f"[INFO] Connection established from {addr}")
    
    try:
        # 读取前 4 字节，获取数据长度
        header = conn.recv(4)
        if not header:
            print("[ERROR] Failed to receive data length")
            return
        data_len = struct.unpack('>I', header)[0]
        print(f"[INFO] Expected data length: {data_len}")
        
        # 读取数据
        data_buffer = b''
        while len(data_buffer) < data_len:
            chunk = conn.recv(data_len - len(data_buffer))
            if not chunk:
                print("[ERROR] Connection closed prematurely")
                return
            data_buffer += chunk
        
        print("[INFO] Data received successfully")
        
        # 解析 COFF 数据
        try:
            vertices, colors, faces, v_count, f_count = parse_coff_data(data_buffer.decode('utf-8'))
            print(f"[INFO] Parsed Mesh - Vertices: {v_count}, Faces: {f_count}")
            
            # 保存为 OFF 文件
            with open('received_mesh.off', 'w') as f:
                f.write("COFF\n")
                f.write(f"{v_count} {f_count} 0\n")
                for v, c in zip(vertices, colors):
                    f.write(f"{v[0]} {v[1]} {v[2]} {c[0]} {c[1]} {c[2]}\n")
                for face in faces:
                    f.write(f"3 {face[0]} {face[1]} {face[2]}\n")
            print("[INFO] Mesh saved as received_mesh.off")
            
        except Exception as e:
            print(f"[ERROR] Failed to parse COFF data: {e}")
    
    except Exception as e:
        print(f"[ERROR] Exception in client handling: {e}")
    
    finally:
        conn.close()
        print("[INFO] Connection closed")

def start_server(host='0.0.0.0', port=12345):
    """启动 TCP 服务器"""
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
        s.bind((host, port))
        s.listen()
        print(f"[INFO] Server listening on {host}:{port}")
        
        while True:
            conn, addr = s.accept()
            handle_client(conn, addr)

if __name__ == '__main__':
    start_server()
