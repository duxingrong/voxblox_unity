import socket

def tcp_server():
    # 创建一个TCP socket 对象
    server_socket = socket.socket(socket.AF_INET,socket.SOCK_STREAM)

    # 绑定IP地址和端口
    server_socket.bind(("localhost",4444))

    # 开始监听，允许最多一个等待连接
    server_socket.listen(1)
    print("TCP 服务端正在监听 4444 端口")


    # 等待客户端连接
    conn , addr = server_socket.accept()
    print("连接来自:",addr)

    while True:
        #接受客户端发送的数据，每次最多1024字节
        data = conn.recv(1024)
        if not data:
            break # 没有收到数据，退出循环
        print("受到数据:",data.decode())
        # 将数据原样发送回客户端
        conn.sendall(data)

    # 关闭连接
    conn.close()

if __name__ == "__main__":
    tcp_server()
