import socket
import time

def tcp_client():
    # 创建一个TCP socket 对象
    client_socket = socket.socket(socket.AF_INET,socket.SOCK_STREAM)

    # 连接到服务器
    client_socket.connect(('localhost',4444))

    while True:
        message = "Hello , TCP 服务端!"

        # 发送数据
        client_socket.sendall(message.encode())


        # 接受服务器返回的数据
        data = client_socket.recv(1024)
        print("从服务器接受的数据:",data.decode())

        time.sleep(2)


    # 关闭连接
    client_socket.close()

if __name__=="__main__":
    tcp_client()
