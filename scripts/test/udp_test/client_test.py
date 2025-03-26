import socket 
import time

"""
UDP 是无连接协议，不需要建立连接，直接使用 sendto() 发送数据，使用 recvfrom() 接收数据，并且需要处理客户端地址

客户端：直接发送消息，并等待接收服务端的回复
""" 


def udp_client():
    # 创建一个UDP socket 对象
    client_socket = socket.socket(socket.AF_INET,socket.SOCK_DGRAM)

    server_address = ('localhost',5555)

    while True:
        message = "Hello, UDP 服务端!"

        # 发送数据到服务端
        client_socket.sendto(message.encode(),server_address)

        #接受服务端返回的数据
        data ,server = client_socket.recvfrom(1024)

        print("从服务器接受的数据:",data.decode())

        time.sleep(2)

    # 关闭socket
    client_socket.close() 

if __name__=="__main__":
    udp_client()
