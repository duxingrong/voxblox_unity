import socket


"""
UDP 是无连接协议，不需要建立连接，直接使用 sendto() 发送数据，使用 recvfrom() 接收数据，并且需要处理客户端地址

服务端：不断监听消息，收到数据后通过客户端地址将消息返回
""" 

def udp_server():
    # 创建一个UDP socket 对象
    server_socket = socket.socket(socket.AF_INET,socket.SOCK_DGRAM)

    # 绑定IP地址和端口
    server_socket.bind(('localhost',5555))
    print("UDP 服务器正在监听5555端口")

    while True:
        # 接受数据(返回数据和客户端地址)
        data ,addr = server_socket.recvfrom(1024)
        print("来自",addr,"的消息",data.decode())
        #将数据原样返回给客户端
        server_socket.sendto(data,addr)

if __name__=="__main__":
    udp_server()
