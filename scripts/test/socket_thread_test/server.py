import socket
import struct



def recall(sock,n):
    """
    辅助函数,确保接收制定长度n 的数据
    """

    data =b''
    while len(data)<n:
        packet = sock.recv(n-len(data))
        if not packet:
            return None
        data += packet

    return data


def server():
    # 创建TCP socket 并绑定端口
    with socket.socket(socket.AF_INET,socket.SOCK_STREAM) as s:
        s.bind(('localhost',7777))
        s.listen(1)
        print("服务器正在监听 7777 端口")
        conn , addr = s.accept()
        with conn:
            print("与客户端建立连接，地址:",addr)
            while True:
                # 先接受4字节数据长度
                raw_msglen = recall(conn ,4)
                if not raw_msglen:
                    print("客户端断开连接")
                    break
                msglen = struct.unpack('!I',raw_msglen)[0]
                # 根据接受到的长度，在接受具体的数据内容
                data = recall(conn , msglen)
                if data is None :
                    print("连接异常中断")
                    break
                print("接受到的数据:",data.decode())

if __name__ == "__main__":
    server()
