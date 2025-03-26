import socket
import threading
import time
import queue
import struct


# 创建一个线程安全的队列，用于存放待发送的数据
data_queue = queue.Queue()

# 创建一个锁，保护socket 发送操作(在多线程环境下，保证同一个时间只有一个线程在操作socket)
send_lock = threading.Lock()

def data_producer():
    """
    主线程或独立线程:模拟数据的产生，将数据放入队列中
    """
    count = 0
    while True:
        data = f"Message {count}"
        print("生产数据:",data)
        data_queue.put(data)
        count += 1
        time.sleep(1) #每秒生产一条数据


def data_sender():
    """
    发送线程:从队列中取出数据，通过TCP socket 发送出去
    发送协议: 先发送4字节长度(使用网络字节序),再发送实际数据
    """
    # 创建 TCP socket 并连接到服务器材
    with socket.socket(socket.AF_INET,socket.SOCK_STREAM) as s:
        s.connect(('localhost',7777))
        while True:
            # 从队列中取出数据(阻塞方式)
            data = data_queue.get()
            # 如果收到特殊的终止信号，可以退出循环
            if data is None :
                break
            encoded_data = data.encode()
            # 使用struct 打包数据长度
            data_length = struct.pack("!I",len(encoded_data))
            # 使用锁保护发送操作
            with send_lock:
                s.sendall(data_length)
                s.sendall(encoded_data)
            print("已发送:",data)



def main():
    # 启动数据生产线程（可以放在主线程中，这里用线程示例）
    producer_thread = threading.Thread(target=data_producer, daemon=True)
    # 启动数据发送线程
    sender_thread = threading.Thread(target=data_sender, daemon=True)
    
    producer_thread.start()
    sender_thread.start()

    producer_thread.join()
    sender_thread.join()


    # try:
    #     # 主线程保持运行状态
    #     while True:
    #         time.sleep(0.1)
    # except KeyboardInterrupt:
    #     print("收到中断信号，准备退出...")
    #     # 发送退出信号给发送线程（可选）
    #     data_queue.put(None)
    #     sender_thread.join()
    #
if __name__ == "__main__":
    main() 
