#!/usr/bin/env python3
import rospy
import tf2_ros
import socket
import json

def send_tf_data():
    # 使用与 launch 文件中一致的节点名称，或使用匿名节点但注意私有参数解析
    rospy.init_node('tf_show_unity', anonymous=True)
    # 这里使用 "~server_ip" 获取私有参数
    tcp_ip = rospy.get_param("~server_ip", "127.0.0.1")  
    tcp_port = 9000  # 端口可以参数化

    # 建立 TCP 连接
    sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    rospy.loginfo("正在连接 Unity 端 {}:{}".format(tcp_ip, tcp_port))
    sock.connect((tcp_ip, tcp_port))
    
    # 初始化 tf2 相关组件
    tfBuffer = tf2_ros.Buffer()
    listener = tf2_ros.TransformListener(tfBuffer)
    
    rate = rospy.Rate(10)  # 每秒 10 次
    frames_to_send = ["base_footprint", "map"]
    
    while not rospy.is_shutdown():
        for frame in frames_to_send:
            try:
                transform = tfBuffer.lookup_transform("map", frame, rospy.Time(0), rospy.Duration(1.0))
                data = {
                    "frame_id": frame,
                    "translation": {
                        "x": transform.transform.translation.x,
                        "y": transform.transform.translation.y,
                        "z": transform.transform.translation.z
                    },
                    "rotation": {
                        "x": transform.transform.rotation.x,
                        "y": transform.transform.rotation.y,
                        "z": transform.transform.rotation.z,
                        "w": transform.transform.rotation.w
                    }
                }
                json_data = json.dumps(data) + "\n"
                sock.sendall(json_data.encode('utf-8'))
                rospy.loginfo("发送数据: " + json_data)
            except Exception as e:
                rospy.logwarn("无法获取坐标系 {} 的 tf: {}".format(frame, e))
        rate.sleep()

if __name__ == '__main__':
    try:
        send_tf_data()
    except rospy.ROSInterruptException:
        pass
