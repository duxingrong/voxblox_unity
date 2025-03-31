#!/usr/bin/env python3
import rospy
import tf2_ros
import socket
import json
from nav_msgs.msg import Odometry
from geometry_msgs.msg import  Pose,Quaternion
import threading

class OdomSender:
    def __init__(self):
        # 初始化ROS节点
        rospy.init_node("send_odom",anonymous=True)

        # 获取参数服务器中的unity服务器IP地址和端口号
        self.tcp_ip =  rospy.get_param("~server_ip","127.0.0.1")
        self.tcp_port = rospy.get_param("~server_port",8888)

        # 创建TCP套接字
        self.sock = socket.socket(socket.AF_INET,socket.SOCK_STREAM)
        rospy.loginfo(f"正在连接Unity端{self.tcp_ip}:{self.tcp_port}")
        try:
            self.sock.connect((self.tcp_ip,self.tcp_port))
            rospy.loginfo("成功连接到unity服务器")
        except socket.error as e:
            rospy.logerr(f"无法连接到 Unity 服务器: {e}")
            rospy.signal_shutdown("无法连接到 Unity 服务器")
            return

        #订阅/odom话题
        self.odom_sub = rospy.Subscriber("/odom",Odometry,self.odom_callback)

        # 使用话题监听ros消息，防止阻塞
        self.thread = threading.Thread(target=self.ros_spin)
        self.thread.start()


    def ros_spin(self):
        rospy.spin()
        self.sock.close()

    def odom_callback(self,msg):
        # 获取位置信息
        position = msg.pose.pose.position
        orientation = msg.pose.pose.orientation

        # 将位姿信息转化成字典
        pose_data = {
            "position":{
                'x':position.x,
                'y':position.y,
                'z':position.z
            },
            "orientation":{
                'x':orientation.x,
                'y':orientation.y,
                'z':orientation.z,
                'w':orientation.w
            }
        }

        # 将字典转换成JSON字符串
        pose_json = json.dumps(pose_data)

        try:
            self.sock.sendall(pose_json.encode("utf-8"))
            rospy.loginfo(f"已发送位姿数据:{pose_json}")
        except socket.error as e:
            rospy.logerr(f"发送失败:{e}")
            rospy.signal_shutdown("发送数据失败")

if __name__=="__main__":
    try:
        odom_sender = OdomSender()
    except rospy.ROSInterruptException:
        pass



 
