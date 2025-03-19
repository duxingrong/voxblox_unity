import rospy 
from voxblox_msgs.msg import Mesh 
from visualization_msgs.msg import Marker , MarkerArray
from std_msgs.msg import ColorRGBA 
 
def mesh_callback(msg):
    marker_array = MarkerArray()
    for block in msg.mesh_blocks:
        for i in range(len(block.x)):
            marker = Marker()
            marker.id = i # 确保每个Marker的id唯一
            marker.header.frame_id = "world" # 设定坐标系
            marker.type = Marker.CUBE #体素块可以用立方体表示
            marker.action = Marker.ADD
            marker.pose.position.x = block.x[i]
            marker.pose.position.y = block.y[i]
            marker.pose.position.z = block.z[i]
            marker.pose.orientation.x = 0
            marker.pose.orientation.y = 0
            marker.pose.orientation.z = 0
            marker.pose.orientation.w = 1
            marker.scale.x = 1
            marker.scale.y = 1
            marker.scale.z = 1
            marker.color=ColorRGBA(block.r[i]/255.0,block.g[i]/255.0,block.b[i]/255.0,1.0) # alpha
            marker_array.markers.append(marker)

    # 发布MarkerArray消息
    pub.publish(marker_array)

def listener():
    rospy.init_node('mesh_listener',anonymous=True)
    rospy.Subscriber('/voxblox_node/mesh',Mesh,mesh_callback)
    rospy.spin()

if __name__=="__main__":
    pub= rospy.Publisher('/visualization_marker_array',MarkerArray,queue_size=10)
    listener()
