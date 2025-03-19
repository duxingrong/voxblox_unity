#!/usr/bin/env python3

import rospy
from voxblox_msgs.msg import Mesh 
from geometry_msgs.msg import Point32
from std_msgs.msg import ColorRGBA


def mesh_callback(msg):
    size = 0
    print("Received a mesh message!")

    if not msg.mesh_blocks:
        print("Warning: mesh_blocks is empty!")
        return

    # length = msg.block_edge_length
    # print(length)


    
    for block in msg.mesh_blocks:
        #体素块的索引
        index = block.index
        # print(f"Voxel Block Index:{index}")

        vectors = []

        for i in range(len(block.x)):
            vectors.append([  # ✅ 直接添加一个列表，而不是访问不存在的 `vectors[i]`
                block.x[i], block.y[i], block.z[i],
                block.r[i], block.g[i], block.b[i]
            ])
            print(len(msg.mesh_blocks))
            print(len(vectors))
            size += len(vectors)
            print(size)
            print("-"*30)

def listener():
    rospy.init_node('mesh_listener',anonymous=True)
    rospy.Subscriber("/voxblox_node/mesh",Mesh,mesh_callback)
    rospy.spin()

if __name__=="__main__":
    listener()
