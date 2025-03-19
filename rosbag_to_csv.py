import bagpy
from bagpy import bagreader 
import json  

# 读取bag文件
bag  = bagreader('/home/du/one_message.bag')


# 获取所有的话题
print(bag.topic_table)

# 选择话题名称提取数据
data = bag.message_by_topic('/voxblox_node/mesh')


print("数据保存为output.json")
