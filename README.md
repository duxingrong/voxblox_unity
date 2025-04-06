# voxblox_unity

![voxblox_unity](show.gif)

> [!TIP]
> 该功能包功能是将voxblox产生的/mesh消息在unity中实时可视化,所以首先需要了解[voxblox](https://github.com/ethz-asl/voxblox) 


### machine
ubuntu20.04<br>
ros noetic<br>
unity2022<br>

> [!WARNING]
> 在了解该功能包之前，请首先理解voxblox,以及下载编译成功voxblox


### install 
> [!WARNING]
> 请将voxblox_msgs  放在该包的同级目录中,防止编辑错误
```bash
cd ~/catkin_ws/src
git clone https://github.com/duxingrong/voxblox_unity.git
cd ~/catkin_ws
catkin build voxblox_unity
```


### example
首先将/unity中的OffMesh_socket.cs 挂在unity空物体上,运行unity
```bash
roslaunch voxblox_ros cow_and_lady.launch
roslaunch voxblox_unity  voxblox_unity.launch server_ip:=你的unityIp
```

成功即可看到上述图片所示




