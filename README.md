# voxblox_unity


> [!TIP]
> 该功能包功能是将voxblox产生的/mesh消息在unity中实时可视化,所以首先需要了解[voxblox](https://github.com/ethz-asl/voxblox) 


### machine
ubuntu20.04<br>
ros noetic<br>
unity2020<br>

> [!WARNING]
> 在了解该功能包之前，请首先理解voxblox,以及下载编译成功voxblox

### install 
```bash
cd ~/catkin_ws/src
git clone https://github.com/duxingrong/voxblox_unity.git
cd ~/catkin_ws
catkin build voxblox_unity
```


### example
> [!TIP]
> 这个例子依赖仿真环境,请按照下列命令执行,该仿真来源于[wpr_simulation](https://github.com/6-robot/wpr_simulation)

#### 仿真环境
```bash
cd ~/catkin_ws/src/
git clone https://github.com/6-robot/wpr_simulation.git
git clone https://github.com/6-robot/wpb_home.git
git clone https://github.com/6-robot/waterplus_map_tools.git

# 安装依赖项
cd ~/catkin_ws/src/wpr_simulation/scripts
./install_for_noetic.sh
cd ~/catkin_ws/src/wpb_home/wpb_home_bringup/scripts
./install_for_noetic.sh
cd ~/catkin_ws/src/waterplus_map_tools/scripts
./install_for_noetic.sh

# 编译
cd ~/catkin_ws
catkin build
```

#### 代码示例
仿真环境+gmaping+nav 示例:<br>
将test.launch文件放入wpr_simulation/launch中
```bash
<launch>

  <!-- 载入仿真环境 -->
  <include file="$(find wpr_simulation)/launch/wpb_stage_robocup.launch"/>

  <!-- 启动 gmapping 节点，实现在线建图 -->
  <node pkg="gmapping" type="slam_gmapping" name="slam_gmapping" output="screen"/>

  <!-- 启动 move_base 节点，利用 gmapping 发布的 /map 进行导航 -->
  <node pkg="move_base" type="move_base" name="move_base" output="screen">
    <!-- 加载全局和局部 costmap 的公共参数 -->
    <rosparam file="$(find wpb_home_tutorials)/nav_lidar/costmap_common_params.yaml" command="load" ns="global_costmap"/>
    <rosparam file="$(find wpb_home_tutorials)/nav_lidar/costmap_common_params.yaml" command="load" ns="local_costmap"/>
    <!-- 加载全局和局部 costmap 的专用参数 -->
    <rosparam file="$(find wpb_home_tutorials)/nav_lidar/global_costmap_params.yaml" command="load" />
    <rosparam file="$(find wpb_home_tutorials)/nav_lidar/local_costmap_params.yaml" command="load" />
    <!-- 规划器设置 -->
    <param name="base_global_planner" value="global_planner/GlobalPlanner" /> 
    <param name="base_local_planner" value="wpbh_local_planner/WpbhLocalPlanner" />
  </node>

  <!-- 启动 rviz 进行可视化 -->
  <arg name="rvizconfig" default="$(find wpr_simulation)/rviz/nav.rviz" />
  <node pkg="rviz" type="rviz" name="rviz" args="-d $(arg rvizconfig)" />

</launch>
```

voxblox_ros示例
将car_true.launch文件放入voxblox_ros/launch中
```bash

<?xml version="1.0" encoding="UTF-8"?>
<launch>
  <!-- 启动 voxblox 的 tsdf_server 节点 -->
  <node name="voxblox_node" pkg="voxblox_ros" type="tsdf_server" output="screen" args="-alsologtostderr" clear_params="true">
    <!-- 将默认的“pointcloud”话题重映射为相机点云话题（请根据实际情况修改话题名称） -->
    <remap from="pointcloud" to="/kinect2/sd/points"/>
    
    <!-- TSDF 参数 -->
    <param name="tsdf_voxel_size" value="0.1" /> 
    <param name="truncation_distance" value="0.5" />
    
    <!-- ✅ 开启颜色支持 -->
    <param name="color_mode" value="color" />
    <param name="enable_color" value="true" />
    
    <!-- ICP 参数 -->
    <param name="enable_icp" value="true" />
    <param name="icp_refine_roll_pitch" value="false" />
    
    <!-- Mesh 更新参数 -->
    <param name="update_mesh_every_n_sec" value="1.0" />
    <param name="mesh_min_weight" value="2" />
    <param name="method" value="fast" />
    <param name="max_ray_length_m" value="10.0" />
    
    <!-- 设置 world_frame :不动点，一般设置为map -->
    <param name="world_frame" value="map"/>
    <param name="verbose" value="true" />
    <param name="use_tf_transforms" value="true" />
    
    <!-- ✅ 确保输出的 mesh 文件支持颜色 -->
    <param name="mesh_filename" value="$(find voxblox_ros)/mesh_results/$(anon camera_mesh).ply" />
  </node>

</launch>
```


#### 具体步骤
1. 首先请在电脑上验证off文件的有效性
```bash
roslaunch wpr_simulation test.launch
roslaunch voxblox_ros car_true.launch
rosrun voxblox_unity socket_test.py
rosrun voxblox_unity accumulate_mesh_to_off_node
```
得到received_mesh.off后，利用meshlab 或者其他方式打开，查看是否有效

2. 在unity中可视化
- 在unity文件夹中,其中OffMeshLoader.cs可以在unity中打开off文件,仅用来调试(可以忽略)

- 将另外两个脚本挂在空物体上后,启动unity,查看Console是否在监听端口12345以及9000

- 以上都正确后，执行下列代码

```bash
roslaunch wpr_simulation test.launch
roslaunch voxblox_ros car_true.launch
roslaunch voxblox_unity voxblox_unity.launch server_ip:=你的unity所在电脑的ip
```

- 在rviz中， 使用2d Goal nav,移动机器人，观察unity中，可视化动态地图以及坐标系即成功!








