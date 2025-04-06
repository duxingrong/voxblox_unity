#ifndef ACCUMULATE_MESH_TO_OFF_H
#define ACCUMULATE_MESH_TO_OFF_H

#include "voxblox_msgs/Mesh.h"
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <arpa/inet.h>
#include <array>
#include <atomic>
#include <cmath>
#include <condition_variable>
#include <fstream>
#include <functional>
#include <iomanip>
#include <mutex>
#include <netinet/in.h>
#include <queue>
#include <ros/ros.h>
#include <sstream>
#include <string>
#include <sys/socket.h>
#include <thread>
#include <tuple>
#include <unistd.h>
#include <unordered_map>
#include <vector>

// 保存每个 mesh block 的数据结构
struct MeshBlockData {
  std::vector<Eigen::Vector3d> vertices;
  std::vector<std::array<int, 3>> faces;
  std::vector<Eigen::Vector3d> colors;
};

class MeshAccumulator {
public:
  MeshAccumulator(ros::NodeHandle &nh);
  ~MeshAccumulator();

  // 回调函数，处理订阅的 voxblox mesh 消息
  void meshCallback(const voxblox_msgs::Mesh::ConstPtr &msg);

  // 初始化 socket 连接
  bool initSocket(const std::string &ip, int port);

private:
  // 线程函数：从发送队列中取数据通过 socket 发送
  void sendDataThread();

  // 计算顶点法线：累加每个面的法线，再归一化
  void computeVertexNormals(const std::vector<Eigen::Vector3d> &vertices,
                            const std::vector<std::array<int, 3>> &faces,
                            std::vector<Eigen::Vector3d> &vertexNormals);

  ros::NodeHandle nh_;
  int sockfd;

  // 用于发送数据的队列与线程
  std::mutex queueMutex;
  std::condition_variable sendCond;
  std::queue<std::string> sendQueue;
  std::thread sendThread;
  std::atomic<bool> shutdownFlag;
};

#endif // ACCUMULATE_MESH_TO_OFF_H
