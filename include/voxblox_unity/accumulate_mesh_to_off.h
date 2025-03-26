#ifndef ACCUMULATE_MESH_TO_OFF_H
#define ACCUMULATE_MESH_TO_OFF_H

#include "voxblox_msgs/Mesh.h"
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <array>
#include <atomic>
#include <cmath>
#include <condition_variable>
#include <fstream>
#include <functional>
#include <iomanip>
#include <mutex>
#include <queue>
#include <ros/ros.h>
#include <sstream>
#include <string>
#include <thread>
#include <tuple>
#include <unordered_map>
#include <vector>

// 自定义哈希函数：用于 std::tuple<int64_t,int64_t,int64_t>
struct TupleHash {
  std::size_t operator()(const std::tuple<int64_t, int64_t, int64_t> &t) const;
};

// 自定义哈希函数：用于 std::array<int,3>
struct ArrayHash {
  std::size_t operator()(const std::array<int, 3> &arr) const;
};

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

  // 保存全局地图到 OFF 文件
  void saveGlobalMap(bool includeNormals);

  // 初始化 socket 连接
  bool initSocket(const std::string &ip, int port);

private:
  // 线程函数：从发送队列中取数据通过 socket 发送
  void sendDataThread();

  // 计算顶点法线：累加每个面的法线，再归一化
  void computeVertexNormals(const std::vector<Eigen::Vector3d> &vertices,
                            const std::vector<std::array<int, 3>> &faces,
                            std::vector<Eigen::Vector3d> &vertexNormals);

  // 合并顶点并重建面索引（类似于 voxblox 的 createConnectedMesh）
  void createConnectedMesh(const std::vector<Eigen::Vector3d> &vertices,
                           const std::vector<std::array<int, 3>> &faces,
                           const std::vector<Eigen::Vector3d> &colors,
                           const std::vector<Eigen::Vector3d> &normals,
                           double tol,
                           std::vector<Eigen::Vector3d> &newVertices,
                           std::vector<std::array<int, 3>> &newFaces,
                           std::vector<Eigen::Vector3d> &newColors,
                           std::vector<Eigen::Vector3d> &newNormals);

  // 将全局地图数据（各 block 数据）融合为整体数据
  void accumulateGlobalMesh(std::vector<Eigen::Vector3d> &globalVertices,
                            std::vector<std::array<int, 3>> &globalFaces,
                            std::vector<Eigen::Vector3d> &globalColors);

  ros::NodeHandle nh_;
  int sockfd;

  // 用于发送数据的队列与线程
  std::mutex queueMutex;
  std::condition_variable sendCond;
  std::queue<std::string> sendQueue;
  std::thread sendThread;
  std::atomic<bool> shutdownFlag;

  // 全局地图数据（各 mesh block），加锁保护
  std::mutex globalMeshMutex;
  std::unordered_map<std::array<int, 3>, MeshBlockData, ArrayHash>
      globalMeshMap;

  // 参数：合并容差与输出文件路径
  double merge_tol_;
  std::string output_file_;
};

#endif // ACCUMULATE_MESH_TO_OFF_H
