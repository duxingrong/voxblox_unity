#include "voxblox_unity/accumulate_mesh_to_off.h"

#include <arpa/inet.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <unistd.h>

// 实现 TupleHash
std::size_t
TupleHash::operator()(const std::tuple<int64_t, int64_t, int64_t> &t) const {
  auto h1 = std::hash<int64_t>{}(std::get<0>(t));
  auto h2 = std::hash<int64_t>{}(std::get<1>(t));
  auto h3 = std::hash<int64_t>{}(std::get<2>(t));
  return h1 ^ (h2 << 1) ^ (h3 << 2);
}

// 实现 ArrayHash
std::size_t ArrayHash::operator()(const std::array<int, 3> &arr) const {
  return std::hash<int>{}(arr[0]) ^ (std::hash<int>{}(arr[1]) << 1) ^
         (std::hash<int>{}(arr[2]) << 2);
}

MeshAccumulator::MeshAccumulator(ros::NodeHandle &nh)
    : nh_(nh), sockfd(-1), shutdownFlag(false) {
  // 通过 ROS 参数服务器加载参数
  nh_.param("merge_tol", merge_tol_, 1e-10);
  nh_.param("output_file", output_file_, std::string("global_map.off"));

  // 启动发送数据线程
  sendThread = std::thread(&MeshAccumulator::sendDataThread, this);
}

MeshAccumulator::~MeshAccumulator() {
  shutdownFlag.store(true);
  sendCond.notify_all();
  if (sendThread.joinable())
    sendThread.join();
  if (sockfd != -1)
    close(sockfd);
}

void MeshAccumulator::computeVertexNormals(
    const std::vector<Eigen::Vector3d> &vertices,
    const std::vector<std::array<int, 3>> &faces,
    std::vector<Eigen::Vector3d> &vertexNormals) {
  vertexNormals.assign(vertices.size(), Eigen::Vector3d::Zero());
  for (const auto &face : faces) {
    int i0 = face[0], i1 = face[1], i2 = face[2];
    Eigen::Vector3d v0 = vertices[i0];
    Eigen::Vector3d v1 = vertices[i1];
    Eigen::Vector3d v2 = vertices[i2];
    Eigen::Vector3d face_normal = (v1 - v0).cross(v2 - v0);
    double norm = face_normal.norm();
    if (norm > 1e-8) {
      face_normal /= norm;
    }
    vertexNormals[i0] += face_normal;
    vertexNormals[i1] += face_normal;
    vertexNormals[i2] += face_normal;
  }
  for (auto &n : vertexNormals) {
    double n_norm = n.norm();
    if (n_norm > 1e-8)
      n /= n_norm;
    else
      n = Eigen::Vector3d(0.0, 0.0, 1.0);
  }
}

void MeshAccumulator::createConnectedMesh(
    const std::vector<Eigen::Vector3d> &vertices,
    const std::vector<std::array<int, 3>> &faces,
    const std::vector<Eigen::Vector3d> &colors,
    const std::vector<Eigen::Vector3d> &normals, double tol,
    std::vector<Eigen::Vector3d> &newVertices,
    std::vector<std::array<int, 3>> &newFaces,
    std::vector<Eigen::Vector3d> &newColors,
    std::vector<Eigen::Vector3d> &newNormals) {
  double threshold_inv = 1.0 / tol;
  std::unordered_map<std::tuple<int64_t, int64_t, int64_t>, int, TupleHash>
      vertex_map;
  std::vector<Eigen::Vector3d> mergedVertices;
  std::vector<Eigen::Vector3d> mergedColors;
  std::vector<Eigen::Vector3d> mergedNormals;
  std::vector<int> index_map(vertices.size(), -1);

  for (size_t i = 0; i < vertices.size(); ++i) {
    const auto &v = vertices[i];
    int64_t k1 = static_cast<int64_t>(std::round(v.x() * threshold_inv));
    int64_t k2 = static_cast<int64_t>(std::round(v.y() * threshold_inv));
    int64_t k3 = static_cast<int64_t>(std::round(v.z() * threshold_inv));
    auto key = std::make_tuple(k1, k2, k3);
    auto it = vertex_map.find(key);
    if (it == vertex_map.end()) {
      int new_index = mergedVertices.size();
      vertex_map[key] = new_index;
      mergedVertices.push_back(v);
      if (!colors.empty())
        mergedColors.push_back(colors[i]);
      if (!normals.empty())
        mergedNormals.push_back(normals[i]);
      index_map[i] = new_index;
    } else {
      int new_index = it->second;
      if (!normals.empty())
        mergedNormals[new_index] += normals[i];
      index_map[i] = new_index;
    }
  }

  // 重建面索引，舍弃退化面
  std::vector<std::array<int, 3>> tmpFaces;
  for (const auto &face : faces) {
    std::array<int, 3> new_face = {index_map[face[0]], index_map[face[1]],
                                   index_map[face[2]]};
    if (new_face[0] != new_face[1] && new_face[1] != new_face[2] &&
        new_face[0] != new_face[2])
      tmpFaces.push_back(new_face);
  }
  if (!normals.empty()) {
    for (auto &n : mergedNormals) {
      double n_norm = n.norm();
      if (n_norm > 1e-8)
        n /= n_norm;
      else
        n = Eigen::Vector3d(0.0, 0.0, 1.0);
    }
  }
  newVertices = mergedVertices;
  newFaces = tmpFaces;
  if (!colors.empty())
    newColors = mergedColors;
  if (!normals.empty())
    newNormals = mergedNormals;
}

void MeshAccumulator::accumulateGlobalMesh(
    std::vector<Eigen::Vector3d> &globalVertices,
    std::vector<std::array<int, 3>> &globalFaces,
    std::vector<Eigen::Vector3d> &globalColors) {
  std::lock_guard<std::mutex> lock(globalMeshMutex);
  globalVertices.clear();
  globalFaces.clear();
  globalColors.clear();
  int offset = 0;
  for (const auto &pair : globalMeshMap) {
    const MeshBlockData &block = pair.second;
    for (const auto &face : block.faces) {
      globalFaces.push_back(
          {face[0] + offset, face[1] + offset, face[2] + offset});
    }
    globalVertices.insert(globalVertices.end(), block.vertices.begin(),
                          block.vertices.end());
    globalColors.insert(globalColors.end(), block.colors.begin(),
                        block.colors.end());
    offset += block.vertices.size();
  }
}

void MeshAccumulator::meshCallback(const voxblox_msgs::Mesh::ConstPtr &msg) {
  // 点转换因子
  double point_conv_factor = 2.0 / 65535.0;
  {
    std::lock_guard<std::mutex> lock(globalMeshMutex);
    // 遍历每个 mesh block
    for (const auto &block : msg->mesh_blocks) {
      std::array<int, 3> block_key = {static_cast<int>(block.index[0]),
                                      static_cast<int>(block.index[1]),
                                      static_cast<int>(block.index[2])};
      size_t num_vertices = block.x.size();
      if (num_vertices == 0) {
        globalMeshMap.erase(block_key);
        continue;
      }
      MeshBlockData blockData;
      Eigen::Vector3d block_offset(block.index[0] * msg->block_edge_length,
                                   block.index[1] * msg->block_edge_length,
                                   block.index[2] * msg->block_edge_length);
      for (size_t i = 0; i < num_vertices; ++i) {
        Eigen::Vector3d q(block.x[i], block.y[i], block.z[i]);
        Eigen::Vector3d vertex =
            block_offset + (q * point_conv_factor) * msg->block_edge_length;
        blockData.vertices.push_back(vertex);
        if (block.r.size() > i)
          blockData.colors.push_back(
              Eigen::Vector3d(block.r[i], block.g[i], block.b[i]));
        else
          blockData.colors.push_back(Eigen::Vector3d(0.0, 0.0, 0.0));
      }
      size_t num_triangles = num_vertices / 3;
      for (size_t i = 0; i < num_triangles; ++i) {
        int base = i * 3;
        blockData.faces.push_back({base, base + 1, base + 2});
      }
      globalMeshMap[block_key] = blockData;
    }
  }

  // 融合全局地图数据
  std::vector<Eigen::Vector3d> globalVertices;
  std::vector<std::array<int, 3>> globalFaces;
  std::vector<Eigen::Vector3d> globalColors;
  accumulateGlobalMesh(globalVertices, globalFaces, globalColors);

  // 计算全局顶点法线
  std::vector<Eigen::Vector3d> normals;
  computeVertexNormals(globalVertices, globalFaces, normals);

  // 使用合并算法生成连通网格（使用 ROS 参数中设定的容差 merge_tol_）
  std::vector<Eigen::Vector3d> mergedVertices;
  std::vector<std::array<int, 3>> mergedFaces;
  std::vector<Eigen::Vector3d> mergedColors;
  std::vector<Eigen::Vector3d> mergedNormals;
  createConnectedMesh(globalVertices, globalFaces, globalColors, normals,
                      merge_tol_, mergedVertices, mergedFaces, mergedColors,
                      mergedNormals);

  // 生成带法线和颜色的 OFF 格式字符串（NCOFF）
  std::stringstream ss;
  ss << "NCOFF\n"
     << mergedVertices.size() << " " << mergedFaces.size() << " 0\n";
  constexpr double norm_factor = 1.0 / 255.0;
  for (size_t i = 0; i < mergedVertices.size(); ++i) {
    const Eigen::Vector3d &v = mergedVertices[i];
    const Eigen::Vector3d &n = mergedNormals[i];
    const Eigen::Vector3d &c = mergedColors[i];
    ss << std::fixed << std::setprecision(6) << v.x() << " " << v.y() << " "
       << v.z() << " " << n.x() << " " << n.y() << " " << n.z() << " "
       << c.x() * norm_factor << " " << c.y() * norm_factor << " "
       << c.z() * norm_factor << "\n";
  }
  for (const auto &face : mergedFaces) {
    ss << "3 " << face[0] << " " << face[1] << " " << face[2] << "\n";
  }
  {
    std::lock_guard<std::mutex> lock(queueMutex);
    sendQueue.push(ss.str());
  }
  sendCond.notify_one();
}

void MeshAccumulator::saveGlobalMap(bool includeNormals) {
  std::vector<Eigen::Vector3d> globalVertices;
  std::vector<std::array<int, 3>> globalFaces;
  std::vector<Eigen::Vector3d> globalColors;
  accumulateGlobalMesh(globalVertices, globalFaces, globalColors);
  if (globalVertices.empty()) {
    ROS_WARN("全局地图数据为空！");
    return;
  }
  std::vector<Eigen::Vector3d> normals;
  if (includeNormals) {
    computeVertexNormals(globalVertices, globalFaces, normals);
  }
  std::vector<Eigen::Vector3d> mergedVertices;
  std::vector<std::array<int, 3>> mergedFaces;
  std::vector<Eigen::Vector3d> mergedColors;
  std::vector<Eigen::Vector3d> mergedNormals;
  createConnectedMesh(
      globalVertices, globalFaces, globalColors,
      (includeNormals ? normals : std::vector<Eigen::Vector3d>()), merge_tol_,
      mergedVertices, mergedFaces, mergedColors, mergedNormals);
  std::string header = includeNormals ? "NCOFF" : "COFF";
  std::stringstream ss;
  ss << header << "\n"
     << mergedVertices.size() << " " << mergedFaces.size() << " 0\n";
  for (size_t i = 0; i < mergedVertices.size(); ++i) {
    const Eigen::Vector3d &v = mergedVertices[i];
    if (includeNormals) {
      const Eigen::Vector3d &n = mergedNormals[i];
      const Eigen::Vector3d &c = mergedColors[i];
      ss << std::fixed << std::setprecision(6) << v.x() << " " << v.y() << " "
         << v.z() << " " << n.x() << " " << n.y() << " " << n.z() << " "
         << c.x() << " " << c.y() << " " << c.z() << "\n";
    } else {
      const Eigen::Vector3d &c = mergedColors[i];
      ss << std::fixed << std::setprecision(6) << v.x() << " " << v.y() << " "
         << v.z() << " " << c.x() << " " << c.y() << " " << c.z() << "\n";
    }
  }
  for (const auto &face : mergedFaces) {
    ss << "3 " << face[0] << " " << face[1] << " " << face[2] << "\n";
  }
  std::ofstream ofs(output_file_);
  if (!ofs) {
    ROS_ERROR("无法写入 %s 文件", output_file_.c_str());
    return;
  }
  ofs << ss.str();
  ofs.close();
  ROS_INFO("全局地图已保存为 %s", output_file_.c_str());
}

void MeshAccumulator::sendDataThread() {
  while (ros::ok() && !shutdownFlag.load()) {
    std::unique_lock<std::mutex> lock(queueMutex);
    sendCond.wait(lock,
                  [this] { return !sendQueue.empty() || shutdownFlag.load(); });
    if (shutdownFlag.load())
      break;
    while (!sendQueue.empty()) {
      std::string data_str = sendQueue.front();
      sendQueue.pop();
      lock.unlock(); // 释放锁后发送数据

      // 先发送4字节数据长度（网络字节序）
      uint32_t len = data_str.size();
      uint32_t len_net = htonl(len);
      ssize_t sent = send(sockfd, &len_net, sizeof(len_net), 0);
      if (sent != sizeof(len_net)) {
        ROS_ERROR("发送数据长度失败");
      }
      sent = send(sockfd, data_str.c_str(), data_str.size(), 0);
      if (sent != (ssize_t)data_str.size()) {
        ROS_ERROR("发送数据内容失败");
      } else {
        ROS_INFO("Data sent successfully");
      }
      lock.lock();
    }
  }
}

bool MeshAccumulator::initSocket(const std::string &ip, int port) {
  sockfd = socket(AF_INET, SOCK_STREAM, 0);
  if (sockfd < 0) {
    ROS_ERROR("创建 socket 失败");
    return false;
  }
  sockaddr_in server_addr;
  server_addr.sin_family = AF_INET;
  server_addr.sin_port = htons(port);
  if (inet_pton(AF_INET, ip.c_str(), &server_addr.sin_addr) <= 0) {
    ROS_ERROR("无效的 IP 地址");
    close(sockfd);
    sockfd = -1;
    return false;
  }
  if (connect(sockfd, (struct sockaddr *)&server_addr, sizeof(server_addr)) <
      0) {
    ROS_ERROR("连接服务器失败: %s:%d", ip.c_str(), port);
    close(sockfd);
    sockfd = -1;
    return false;
  }
  ROS_INFO("成功连接到服务器 %s:%d", ip.c_str(), port);
  return true;
}

int main(int argc, char **argv) {
  setlocale(LC_ALL, "");
  ros::init(argc, argv, "accumulate_mesh_to_off");
  ros::NodeHandle nh("~");

  // 通过 ROS 参数加载服务器 IP、端口等参数
  std::string ip;
  int port;
  nh.param<std::string>("server_ip", ip, "127.0.0.1");
  nh.param<int>("server_port", port, 12345);

  MeshAccumulator accumulator(nh);

  // 订阅 /voxblox_node/mesh 消息
  ros::Subscriber sub = nh.subscribe(
      "/voxblox_node/mesh", 10, &MeshAccumulator::meshCallback, &accumulator);

  // 初始化 socket 连接
  if (!accumulator.initSocket(ip, port)) {
    ROS_ERROR("Socket init failed");
    return -1;
  }

  ros::spin();
  return 0;
}
