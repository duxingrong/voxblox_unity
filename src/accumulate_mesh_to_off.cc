#include "voxblox_unity/accumulate_mesh_to_off.h"

// 构造函数：初始化 ROS 节点句柄和发送线程
MeshAccumulator::MeshAccumulator(ros::NodeHandle &nh)
    : nh_(nh), sockfd(-1), shutdownFlag(false) {
  // 启动发送数据线程
  sendThread = std::thread(&MeshAccumulator::sendDataThread, this);
}

// 析构函数：停止线程并关闭 socket
MeshAccumulator::~MeshAccumulator() {
  shutdownFlag.store(true);
  sendCond.notify_all();
  if (sendThread.joinable())
    sendThread.join();
  if (sockfd != -1)
    close(sockfd);
}

/**
 * @brief 计算顶点法线
 *
 * 根据给定的顶点和面信息，计算每个顶点的法线。
 *
 * @param vertices 顶点列表
 * @param faces 每个面由三个顶点索引构成
 * @param vertexNormals 输出计算得到的顶点法线列表
 */
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

/**
 * @brief voxblox 消息回调函数
 *
 * 处理来自话题 /voxblox_node/mesh 的消息，将增量网格数据转换为 OFF 格式，
 * 然后加入发送队列，通过 socket 发送出去。
 *
 * @param msg 指向接收到的 voxblox 网格消息
 */
void MeshAccumulator::meshCallback(const voxblox_msgs::Mesh::ConstPtr &msg) {
  // 遍历每个 mesh block
  for (const auto &block : msg->mesh_blocks) {
    std::array<int, 3> block_key = {static_cast<int>(block.index[0]),
                                    static_cast<int>(block.index[1]),
                                    static_cast<int>(block.index[2])};

    size_t num_vertices = block.x.size();
    std::stringstream ss;
    if (num_vertices == 0) {
      // 空块表示该块被删除，发送一个特殊的标记
      ss << "DELETE_BLOCK\n"
         << block.index[0] << " " << block.index[1] << " " << block.index[2]
         << "\n";
    } else {
      // 处理非空块
      Eigen::Vector3d block_offset(block.index[0] * msg->block_edge_length,
                                   block.index[1] * msg->block_edge_length,
                                   block.index[2] * msg->block_edge_length);

      std::vector<Eigen::Vector3d> blockVertices;
      std::vector<std::array<int, 3>> blockFaces;
      std::vector<Eigen::Vector3d> blockColors;

      // 处理顶点和颜色（voxblox 消息中顶点为整数，需要转换）
      double point_conv_factor = 2.0 / 65535.0;
      for (size_t i = 0; i < num_vertices; ++i) {
        Eigen::Vector3d q(block.x[i], block.y[i], block.z[i]);
        Eigen::Vector3d vertex =
            block_offset + (q * point_conv_factor) * msg->block_edge_length;
        blockVertices.push_back(vertex);

        if (block.r.size() > i)
          blockColors.push_back(
              Eigen::Vector3d(block.r[i], block.g[i], block.b[i]));
        else
          blockColors.push_back(Eigen::Vector3d(0.0, 0.0, 0.0));
      }

      // 处理面索引：假设每 3 个连续顶点构成一个面
      size_t num_triangles = num_vertices / 3;
      for (size_t i = 0; i < num_triangles; ++i) {
        int base = i * 3;
        blockFaces.push_back({base, base + 1, base + 2});
      }

      // 计算顶点法线
      std::vector<Eigen::Vector3d> blockNormals;
      computeVertexNormals(blockVertices, blockFaces, blockNormals);

      // 生成带块索引的 OFF 格式数据
      ss << "BLOCK_UPDATE\n"
         << block.index[0] << " " << block.index[1] << " " << block.index[2]
         << "\n";
      ss << "NCOFF\n"
         << blockVertices.size() << " " << blockFaces.size() << " 0\n";

      constexpr double norm_factor = 1.0 / 255.0;
      for (size_t i = 0; i < blockVertices.size(); ++i) {
        const auto &v = blockVertices[i];
        const auto &n = blockNormals[i];
        const auto &c = blockColors[i];
        ss << std::fixed << std::setprecision(6) << v.x() << " " << v.y() << " "
           << v.z() << " " << n.x() << " " << n.y() << " " << n.z() << " "
           << c.x() * norm_factor << " " << c.y() * norm_factor << " "
           << c.z() * norm_factor << "\n";
      }
      for (const auto &face : blockFaces) {
        ss << "3 " << face[0] << " " << face[1] << " " << face[2] << "\n";
      }
    }
    // 将当前块更新数据加入发送队列
    {
      std::lock_guard<std::mutex> lock(queueMutex);
      sendQueue.push(ss.str());
    }
    sendCond.notify_one();
  }
}

/**
 * @brief nvblox 消息回调函数
 *
 * 处理来自话题 /nvblox_node/mesh 的消息，将增量地图数据（nvblox 格式）
 * 转换为 OFF 格式数据，然后加入发送队列，通过 socket 发送出去。
 *
 * @param msg 指向接收到的 nvblox 网格消息
 */
// void MeshAccumulator::nvbloxMeshCallback(
//     const nvblox_msgs::Mesh::ConstPtr &msg) {
//   // 遍历每个网格块，注意：nvblox 消息中 block_indices 与 blocks 数组长度相同
//   for (size_t i = 0; i < msg->block_indices.size(); ++i) {
//     const nvblox_msgs::Index3D &block_idx = msg->block_indices[i];
//     const nvblox_msgs::MeshBlock &block = msg->blocks[i];
//
//     std::stringstream ss;
//     // 若该块为空，表示此块已被删除，发送删除标记
//     if (block.vertices.empty()) {
//       ss << "DELETE_BLOCK\n"
//          << block_idx.x << " " << block_idx.y << " " << block_idx.z << "\n";
//     } else {
//       // 计算块的全局偏移：根据 block_indices 与消息中的 block_size
//       Eigen::Vector3d block_offset(block_idx.x * msg->block_size,
//                                    block_idx.y * msg->block_size,
//                                    block_idx.z * msg->block_size);
//
//       std::vector<Eigen::Vector3d> blockVertices;
//       std::vector<std::array<int, 3>> blockFaces;
//       std::vector<Eigen::Vector3d> blockColors;
//
//       // 处理顶点和颜色数据（nvblox 消息中顶点为 geometry_msgs/Point32
//       // 类型，直接读取即可）
//       size_t num_vertices = block.vertices.size();
//       for (size_t j = 0; j < num_vertices; ++j) {
//         Eigen::Vector3d vertex(block.vertices[j].x, block.vertices[j].y,
//                                block.vertices[j].z);
//         // 顶点坐标加上块的全局偏移
//         vertex += block_offset;
//         blockVertices.push_back(vertex);
//
//         // 处理颜色数据
//         if (block.colors.size() > j) {
//           blockColors.push_back(Eigen::Vector3d(
//               block.colors[j].r, block.colors[j].g, block.colors[j].b));
//         } else {
//           blockColors.push_back(Eigen::Vector3d(0.0, 0.0, 0.0));
//         }
//       }
//
//       // 使用 nvblox 消息中提供的三角形索引构建面列表
//       size_t num_triangles = block.triangles.size() / 3;
//       for (size_t j = 0; j < num_triangles; ++j) {
//         int base = j * 3;
//         blockFaces.push_back({block.triangles[base], block.triangles[base +
//         1],
//                               block.triangles[base + 2]});
//       }
//
//       // 计算顶点法线（即便消息中有 normals，也重新计算以保证一致性）
//       std::vector<Eigen::Vector3d> blockNormals;
//       computeVertexNormals(blockVertices, blockFaces, blockNormals);
//
//       // 生成带块索引的 OFF 格式数据
//       ss << "BLOCK_UPDATE\n"
//          << block_idx.x << " " << block_idx.y << " " << block_idx.z << "\n";
//       ss << "NCOFF\n"
//          << blockVertices.size() << " " << blockFaces.size() << " 0\n";
//
//       for (size_t j = 0; j < blockVertices.size(); ++j) {
//         const auto &v = blockVertices[j];
//         const auto &n = blockNormals[j];
//         const auto &c = blockColors[j];
//         ss << std::fixed << std::setprecision(6) << v.x() << " " << v.y() <<
//         " "
//            << v.z() << " " << n.x() << " " << n.y() << " " << n.z() << " "
//            << c.x() << " " << c.y() << " " << c.z() << "\n";
//       }
//       for (const auto &face : blockFaces) {
//         ss << "3 " << face[0] << " " << face[1] << " " << face[2] << "\n";
//       }
//     }
//
//     // 将当前块更新数据加入发送队列
//     {
//       std::lock_guard<std::mutex> lock(queueMutex);
//       sendQueue.push(ss.str());
//     }
//     sendCond.notify_one();
//   }
// }

/**
 * @brief 发送数据线程
 *
 * 该线程不断等待发送队列中的数据，当有数据时，通过 socket 将数据发送出去。
 */
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

/**
 * @brief 初始化 socket 连接
 *
 * 根据给定的 IP 地址和端口号，初始化 TCP socket 并建立连接。
 *
 * @param ip 服务器 IP 地址
 * @param port 服务器端口号
 * @return true 连接成功
 * @return false 连接失败
 */
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

  // 订阅 voxblox 网格消息
  ros::Subscriber sub_voxblox = nh.subscribe(
      "/voxblox_node/mesh", 10, &MeshAccumulator::meshCallback, &accumulator);

  // 订阅 nvblox 网格消息
  // ros::Subscriber sub_nvblox =
  //     nh.subscribe("/nvblox_node/mesh", 10,
  //                  &MeshAccumulator::nvbloxMeshCallback, &accumulator);

  // 初始化 socket 连接
  if (!accumulator.initSocket(ip, port)) {
    ROS_ERROR("Socket init failed");
    return -1;
  }

  ros::spin();
  return 0;
}
