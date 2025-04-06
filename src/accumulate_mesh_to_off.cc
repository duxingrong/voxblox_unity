#include "voxblox_unity/accumulate_mesh_to_off.h"

#include <arpa/inet.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <unistd.h>

MeshAccumulator::MeshAccumulator(ros::NodeHandle &nh)
    : nh_(nh), sockfd(-1), shutdownFlag(false) {

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

void MeshAccumulator::meshCallback(const voxblox_msgs::Mesh::ConstPtr &msg) {
  // 遍历每个 mesh block
  for (const auto &block : msg->mesh_blocks) {
    std::array<int, 3> block_key = {static_cast<int>(block.index[0]),
                                    static_cast<int>(block.index[1]),
                                    static_cast<int>(block.index[2])};

    size_t num_vertices = block.x.size();

    // 创建单个块的OFF数据
    std::stringstream ss;

    if (num_vertices == 0) {
      // 空块表示该块被删除，发送一个特殊的标记
      ss << "DELETE_BLOCK\n"
         << block.index[0] << " " << block.index[1] << " " << block.index[2]
         << "\n";
    } else {
      // 处理非空块
      MeshBlockData blockData;
      Eigen::Vector3d block_offset(block.index[0] * msg->block_edge_length,
                                   block.index[1] * msg->block_edge_length,
                                   block.index[2] * msg->block_edge_length);

      // 构建顶点数据
      std::vector<Eigen::Vector3d> blockVertices;
      std::vector<std::array<int, 3>> blockFaces;
      std::vector<Eigen::Vector3d> blockColors;

      // 处理顶点和颜色
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

      // 处理面索引
      size_t num_triangles = num_vertices / 3;
      for (size_t i = 0; i < num_triangles; ++i) {
        int base = i * 3;
        blockFaces.push_back({base, base + 1, base + 2});
      }

      // 计算顶点法线
      std::vector<Eigen::Vector3d> blockNormals;
      computeVertexNormals(blockVertices, blockFaces, blockNormals);

      // 生成带块索引的OFF格式
      ss << "BLOCK_UPDATE\n"
         << block.index[0] << " " << block.index[1] << " " << block.index[2]
         << "\n";
      ss << "NCOFF\n"
         << blockVertices.size() << " " << blockFaces.size() << " 0\n";

      // 写入顶点、法线、颜色数据
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

      // 写入面数据
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
