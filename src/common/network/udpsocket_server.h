//
// Created by xucong on 23-9-15.
//

#pragma once

#include <iostream>
#include <mutex>
#include <string>
#include <thread>
#include <csignal>
#include <cstring>
#include <unistd.h>
#include <sys/socket.h>
#include <sys/types.h>
#include <arpa/inet.h>
#include <netinet/in.h>

struct UdpConnection_server {
  int sock_fd;
  struct sockaddr_in clnt_addr;
  socklen_t sock_len = static_cast<socklen_t>(sizeof(clnt_addr));
  uint32_t FRAME_SIZE = 1024;
};

namespace stoic {

template <typename D>
class UdpSocketServer {
 public:
  typedef D DataType;

  /**
   * @brief UDP socket initialized
   * @param port remote port
   */
  bool connectToUdpClient(const int& port) {
    udp_server_.sock_fd = socket(AF_INET, SOCK_DGRAM, 0);
    if (udp_server_.sock_fd < 0) {
      std::cerr << "create socket failed!\n" << std::endl;
      return -1;
    }

    //中断函数
    signal(SIGINT, signalHandler);

    memset(&udp_server_.clnt_addr, 0, sizeof(udp_server_.clnt_addr));
    udp_server_.clnt_addr.sin_family = AF_INET;
    udp_server_.clnt_addr.sin_port = htons(port);
    udp_server_.clnt_addr.sin_addr.s_addr = htonl(INADDR_ANY);

    int opt = 1;
    setsockopt ( udp_server_.sock_fd, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt));
    int ret = bind(udp_server_.sock_fd,(struct sockaddr *)&udp_server_.clnt_addr, udp_server_.sock_len);
    if (ret == -1) {
      std::cerr << "bind socket failed" << std::endl;
      close(udp_server_.sock_fd);
      return false;
    }

    std::cout << "connected to client success. port [" << port << "]" << std::endl;

    return true;

  }


  /**
   * @brief receive udp data
   * @param data The data of udp server received
   */
  const D& recvData(const DataType& data) {
    int nbytes = 0;
    int total_recv = 2 * udp_server_.FRAME_SIZE;
    char recvBuf[2048] = {0};
    memset(recvBuf, 0, sizeof(char) * total_recv);

    nbytes = recvfrom(udp_server_.sock_fd, recvBuf, sizeof(recvBuf), 0,
                      (sockaddr *)&udp_server_.clnt_addr, &udp_server_.sock_len);
    if (nbytes == -1 || nbytes > total_recv) {
      std::cout << "failed to receive data." << std::endl;
    }
//    std::cout << "recv " << nbytes << " bytes from client." << std::endl;

    memcpy((char*)(&data), recvBuf, sizeof(data));

    return data;
  }

  void closeSocket() {close(udp_server_.sock_fd);}

 private:
  static void signalHandler(int signum) {
    if (signum == SIGINT) {
      std::cout << "Ctrl+C pressed. Closing socket and exiting." << std::endl;
      static UdpSocketServer<D>* instance;
      close(instance->udp_server_.sock_fd);
    }
  }

 private:
  UdpConnection_server udp_server_;
};


} // namespace stoic
