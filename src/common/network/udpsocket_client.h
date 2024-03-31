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

struct UdpConnection_client {
  int sock_fd;
  struct sockaddr_in serv_addr;
  socklen_t sock_len = static_cast<socklen_t>(sizeof(serv_addr));
};

namespace stoic {

template <typename D>
class UdpSocketClient {
 public:
  typedef D DataType;

  /**
   * @brief UDP socket initialized
   * @param ip_addr remote ip address
   * @param port remote port
   */
  bool connectToUdpServer(const std::string& ip_addr, const int& port) {
    std::cout << "connecting to server... " << std::endl;
    udp_client_.sock_fd = socket(AF_INET, SOCK_DGRAM, 0);
    if (udp_client_.sock_fd < 0) {
      std::cout << "create socket failed!" << std::endl;
      return false;
    }
    memset(&udp_client_.serv_addr, 0, sizeof(udp_client_.serv_addr));
    udp_client_.serv_addr.sin_family = AF_INET;
    udp_client_.serv_addr.sin_port = htons(static_cast<uint16_t>(port));
    udp_client_.serv_addr.sin_addr.s_addr = inet_addr(ip_addr.c_str());

    if (ip_addr.empty() || port == 0) {
      std::cout << "remote info is invalid." << std::endl;
      return false;
    }

    int res =
        connect(udp_client_.sock_fd, (struct sockaddr*)&udp_client_.serv_addr,udp_client_.sock_len);

    if (res < 0) {
      std::cout << "connect socket failed!" << std::endl;
      close(udp_client_.sock_fd);
    }

    std::cout << "connected to server success. port [" << port << "]" << std::endl;

    return true;
  }

  /**
   * @brief send udp data
   * @param data data of udp client send to server
   */
  const D& sendData(const DataType& data) {
    ssize_t nbytes = sendto(udp_client_.sock_fd, &data, sizeof(data), 0,
                            (struct sockaddr*)&udp_client_.serv_addr, udp_client_.sock_len);
    if (nbytes == -1) {
      std::cerr << "sent msg failed." << std::endl;
      std::cerr << "Failed to send data. Error code: " << errno << std::endl;
      close(udp_client_.sock_fd);
    }
//    std::cout << "sent " << nbytes << " bytes to server." << std::endl;

    return data;
  }

  void closeSocket() {close(udp_client_.sock_fd);}

 private:
  static void signalHandler(int signum) {
    if (signum == SIGINT) {
      std::cout << "Ctrl+C pressed. Closing socket and exiting." << std::endl;
      static UdpSocketClient<D>* instance;
      close(instance->udp_client_.sock_fd);
    }
  }

 private:
  UdpConnection_client udp_client_;
};


} // namespace stoic
