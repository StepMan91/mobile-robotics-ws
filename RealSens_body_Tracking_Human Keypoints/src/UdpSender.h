#pragma once

#ifndef NOMINMAX
#define NOMINMAX
#endif

#include "Utils.h"
#include <string>
#include <vector>
#include <winsock2.h>
#include <ws2tcpip.h>

#pragma comment(lib, "ws2_32.lib")

namespace RealsenseBodyPose {

class UdpSender {
public:
  UdpSender(const std::string &ip = "127.0.0.1", int port = 8888);
  ~UdpSender();

  // Initialize Winsock
  bool initialize();

  // Send skeleton data as JSON string
  void send(const std::vector<Skeleton> &skeletons);

private:
  std::string m_ip;
  int m_port;
  SOCKET m_socket;
  sockaddr_in m_destAddr;
  bool m_initialized;

  // Helper to format float string
  std::string formatFloat(float val);
};

} // namespace RealsenseBodyPose
