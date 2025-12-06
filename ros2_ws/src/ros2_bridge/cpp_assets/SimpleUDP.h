#ifndef SIMPLE_UDP_H
#define SIMPLE_UDP_H

#include <iostream>
#include <string>
#include <winsock2.h>
#include <ws2tcpip.h>


#pragma comment(lib, "ws2_32.lib") // Link automatically in Visual Studio

class SimpleUDP {
public:
  SimpleUDP() : initialized(false) {
    // Initialize Winsock
    WSADATA wsaData;
    int result = WSAStartup(MAKEWORD(2, 2), &wsaData);
    if (result != 0) {
      std::cerr << "WSAStartup failed: " << result << std::endl;
    }
  }

  ~SimpleUDP() {
    if (initialized) {
      closesocket(sendSocket);
    }
    WSACleanup();
  }

  bool init(const std::string &ip_address, int port) {
    sendSocket = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
    if (sendSocket == INVALID_SOCKET) {
      std::cerr << "Socket creation failed: " << WSAGetLastError() << std::endl;
      return false;
    }

    destAddr.sin_family = AF_INET;
    destAddr.sin_port = htons(port);
    inet_pton(AF_INET, ip_address.c_str(), &destAddr.sin_addr);

    initialized = true;
    return true;
  }

  void send(const std::string &message) {
    if (!initialized)
      return;

    int result = sendto(sendSocket, message.c_str(), (int)message.length(), 0,
                        (SOCKADDR *)&destAddr, sizeof(destAddr));
    if (result == SOCKET_ERROR) {
      // std::cerr << "sendto failed: " << WSAGetLastError() << std::endl;
    }
  }

private:
  SOCKET sendSocket;
  sockaddr_in destAddr;
  bool initialized;
};

#endif // SIMPLE_UDP_H
