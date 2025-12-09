#include "UdpSender.h"
#include <iomanip>
#include <iostream>
#include <sstream>

namespace RealsenseBodyPose {

UdpSender::UdpSender(const std::string &ip, int port)
    : m_ip(ip), m_port(port), m_socket(INVALID_SOCKET), m_initialized(false) {}

UdpSender::~UdpSender() {
  if (m_socket != INVALID_SOCKET) {
    closesocket(m_socket);
  }
  WSACleanup();
}

bool UdpSender::initialize() {
  WSADATA wsaData;
  int result = WSAStartup(MAKEWORD(2, 2), &wsaData);
  if (result != 0) {
    std::cerr << "WSAStartup failed with error: " << result << std::endl;
    return false;
  }

  m_socket = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
  if (m_socket == INVALID_SOCKET) {
    std::cerr << "socket failed with error: " << WSAGetLastError() << std::endl;
    WSACleanup();
    return false;
  }

  m_destAddr.sin_family = AF_INET;
  m_destAddr.sin_port = htons(m_port);
  inet_pton(AF_INET, m_ip.c_str(), &m_destAddr.sin_addr);

  m_initialized = true;
  std::cout << "[UDP] Sender initialized on " << m_ip << ":" << m_port
            << std::endl;
  return true;
}

std::string UdpSender::formatFloat(float val) {
  std::stringstream ss;
  ss << std::fixed << std::setprecision(3) << val;
  return ss.str();
}

void UdpSender::send(const std::vector<Skeleton> &skeletons) {
  if (!m_initialized || skeletons.empty())
    return;

  // Manual JSON construction
  std::stringstream json;
  json << "{\"skeletons\":[";

  for (size_t i = 0; i < skeletons.size(); ++i) {
    const auto &skel = skeletons[i];

    if (i > 0)
      json << ",";

    // Use index + 1 as ID
    json << "{\"id\":" << (i + 1) << ",\"joints\":{";

    bool needsComma = false;

    // 1. Calculate and add Pelvis (Midpoint of Hips: 11 and 12)
    if (skel.keypoints3D.size() > 12) {
      const auto &lHip = skel.keypoints3D[11];
      const auto &rHip = skel.keypoints3D[12];
      if (lHip.isValid() && rHip.isValid()) {
        float px = (lHip.x + rHip.x) / 2.0f;
        float py = (lHip.y + rHip.y) / 2.0f;
        float pz = (lHip.z + rHip.z) / 2.0f;
        float pConf = (lHip.confidence + rHip.confidence) / 2.0f;

        json << "\"Pelvis\":{\"x\":" << formatFloat(px)
             << ",\"y\":" << formatFloat(py) << ",\"z\":" << formatFloat(pz)
             << ",\"confidence\":" << formatFloat(pConf) << "}";
        needsComma = true;
      }
    }

    // 2. Add other joints with new naming convention
    struct JointName {
      int id;
      std::string name;
    };
    // Mapping COCO indices to requested names
    std::vector<JointName> keyJoints = {
        {0, "Nose"},        {5, "LeftShoulder"}, {6, "RightShoulder"},
        {7, "LeftElbow"},   {8, "RightElbow"},   {9, "LeftWrist"},
        {10, "RightWrist"}, {11, "LeftHip"},     {12, "RightHip"},
        {13, "LeftKnee"},   {14, "RightKnee"},   {15, "LeftAnkle"},
        {16, "RightAnkle"}};

    for (const auto &kj : keyJoints) {
      int jId = kj.id;
      // Check bounds
      if (jId < (int)skel.keypoints3D.size()) {
        const auto &k3d = skel.keypoints3D[jId];
        if (k3d.isValid()) {
          if (needsComma)
            json << ",";
          json << "\"" << kj.name << "\":{\"x\":" << formatFloat(k3d.x)
               << ",\"y\":" << formatFloat(k3d.y)
               << ",\"z\":" << formatFloat(k3d.z)
               << ",\"confidence\":" << formatFloat(k3d.confidence) << "}";
          needsComma = true;
        }
      }
    }
    json << "}}";
  }
  json << "]}";

  std::string payload = json.str();
  sendto(m_socket, payload.c_str(), (int)payload.length(), 0,
         (sockaddr *)&m_destAddr, sizeof(m_destAddr));
}

} // namespace RealsenseBodyPose
