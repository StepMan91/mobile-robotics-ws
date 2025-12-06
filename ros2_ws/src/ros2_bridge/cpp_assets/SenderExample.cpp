#include "SimpleUDP.h"
#include <iostream>
#include <vector>
#include <string>
#include <sstream>
#include <thread>
#include <chrono>

// Mock Data Structures (Replace with your own)
struct Joint {
    std::string name;
    float x, y, z;
    float confidence;
};

struct Skeleton {
    int id;
    std::vector<Joint> joints;
};

// Helper to create JSON string manually (to avoid adding JSON library dependency for this example)
// In your real app, use nlohmann/json or similar if available.
std::string create_json_payload(const Skeleton& skel) {
    std::stringstream ss;
    ss << "{";
    ss << "\"skeletons\": [";
    ss << "{";
    ss << "\"id\": " << skel.id << ",";
    ss << "\"joints\": {";
    
    for (size_t i = 0; i < skel.joints.size(); ++i) {
        const auto& joint = skel.joints[i];
        ss << "\"" << joint.name << "\": {";
        ss << "\"x\": " << joint.x << ",";
        ss << "\"y\": " << joint.y << ",";
        ss << "\"z\": " << joint.z << ",";
        ss << "\"confidence\": " << joint.confidence;
        ss << "}";
        if (i < skel.joints.size() - 1) ss << ",";
    }
    
    ss << "}"; // End joints
    ss << "}"; // End skeleton
    ss << "]"; // End skeletons array
    ss << "}";
    return ss.str();
}

int main() {
    // 1. Initialize UDP Sender
    SimpleUDP udp;
    if (!udp.init("127.0.0.1", 8888)) { // Target IP (Localhost) and Port
        std::cerr << "Failed to init UDP" << std::endl;
        return -1;
    }
    std::cout << "UDP Sender started on port 8888" << std::endl;

    // 2. Mock Loop (Replace with your Camera Frame Loop)
    int frame_count = 0;
    while (true) {
        // Create dummy data
        Skeleton skel;
        skel.id = 1;
        
        // Simulating a moving hand (Circle motion)
        float t = frame_count * 0.1f;
        skel.joints.push_back({ "RightWrist", 0.5f + cos(t) * 0.2f, 0.0f, 0.5f + sin(t) * 0.2f, 1.0f });
        skel.joints.push_back({ "LeftWrist",  -0.5f, 0.0f, 0.5f, 1.0f });

        // 3. Prepare JSON
        std::string json_data = create_json_payload(skel);

        // 4. Send
        udp.send(json_data);
        std::cout << "Sent frame " << frame_count << std::endl;

        frame_count++;
        std::this_thread::sleep_for(std::chrono::milliseconds(33)); // ~30 FPS
    }

    return 0;
}
