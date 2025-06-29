#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>

#include <geometry_msgs/msg/vector3.hpp>         // 角速度和加速度
#include <geometry_msgs/msg/quaternion.hpp>      // 四元数
#include <sensor_msgs/msg/joy.hpp>               // 摇杆消息

#include "ares_protocol.hpp"
#include <iostream>
#include <vector>
#include <chrono> // for sleep
#include <thread> // for sleep
#include <limits> // Required for numeric_limits
#include <ios>    // Required for streamsize

#include <sys/socket.h>
#include <netinet/in.h>
#include <unistd.h>
#include <string>
#include <sstream>
#include <algorithm>
#include <fcntl.h>

// 函数：将遥控器通道值从 [1000, 2000] 映射到 [-1.0, 1.0]
float normalize_rc_channel(int raw_value) {
    if (raw_value < 1000 || raw_value > 2000) {
        // 对于超出典型范围的值，可以返回0.0或者根据需要进行其他处理
        return 0.0f;
    }
    float normalized = (static_cast<float>(raw_value) - 1500.0f) / 500.0f;
    return std::clamp(normalized, -1.0f, 1.0f);
}

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("remote_rc_publisher");
    auto joy_pub = node->create_publisher<sensor_msgs::msg::Joy>("joy", 10);

    int server_fd, new_socket;
    struct sockaddr_in address;
    int opt = 1;
    int addrlen = sizeof(address);

    if ((server_fd = socket(AF_INET, SOCK_STREAM, 0)) == 0) {
        RCLCPP_ERROR(node->get_logger(), "Socket creation failed");
        return -1;
    }

    if (setsockopt(server_fd, SOL_SOCKET, SO_REUSEADDR | SO_REUSEPORT, &opt, sizeof(opt))) {
        RCLCPP_ERROR(node->get_logger(), "setsockopt failed");
        return -1;
    }
    address.sin_family = AF_INET;
    address.sin_addr.s_addr = INADDR_ANY;
    address.sin_port = htons(24660);

    if (bind(server_fd, (struct sockaddr *)&address, sizeof(address)) < 0) {
        RCLCPP_ERROR(node->get_logger(), "Bind failed");
        return -1;
    }

    if (listen(server_fd, 3) < 0) {
        RCLCPP_ERROR(node->get_logger(), "Listen failed");
        return -1;
    }

    RCLCPP_INFO(node->get_logger(), "Waiting for a connection on port 24660...");

    while (rclcpp::ok()) {
        new_socket = accept(server_fd, (struct sockaddr *)&address, (socklen_t*)&addrlen);
        if (new_socket < 0) {
            if (errno == EINTR && !rclcpp::ok()) {
                break;
            }
            RCLCPP_WARN(node->get_logger(), "Accept failed, errno: %d", errno);
            continue;
        }

        RCLCPP_INFO(node->get_logger(), "Client connected.");
        std::string data_buffer;
        char buffer[1024] = {0};

        while (rclcpp::ok()) {
            int valread = recv(new_socket, buffer, 1024, 0);
            if (valread <= 0) {
                RCLCPP_INFO(node->get_logger(), "Client disconnected.");
                close(new_socket);
                break;
            }

            data_buffer.append(buffer, valread);

            size_t start_pos, end_pos;
            while ((start_pos = data_buffer.find('{')) != std::string::npos &&
                   (end_pos = data_buffer.find('}')) != std::string::npos &&
                   start_pos < end_pos)
            {
                std::string json_str = data_buffer.substr(start_pos, end_pos - start_pos + 1);
                
                std::string key = "\"rc_channels\": [";
                size_t array_start = json_str.find(key);
                if (array_start != std::string::npos) {
                    array_start += key.length();
                    size_t array_end = json_str.find(']', array_start);
                    if (array_end != std::string::npos) {
                        std::string array_content = json_str.substr(array_start, array_end - array_start);
                        std::stringstream ss(array_content);
                        std::string segment;
                        std::vector<int> channels;
                        while(std::getline(ss, segment, ',')) {
                            try {
                                channels.push_back(std::stoi(segment));
                            } catch(const std::invalid_argument& e) {
                                RCLCPP_WARN(node->get_logger(), "Invalid number in JSON array: %s", segment.c_str());
                            }
                        }

                        if (channels.size() >= 8) {
                            sensor_msgs::msg::Joy joy_msg;
                            joy_msg.header.stamp = node->get_clock()->now();
                            joy_msg.axes.resize(8);
                            for(int i = 0; i < 8; ++i) {
                                joy_msg.axes[i] = normalize_rc_channel(channels[i]);
                            }
                            joy_pub->publish(joy_msg);
                        }
                    }
                }
                data_buffer.erase(0, end_pos + 1);
            }
        }
        rclcpp::spin_some(node);
    }
    
    close(server_fd);
    rclcpp::shutdown();
    return 0;
}
