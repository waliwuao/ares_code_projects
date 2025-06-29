#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <sensor_msgs/msg/joy.hpp>        // 摇杆消息
#include <nlohmann/json.hpp>
#include <iostream>
#include <vector>
#include <chrono> // for sleep
#include <thread> // for sleep
#include <limits> // Required for numeric_limits
#include <ios>  // Required for streamsize
#include <sys/socket.h>
#include <netinet/in.h>
#include <unistd.h> 
#include <string>
#include <vector>
#include <sstream>
#include <algorithm>
#include <fcntl.h>

const std::string vel_node_name = "cmd_vel";

// 映射关系: source_channel_index -> destination_joy_index
// -1 表示该通道不被映射
// Axis: Maps RC channel index to joy.axes index
const int axis_mappings[18] = 	{0, 1, 4, 2, 3, 5, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1};
// Button: Maps RC channel index to joy.buttons index
const int button_mappings[18] = {-1, -1, -1, -1, -1, -1, -1, -1, 0, 1, 2, 3, 4, 5, -1, -1, -1, -1};

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
  auto node = rclcpp::Node::make_shared("udp_joy_node");
  auto joy_pub = node->create_publisher<sensor_msgs::msg::Joy>("joy", 10);
  auto twist_pub = node->create_publisher<geometry_msgs::msg::Twist>(vel_node_name, 10);

  int server_fd;
  struct sockaddr_in address;
  
  if ((server_fd = socket(AF_INET, SOCK_DGRAM, 0)) < 0) {
    RCLCPP_ERROR(node->get_logger(), "Socket creation failed");
    return -1;
  }

  int opt = 1;
  if (setsockopt(server_fd, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt))) {
    RCLCPP_ERROR(node->get_logger(), "setsockopt failed");
    close(server_fd);
    return -1;
  }

  address.sin_family = AF_INET;
  address.sin_addr.s_addr = INADDR_ANY;
  address.sin_port = htons(24660);

  if (bind(server_fd, (struct sockaddr *)&address, sizeof(address)) < 0) {
    RCLCPP_ERROR(node->get_logger(), "Bind failed");
    close(server_fd);
    return -1;
  }

  RCLCPP_INFO(node->get_logger(), "Listening for UDP packets on port 24660...");
  
  char buffer[2048] = {0};

	// Determine size of axes and buttons from mappings robustly, once at startup.
	int max_axis = -1;
	for(int val : axis_mappings) if(val > max_axis) max_axis = val;
	const size_t axes_size = (max_axis > -1) ? (max_axis + 1) : 0;

	int max_button = -1;
	for(int val : button_mappings) if(val > max_button) max_button = val;
	const size_t buttons_size = (max_button > -1) ? (max_button + 1) : 0;

  while (rclcpp::ok()) {
    struct sockaddr_in client_addr;
    socklen_t client_addr_len = sizeof(client_addr);
    int n_bytes = recvfrom(server_fd, buffer, sizeof(buffer) - 1, 0, (struct sockaddr*)&client_addr, &client_addr_len);

    if (n_bytes > 0) {
      buffer[n_bytes] = '\0';
      try {
        auto json_data = nlohmann::json::parse(buffer);
        if (json_data.contains("rc_channels")) {
          const auto& channels = json_data["rc_channels"];
          if (channels.is_array() && channels.size() >= 18) {
            sensor_msgs::msg::Joy joy_msg;
            joy_msg.header.stamp = node->get_clock()->now();
            joy_msg.axes.resize(axes_size, 0.0f);
            joy_msg.buttons.resize(buttons_size, 0);

            // Safely populate axes and buttons based on mappings
            for (int i = 0; i < 18; ++i) { // i is the source channel index
              int axis_dest_idx = axis_mappings[i];
              if (axis_dest_idx != -1) {
                if (channels[i].is_number_integer()) {
                  joy_msg.axes[axis_dest_idx] = normalize_rc_channel(channels[i].get<int>());
                }
              }
              
              int button_dest_idx = button_mappings[i];
              if (button_dest_idx != -1) {
                if (channels[i].is_number_integer()) {
                  // Treat channel value > 1500 as "pressed" (1)
                  joy_msg.buttons[button_dest_idx] = (channels[i].get<int>() > 1500) ? 1 : 0;
                }
              }
            }

            joy_pub->publish(joy_msg);
            RCLCPP_DEBUG(node->get_logger(), "Published Joy message from received data.");

            // Create and publish the Twist message based on the provided logic
            auto twist_msg = std::make_unique<geometry_msgs::msg::Twist>();
            if (joy_msg.axes.size() >= 3) {
              // Mapping joy axes to twist components, applying negation as per your example
              twist_msg->linear.x = joy_msg.axes[0];  // Corresponds to arg1
              twist_msg->linear.y = -joy_msg.axes[1]; // Corresponds to arg2
              twist_msg->angular.x = -joy_msg.axes[2]; // Corresponds to arg3
            }
            twist_pub->publish(std::move(twist_msg));
            RCLCPP_DEBUG(node->get_logger(), "Published Twist message from joy data.");

          // INSERT_YOUR_CODE
          // 定义静态变量用于统计和定时
          static int send_count = 0;
          static auto last_print_time = std::chrono::steady_clock::now();

          send_count++;

          auto now = std::chrono::steady_clock::now();
          auto duration = std::chrono::duration_cast<std::chrono::seconds>(now - last_print_time).count();
          if (duration >= 1) {
            std::cout << "Twist消息发送频率: " << send_count << " 次/秒" << std::endl;
            send_count = 0;
            last_print_time = now;
          }
          }
        }
      } catch (const nlohmann::json::parse_error& e) {
        RCLCPP_WARN(node->get_logger(), "JSON parse error: %s. Received data: %s", e.what(), buffer);
      }
    } else if (n_bytes < 0) {
       RCLCPP_WARN(node->get_logger(), "recvfrom failed, errno: %d", errno);
    }
    rclcpp::spin_some(node);
  }
  
  close(server_fd);
  rclcpp::shutdown();
  return 0;
} 