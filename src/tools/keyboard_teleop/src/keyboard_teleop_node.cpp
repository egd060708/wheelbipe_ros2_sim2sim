// Copyright (c) 2023 Direct Drive Technology Co., Ltd. All rights reserved.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <std_msgs/msg/float64.hpp>
#include <termios.h>
#include <unistd.h>
#include <fcntl.h>
#include <iostream>
#include <iomanip>
#include <thread>
#include <atomic>
#include <cerrno>
#include <cstring>
#include <map>
#include <chrono>

class KeyboardTeleopNode : public rclcpp::Node
{
public:
  KeyboardTeleopNode()
    : Node("keyboard_teleop_node")
  {
    // Declare parameters
    this->declare_parameter<int>("control_mode", 0);  // 0: step mode, 1: continuous mode
    this->declare_parameter<double>("linear_vel_step", 0.1);
    this->declare_parameter<double>("angular_vel_step", 0.1);
    this->declare_parameter<double>("height_step", 0.01);
    this->declare_parameter<double>("max_linear_vel", 1.0);
    this->declare_parameter<double>("max_angular_vel", 1.0);
    this->declare_parameter<double>("min_height", 0.2);
    this->declare_parameter<double>("max_height", 0.3);
    this->declare_parameter<double>("default_height", 0.2);
    this->declare_parameter<double>("linear_vel_rate", 1.0);   // m/s per second for mode 1
    this->declare_parameter<double>("angular_vel_rate", 1.0);  // rad/s per second for mode 1
    this->declare_parameter<bool>("use_rate_in_continuous_mode", false);  // For mode 1: true=use rate, false=instant max speed
    this->declare_parameter<std::string>("prefix", "");
    this->declare_parameter<std::string>("motion_command_topic", "motion_command");
    this->declare_parameter<std::string>("height_command_topic", "height_command");

    // Get parameters
    control_mode_ = this->get_parameter("control_mode").as_int();
    linear_vel_step_ = this->get_parameter("linear_vel_step").as_double();
    angular_vel_step_ = this->get_parameter("angular_vel_step").as_double();
    height_step_ = this->get_parameter("height_step").as_double();
    max_linear_vel_ = this->get_parameter("max_linear_vel").as_double();
    max_angular_vel_ = this->get_parameter("max_angular_vel").as_double();
    min_height_ = this->get_parameter("min_height").as_double();
    max_height_ = this->get_parameter("max_height").as_double();
    default_height_ = this->get_parameter("default_height").as_double();
    linear_vel_rate_ = this->get_parameter("linear_vel_rate").as_double();
    angular_vel_rate_ = this->get_parameter("angular_vel_rate").as_double();
    use_rate_in_continuous_mode_ = this->get_parameter("use_rate_in_continuous_mode").as_bool();
    std::string prefix = this->get_parameter("prefix").as_string();
    std::string motion_topic = this->get_parameter("motion_command_topic").as_string();
    std::string height_topic = this->get_parameter("height_command_topic").as_string();
    
    // Validate control mode
    if (control_mode_ != 0 && control_mode_ != 1) {
      RCLCPP_WARN(this->get_logger(), 
        "Invalid control_mode %d, using default mode 0", control_mode_);
      control_mode_ = 0;
    }
    
    RCLCPP_INFO(this->get_logger(), 
      "Control mode: %d (%s)", control_mode_, 
      control_mode_ == 0 ? "Step mode" : "Continuous mode");
    if (control_mode_ == 1) {
      RCLCPP_INFO(this->get_logger(), 
        "Continuous mode subtype: %s", 
        use_rate_in_continuous_mode_ ? "Rate-based (gradual change)" : "Instant (max speed on press)");
    }

    // Add namespace prefix to topic names if specified
    if (!prefix.empty()) {
      // Ensure prefix starts with '/' and doesn't end with '/'
      if (prefix[0] != '/') {
        prefix = "/" + prefix;
      }
      // Remove trailing '/' if present
      if (prefix.back() == '/') {
        prefix.pop_back();
      }
      motion_topic = prefix + "/" + motion_topic;
      height_topic = prefix + "/" + height_topic;
      RCLCPP_INFO(this->get_logger(), 
        "Using namespace prefix: '%s'. Topics: '%s', '%s'",
        prefix.c_str(), motion_topic.c_str(), height_topic.c_str());
    }

    // Create publishers
    motion_cmd_pub_ = this->create_publisher<geometry_msgs::msg::Twist>(motion_topic, 10);
    height_cmd_pub_ = this->create_publisher<std_msgs::msg::Float64>(height_topic, 10);
    
    RCLCPP_INFO(this->get_logger(), 
      "Publishing to topics: '%s', '%s'",
      motion_cmd_pub_->get_topic_name(), height_cmd_pub_->get_topic_name());

    // Initialize command values
    current_lin_vel_x_ = 0.0;
    current_lin_vel_y_ = 0.0;
    current_ang_vel_z_ = 0.0;
    current_height_ = default_height_;

    // Start keyboard input thread
    running_ = true;
    keyboard_thread_ = std::thread(&KeyboardTeleopNode::keyboardInputLoop, this);

    // Start command publishing timer
    publish_timer_ = this->create_wall_timer(
      std::chrono::milliseconds(50),  // 20 Hz
      std::bind(&KeyboardTeleopNode::publishCommands, this));
    
    // Start command update timer for continuous mode
    if (control_mode_ == 1) {
      update_timer_ = this->create_wall_timer(
        std::chrono::milliseconds(20),  // 50 Hz for smooth updates
        std::bind(&KeyboardTeleopNode::updateCommands, this));
      last_update_time_ = std::chrono::steady_clock::now();
    }

    RCLCPP_INFO(this->get_logger(), "Keyboard teleop node started");
    printInstructions();
  }

  ~KeyboardTeleopNode()
  {
    running_ = false;
    if (keyboard_thread_.joinable()) {
      keyboard_thread_.join();
    }
    // Restore terminal settings
    restoreTerminal();
  }

private:
  void printInstructions()
  {
    // Print instructions once at startup
    printStatus(true);
  }
  
  void printStatus(bool print_instructions = false)
  {
    if (print_instructions || !instructions_printed_) {
      // Clear screen and move cursor to top-left
      std::cout << "\033[2J\033[H";  // ANSI escape codes: clear screen and home cursor
      
      // Print instructions (static content)
      std::cout << "========================================\n";
      std::cout << "Keyboard Teleoperation Control\n";
      std::cout << "========================================\n";
      std::cout << "Control Mode: " << (control_mode_ == 0 ? "Step Mode" : "Continuous Mode") << "\n";
      if (control_mode_ == 0) {
        std::cout << "  (Each key press increases/decreases by a fixed step)\n";
      } else {
        if (use_rate_in_continuous_mode_) {
          std::cout << "  (Hold keys to continuously change at rate, release to return to zero)\n";
        } else {
          std::cout << "  (Press keys for instant max speed, release to return to zero)\n";
        }
      }
      std::cout << "Note: Height control always uses step mode\n";
      std::cout << "\n";
      std::cout << "Movement Control:\n";
      std::cout << "  w/W : Forward velocity (lin_vel_x)\n";
      std::cout << "  s/S : Backward velocity (lin_vel_x)\n";
      std::cout << "  q/Q : Left velocity (lin_vel_y)\n";
      std::cout << "  e/E : Right velocity (lin_vel_y)\n";
      std::cout << "  a/A : Turn left (ang_vel_z)\n";
      std::cout << "  d/D : Turn right (ang_vel_z)\n";
      std::cout << "\n";
      std::cout << "Height Control (always step mode):\n";
      std::cout << "  t/T : Increase height\n";
      std::cout << "  g/G : Decrease height\n";
      std::cout << "\n";
      std::cout << "Other:\n";
      std::cout << "  Space : Stop all motion (reset velocities to 0)\n";
      std::cout << "  r/R   : Reset height to default\n";
      std::cout << "  x/X   : Exit program\n";
      std::cout << "========================================\n";
      std::cout << "\n";
      std::cout << "Current Command Values:\n";
      instructions_printed_ = true;
    }
    
    // Only update command values if instructions have been printed
    if (instructions_printed_) {
      if (!print_instructions) {
        // Move cursor back to the "Current Command Values" line
        // Instructions take about 25 lines, so move to line 26
        std::cout << "\033[26;1H";  // Move cursor to line 26, column 1
        // Clear from cursor to end of screen to remove any leftover content
        std::cout << "\033[J";  // Clear from cursor to end of screen
      }
      
      // Print current command values (dynamic content)
      std::cout << "  lin_vel_x:  " << std::fixed << std::setprecision(3) << std::setw(8) << current_lin_vel_x_ 
                << " m/s  (max: " << max_linear_vel_ << " m/s)\n";
      std::cout << "  lin_vel_y:  " << std::fixed << std::setprecision(3) << std::setw(8) << current_lin_vel_y_ 
                << " m/s  (max: " << max_linear_vel_ << " m/s)\n";
      std::cout << "  ang_vel_z:  " << std::fixed << std::setprecision(3) << std::setw(8) << current_ang_vel_z_ 
                << " rad/s  (max: " << max_angular_vel_ << " rad/s)\n";
      std::cout << "  height:     " << std::fixed << std::setprecision(3) << std::setw(8) << current_height_ 
                << " m  (range: " << min_height_ << " - " << max_height_ << " m)\n";
      std::cout << "\n";
      std::cout.flush();  // Ensure output is displayed immediately
    }
  }

  void keyboardInputLoop()
  {
    // Check if stdin is a terminal
    if (!isatty(STDIN_FILENO)) {
      RCLCPP_ERROR(this->get_logger(), 
        "stdin is not a terminal! Please run this node in a terminal.");
      return;
    }

    // Save terminal settings
    struct termios old_termios;
    if (tcgetattr(STDIN_FILENO, &old_termios) != 0) {
      RCLCPP_ERROR(this->get_logger(), "Failed to get terminal attributes: %s", strerror(errno));
      return;
    }
    old_termios_ = old_termios;
    old_termios_set_ = true;

    // Set terminal to raw mode
    struct termios new_termios = old_termios;
    new_termios.c_lflag &= ~(ICANON | ECHO);
    new_termios.c_cc[VMIN] = 0;
    new_termios.c_cc[VTIME] = 0;
    if (tcsetattr(STDIN_FILENO, TCSANOW, &new_termios) != 0) {
      RCLCPP_ERROR(this->get_logger(), "Failed to set terminal to raw mode: %s", strerror(errno));
      return;
    }

    // Set stdin to non-blocking
    int flags = fcntl(STDIN_FILENO, F_GETFL);
    if (flags == -1) {
      RCLCPP_ERROR(this->get_logger(), "Failed to get file flags: %s", strerror(errno));
      restoreTerminal();
      return;
    }
    if (fcntl(STDIN_FILENO, F_SETFL, flags | O_NONBLOCK) == -1) {
      RCLCPP_ERROR(this->get_logger(), "Failed to set non-blocking mode: %s", strerror(errno));
      restoreTerminal();
      return;
    }

    RCLCPP_INFO(this->get_logger(), "Keyboard input loop started. Waiting for input...");

    char buffer[1];  // Buffer to read multiple characters at once
    while (rclcpp::ok() && running_) {
      // Read all available characters in the buffer to handle simultaneous key presses
      // Use a larger buffer to read multiple characters in one call
      ssize_t bytes_read = read(STDIN_FILENO, buffer, sizeof(buffer));

      if (bytes_read > 0) {
        // Process all characters read in this batch
        for (ssize_t i = 0; i < bytes_read; ++i) {
          processKeyInput(buffer[i]);
        }
      } else if (bytes_read < 0) {
        // Check if it's just EAGAIN/EWOULDBLOCK (expected for non-blocking I/O)
        if (errno != EAGAIN && errno != EWOULDBLOCK) {
          RCLCPP_WARN(this->get_logger(), "Error reading from stdin: %s", strerror(errno));
        }
      }
      // No sleep or very short sleep for maximum responsiveness
      // The non-blocking read will return immediately if no data is available
      std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }

    // Restore terminal settings
    restoreTerminal();
  }

  void processKeyInput(char c)
  {
    std::lock_guard<std::mutex> lock(cmd_mutex_);

    if (control_mode_ == 0) {
      // Mode 0: Step mode (original behavior)
      switch (c) {
        // Forward/Backward
        case 'w':
        case 'W':
          current_lin_vel_x_ = std::min(current_lin_vel_x_ + linear_vel_step_, max_linear_vel_);
          // RCLCPP_INFO(this->get_logger(), "lin_vel_x: %.3f", current_lin_vel_x_);
          break;
        case 's':
        case 'S':
          current_lin_vel_x_ = std::max(current_lin_vel_x_ - linear_vel_step_, -max_linear_vel_);
          // RCLCPP_INFO(this->get_logger(), "lin_vel_x: %.3f", current_lin_vel_x_);
          break;

        // Left/Right (lin_vel_y)
        case 'q':
        case 'Q':
          current_lin_vel_y_ = std::min(current_lin_vel_y_ + linear_vel_step_, max_linear_vel_);
          // RCLCPP_INFO(this->get_logger(), "lin_vel_y: %.3f", current_lin_vel_y_);
          break;
        case 'e':
        case 'E':
          current_lin_vel_y_ = std::max(current_lin_vel_y_ - linear_vel_step_, -max_linear_vel_);
          // RCLCPP_INFO(this->get_logger(), "lin_vel_y: %.3f", current_lin_vel_y_);
          break;

        // Angular velocity (turn left/right)
        case 'a':
        case 'A':
          current_ang_vel_z_ = std::min(current_ang_vel_z_ + angular_vel_step_, max_angular_vel_);
          // RCLCPP_INFO(this->get_logger(), "ang_vel_z: %.3f", current_ang_vel_z_);
          break;
        case 'd':
        case 'D':
          current_ang_vel_z_ = std::max(current_ang_vel_z_ - angular_vel_step_, -max_angular_vel_);
          // RCLCPP_INFO(this->get_logger(), "ang_vel_z: %.3f", current_ang_vel_z_);
          break;

        // Height control
        case 't':
        case 'T':
          current_height_ = std::min(current_height_ + height_step_, max_height_);
          // RCLCPP_INFO(this->get_logger(), "height: %.3f", current_height_);
          break;
        case 'g':
        case 'G':
          current_height_ = std::max(current_height_ - height_step_, min_height_);
          // RCLCPP_INFO(this->get_logger(), "height: %.3f", current_height_);
          break;

        // Stop all motion
        case ' ':
          current_lin_vel_x_ = 0.0;
          current_lin_vel_y_ = 0.0;
          current_ang_vel_z_ = 0.0;
          // RCLCPP_INFO(this->get_logger(), "Stopped all motion");
          break;

        // Reset height
        case 'r':
        case 'R':
          current_height_ = default_height_;
          // RCLCPP_INFO(this->get_logger(), "Height reset to %.3fm", default_height_);
          break;

        // Exit
        case 'x':
        case 'X':
          RCLCPP_INFO(this->get_logger(), "Exiting...");
          running_ = false;
          rclcpp::shutdown();
          break;
      }
    } else {
      // Mode 1: Continuous mode (update key state)
      auto now = std::chrono::steady_clock::now();

      // Update the timestamp for the pressed key
      // This allows multiple keys to be pressed simultaneously
      switch (c) {
        // Forward/Backward
        case 'w':
        case 'W':
          pressed_keys_['w'] = now;
          // Don't remove other keys - allow simultaneous presses
          break;
        case 's':
        case 'S':
          pressed_keys_['s'] = now;
          // Don't remove other keys - allow simultaneous presses
          break;

        // Left/Right (lin_vel_y)
        case 'q':
        case 'Q':
          pressed_keys_['q'] = now;
          // Don't remove other keys - allow simultaneous presses
          break;
        case 'e':
        case 'E':
          pressed_keys_['e'] = now;
          // Don't remove other keys - allow simultaneous presses
          break;

        // Angular velocity (turn left/right)
        case 'a':
        case 'A':
          pressed_keys_['a'] = now;
          // Don't remove other keys - allow simultaneous presses
          break;
        case 'd':
        case 'D':
          pressed_keys_['d'] = now;
          // Don't remove other keys - allow simultaneous presses
          break;

        // Height control (always uses step mode, regardless of control_mode)
        case 't':
        case 'T':
          current_height_ = std::min(current_height_ + height_step_, max_height_);
          // RCLCPP_INFO(this->get_logger(), "height: %.3f", current_height_);
          break;
        case 'g':
        case 'G':
          current_height_ = std::max(current_height_ - height_step_, min_height_);
          // RCLCPP_INFO(this->get_logger(), "height: %.3f", current_height_);
          break;

        // Stop all motion
        case ' ':
          current_lin_vel_x_ = 0.0;
          current_lin_vel_y_ = 0.0;
          current_ang_vel_z_ = 0.0;
          pressed_keys_.clear();
          // RCLCPP_INFO(this->get_logger(), "Stopped all motion");
          break;

        // Reset height
        case 'r':
        case 'R':
          current_height_ = default_height_;
          // RCLCPP_INFO(this->get_logger(), "Height reset to %.3fm", default_height_);
          break;

        // Exit
        case 'x':
        case 'X':
          RCLCPP_INFO(this->get_logger(), "Exiting...");
          running_ = false;
          rclcpp::shutdown();
          break;
      }
    }
  }
  
  void updateCommands()
  {
    if (control_mode_ != 1) {
      return;
    }
    
    std::lock_guard<std::mutex> lock(cmd_mutex_);
    
    auto now = std::chrono::steady_clock::now();
    auto dt = std::chrono::duration<double>(now - last_update_time_).count();
    last_update_time_ = now;
    
    // Key timeout: if a key hasn't been updated in 500ms, consider it released
    // Increased timeout to handle key repeat delays better
    auto timeout_threshold = now - std::chrono::milliseconds(500);
    
    // Clean up timed-out keys from pressed_keys_ map
    // This ensures that when user releases a key, it will be removed after timeout
    // auto it = pressed_keys_.begin();
    // while (it != pressed_keys_.end()) {
    //   if (it->second < timeout_threshold) {
    //     // Key has timed out, remove it
    //     it = pressed_keys_.erase(it);
    //   } else {
    //     ++it;
    //   }
    // }

    if (use_rate_in_continuous_mode_) {
      // Mode 1a: Rate-based (gradual change)
      // Update lin_vel_x
      bool w_active = pressed_keys_.find('w') != pressed_keys_.end() &&
                      pressed_keys_['w'] > timeout_threshold;
      bool s_active = pressed_keys_.find('s') != pressed_keys_.end() && 
                      pressed_keys_['s'] > timeout_threshold;
      
      if (w_active) {
        current_lin_vel_x_ = std::min(current_lin_vel_x_ + linear_vel_rate_ * dt, max_linear_vel_);
      } else if (s_active) {
        current_lin_vel_x_ = std::max(current_lin_vel_x_ - linear_vel_rate_ * dt, -max_linear_vel_);
      } else {
        // Decay to zero - key is either not pressed or has timed out
        if (current_lin_vel_x_ > 0) {
          current_lin_vel_x_ = std::max(0.0, current_lin_vel_x_ - linear_vel_rate_ * dt);
        } else if (current_lin_vel_x_ < 0) {
          current_lin_vel_x_ = std::min(0.0, current_lin_vel_x_ + linear_vel_rate_ * dt);
        }
      }
      
      // Update lin_vel_y
      bool q_active = pressed_keys_.find('q') != pressed_keys_.end() && 
                      pressed_keys_['q'] > timeout_threshold;
      bool e_active = pressed_keys_.find('e') != pressed_keys_.end() && 
                      pressed_keys_['e'] > timeout_threshold;
      
      if (q_active) {
        current_lin_vel_y_ = std::min(current_lin_vel_y_ + linear_vel_rate_ * dt, max_linear_vel_);
      } else if (e_active) {
        current_lin_vel_y_ = std::max(current_lin_vel_y_ - linear_vel_rate_ * dt, -max_linear_vel_);
      } else {
        // Decay to zero
        if (current_lin_vel_y_ > 0) {
          current_lin_vel_y_ = std::max(0.0, current_lin_vel_y_ - linear_vel_rate_ * dt);
        } else if (current_lin_vel_y_ < 0) {
          current_lin_vel_y_ = std::min(0.0, current_lin_vel_y_ + linear_vel_rate_ * dt);
        }
      }
      
      // Update ang_vel_z
      bool a_active = pressed_keys_.find('a') != pressed_keys_.end() && 
                      pressed_keys_['a'] > timeout_threshold;
      bool d_active = pressed_keys_.find('d') != pressed_keys_.end() && 
                      pressed_keys_['d'] > timeout_threshold;
      
      if (a_active) {
        current_ang_vel_z_ = std::min(current_ang_vel_z_ + angular_vel_rate_ * dt, max_angular_vel_);
      } else if (d_active) {
        current_ang_vel_z_ = std::max(current_ang_vel_z_ - angular_vel_rate_ * dt, -max_angular_vel_);
      } else {
        // Decay to zero
        if (current_ang_vel_z_ > 0) {
          current_ang_vel_z_ = std::max(0.0, current_ang_vel_z_ - angular_vel_rate_ * dt);
        } else if (current_ang_vel_z_ < 0) {
          current_ang_vel_z_ = std::min(0.0, current_ang_vel_z_ + angular_vel_rate_ * dt);
        }
      }
    } else {
      // Mode 1b: Instant max speed (no rate)
      // Update lin_vel_x
      bool w_active = pressed_keys_.find('w') != pressed_keys_.end() &&
                      pressed_keys_['w'] > timeout_threshold;
      bool s_active = pressed_keys_.find('s') != pressed_keys_.end() && 
                      pressed_keys_['s'] > timeout_threshold;
      
      if (w_active) {
        current_lin_vel_x_ = max_linear_vel_;
      } else if (s_active) {
        current_lin_vel_x_ = -max_linear_vel_;
      } else {
        current_lin_vel_x_ = 0.0;
      }
      
      // Update lin_vel_y
      bool q_active = pressed_keys_.find('q') != pressed_keys_.end() && 
                      pressed_keys_['q'] > timeout_threshold;
      bool e_active = pressed_keys_.find('e') != pressed_keys_.end() && 
                      pressed_keys_['e'] > timeout_threshold;
      
      if (q_active) {
        current_lin_vel_y_ = max_linear_vel_;
      } else if (e_active) {
        current_lin_vel_y_ = -max_linear_vel_;
      } else {
        current_lin_vel_y_ = 0.0;
      }
      
      // Update ang_vel_z
      bool a_active = pressed_keys_.find('a') != pressed_keys_.end() && 
                      pressed_keys_['a'] > timeout_threshold;
      bool d_active = pressed_keys_.find('d') != pressed_keys_.end() && 
                      pressed_keys_['d'] > timeout_threshold;
      
      if (a_active) {
        current_ang_vel_z_ = max_angular_vel_;
      } else if (d_active) {
        current_ang_vel_z_ = -max_angular_vel_;
      } else {
        current_ang_vel_z_ = 0.0;
      }
    }
    
    // Height control always uses step mode, so no update needed here
  }

  void publishCommands()
  {
    std::lock_guard<std::mutex> lock(cmd_mutex_);

    // Publish motion command
    auto twist_msg = geometry_msgs::msg::Twist();
    twist_msg.linear.x = current_lin_vel_x_;
    twist_msg.linear.y = current_lin_vel_y_;
    twist_msg.linear.z = 0.0;
    twist_msg.angular.x = 0.0;
    twist_msg.angular.y = 0.0;
    twist_msg.angular.z = current_ang_vel_z_;
    motion_cmd_pub_->publish(twist_msg);

    // Publish height command
    auto height_msg = std_msgs::msg::Float64();
    height_msg.data = current_height_;
    height_cmd_pub_->publish(height_msg);
    
    // Update display with current command values
    printStatus(false);
  }

  void restoreTerminal()
  {
    if (old_termios_set_) {
      tcsetattr(STDIN_FILENO, TCSANOW, &old_termios_);
      int flags = fcntl(STDIN_FILENO, F_GETFL);
      fcntl(STDIN_FILENO, F_SETFL, flags & ~O_NONBLOCK);
    }
  }

  // Publishers
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr motion_cmd_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr height_cmd_pub_;

  // Timer
  rclcpp::TimerBase::SharedPtr publish_timer_;
  rclcpp::TimerBase::SharedPtr update_timer_;

  // Command values
  double current_lin_vel_x_;
  double current_lin_vel_y_;
  double current_ang_vel_z_;
  double current_height_;

  // Parameters
  int control_mode_;  // 0: step mode, 1: continuous mode
  double linear_vel_step_;
  double angular_vel_step_;
  double height_step_;
  double max_linear_vel_;
  double max_angular_vel_;
  double min_height_;
  double max_height_;
  double default_height_;
  double linear_vel_rate_;   // For continuous mode (m/s per second)
  double angular_vel_rate_;  // For continuous mode (rad/s per second)
  bool use_rate_in_continuous_mode_;  // For mode 1: true=use rate, false=instant max speed

  // Continuous mode state
  std::map<char, std::chrono::steady_clock::time_point> pressed_keys_;
  std::chrono::steady_clock::time_point last_update_time_;

  // Threading
  std::thread keyboard_thread_;
  std::atomic<bool> running_;
  std::mutex cmd_mutex_;

  // Display state
  bool instructions_printed_{false};

  // Terminal settings
  struct termios old_termios_;
  std::atomic<bool> old_termios_set_{false};
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<KeyboardTeleopNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}

