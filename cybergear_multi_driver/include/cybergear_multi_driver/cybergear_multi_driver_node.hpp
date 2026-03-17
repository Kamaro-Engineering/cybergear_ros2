// MIT License
//
// Copyright (c) 2024 Maintainer
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in all
// copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.

#pragma once

#include <linux/can.h>
#include <linux/can/raw.h>

#include <atomic>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <unordered_map>
#include <vector>

#include <cybergear_driver_core/cybergear_driver_core.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <std_srvs/srv/set_bool.hpp>
#include <std_srvs/srv/trigger.hpp>

#include "ros2_socketcan/socket_can_receiver.hpp"
#include "ros2_socketcan/socket_can_sender.hpp"


#include <cybergear_multi_driver/srv/set_motor_gain.hpp>

namespace cybergear_multi_driver
{

// ---------------------------------------------------------------------------
// Configuration for one motor (loaded from ROS parameters)
// ---------------------------------------------------------------------------
struct MotorConfig
{
  int can_id{127};
  std::string can_interface{"can0"};
  int primary_id{0};
  std::string name{"motor"};
  double kp{10.0};
  double kd{0.5};
};

// ---------------------------------------------------------------------------
// ---------------------------------------------------------------------------
// Per-motor state.
// unique_ptr<CybergearPacket> makes MotorState movable so std::vector works.
// Two node-level mutexes (cmd_mutex_, feedback_mutex_) protect the mutable
// fields — the critical sections are tiny so single-mutex-per-class is fine.
// ---------------------------------------------------------------------------
struct MotorState
{
  MotorConfig config;
  std::unique_ptr<cybergear_driver_core::CybergearPacket> packet;

  // ---- feedback (protected by Node::feedback_mutex_) ----
  float fb_pos{0.0f};
  float fb_vel{0.0f};
  float fb_eff{0.0f};
  float fb_temp{0.0f};
  bool  fb_valid{false};

  // ---- command (protected by Node::cmd_mutex_) ----
  float cmd_pos{0.0f};
  float cmd_vel{0.0f};
  float cmd_eff{0.0f};
  float kp{1.0f};
  float kd{0.1f};
  // nanoseconds since epoch of last command (0 = never received)
  int64_t last_cmd_ns{0};
};

// ---------------------------------------------------------------------------
// One CAN interface (socket + receive-thread)
// ---------------------------------------------------------------------------
struct CanIface
{
  std::string name;
  std::chrono::nanoseconds timeout_ns_;
  std::unique_ptr<drivers::socketcan::SocketCanSender> sender_;
  std::chrono::nanoseconds interval_ns_;
  std::string can_filters_ = "0:0";
  std::unique_ptr<drivers::socketcan::SocketCanReceiver> receiver_;
  std::thread rx_thread;
  std::atomic<bool> running{false};
  mutable std::mutex send_mtx;
};

// ---------------------------------------------------------------------------
// Main node
// ---------------------------------------------------------------------------
class CybergearMultiDriverNode : public rclcpp::Node
{
public:
  explicit CybergearMultiDriverNode(
    const rclcpp::NodeOptions & options = rclcpp::NodeOptions{});
  ~CybergearMultiDriverNode() override;

private:
  // ---- parameters ----
  double send_frequency_{100.0};
  double command_timeout_ms_{500.0};

  // ---- motor state ----
  std::vector<MotorState> motors_;

  // Protects cmd_pos/vel/eff/kp/kd/last_cmd_ns across all motors.
  // Held briefly by subscription callbacks and the send timer.
  mutable std::mutex cmd_mutex_;
  // Protects fb_pos/vel/eff/temp/valid across all motors.
  // Held briefly by the receive thread and the send timer / joint state publisher.
  mutable std::mutex feedback_mutex_;

  // device_id → motor_index per CAN interface, for fast receive dispatch
  std::unordered_map<std::string,
    std::unordered_map<uint8_t, size_t>> iface_device_map_;

  // Per-interface state (sender, receiver, rx_thread)
  std::unordered_map<std::string, CanIface> can_ifaces_;

  // ---- ROS subscriptions ----
  rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr pos_sub_;
  rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr vel_sub_;
  rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr eff_sub_;

  // ---- ROS publishers ----
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_state_pub_;

  // ---- ROS services ----
  rclcpp::Service<cybergear_multi_driver::srv::SetMotorGain>::SharedPtr set_kp_srv_;
  rclcpp::Service<cybergear_multi_driver::srv::SetMotorGain>::SharedPtr set_kd_srv_;
  rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr enable_torque_srv_;

  // ---- send timer ----
  rclcpp::TimerBase::SharedPtr send_timer_;

  // ---- helpers ----
  void loadParameters();
  void initPackets();
  void initCanInterfaces();
  void shutdownCanInterfaces();
  bool openCanSocket(CanIface & iface);
  void closeCanSocket(CanIface & iface);

  // Sends a CanFrame on the given interface (thread-safe).
  // Returns false and logs a warning on error.
  bool sendFrame(const std::string & iface_name,
    const cybergear_driver_core::CanFrame & frame);

  // Motor initialisation sequence (reset → set op mode → enable torque)
  void initMotor(size_t idx);
  void disableMotorTorque(size_t idx);

  // Builds a zero-data CanFrame with the given extended ID
  static cybergear_driver_core::CanFrame makeControlFrame(uint32_t id);

  // Receive loop – one thread per CAN interface
  void receiveLoop(const std::string & iface_name);

  // Timer callback – sends move commands for all motors
  void sendTimerCb();

  // Subscription callbacks
  void positionCb(const std_msgs::msg::Float64MultiArray::SharedPtr msg);
  void velocityCb(const std_msgs::msg::Float64MultiArray::SharedPtr msg);
  void effortCb(const std_msgs::msg::Float64MultiArray::SharedPtr msg);

  // Service callbacks
  void setKpCb(
    const std::shared_ptr<cybergear_multi_driver::srv::SetMotorGain::Request> req,
    std::shared_ptr<cybergear_multi_driver::srv::SetMotorGain::Response> res);
  void setKdCb(
    const std::shared_ptr<cybergear_multi_driver::srv::SetMotorGain::Request> req,
    std::shared_ptr<cybergear_multi_driver::srv::SetMotorGain::Response> res);
  void enableTorqueCb(
    const std::shared_ptr<std_srvs::srv::SetBool::Request> req,
    std::shared_ptr<std_srvs::srv::SetBool::Response> res);

  void publishJointStates();
};

}  // namespace cybergear_multi_driver
