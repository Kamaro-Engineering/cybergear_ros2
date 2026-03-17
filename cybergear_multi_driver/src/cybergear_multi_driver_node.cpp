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
// The above copyright notice and this permission notice shall be included in
// all copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.

#include "cybergear_multi_driver/cybergear_multi_driver_node.hpp"

#include <net/if.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <unistd.h>

#include <cerrno>
#include <chrono>
#include <cstring>
#include <stdexcept>

#include "ros2_socketcan/socket_can_receiver.hpp"
#include "ros2_socketcan/socket_can_sender.hpp"

namespace cybergear_multi_driver {

using namespace std::chrono_literals;

// ============================================================================
// Construction / destruction
// ============================================================================

CybergearMultiDriverNode::CybergearMultiDriverNode(
    const rclcpp::NodeOptions& options)
    : Node("cybergear_multi_driver", options) {
  loadParameters();
  initPackets();
  initCanInterfaces();

  // ---- publishers ----
  joint_state_pub_ = create_publisher<sensor_msgs::msg::JointState>(
      "~/joint_states", rclcpp::SensorDataQoS());

  // ---- subscriptions (best-effort, depth 1 – we only care about latest) ----
  auto qos = rclcpp::QoS(1).best_effort();
  pos_sub_ = create_subscription<std_msgs::msg::Float64MultiArray>(
      "~/position", qos,
      [this](const std_msgs::msg::Float64MultiArray::SharedPtr m) {
        positionCb(m);
      });
  vel_sub_ = create_subscription<std_msgs::msg::Float64MultiArray>(
      "~/velocity", qos,
      [this](const std_msgs::msg::Float64MultiArray::SharedPtr m) {
        velocityCb(m);
      });
  eff_sub_ = create_subscription<std_msgs::msg::Float64MultiArray>(
      "~/effort", qos,
      [this](const std_msgs::msg::Float64MultiArray::SharedPtr m) {
        effortCb(m);
      });

  // ---- services ----
  set_kp_srv_ = create_service<cybergear_multi_driver::srv::SetMotorGain>(
      "~/set_kp",
      [this](
          const std::shared_ptr<
              cybergear_multi_driver::srv::SetMotorGain::Request>
              req,
          std::shared_ptr<cybergear_multi_driver::srv::SetMotorGain::Response>
              res) { setKpCb(req, res); });
  set_kd_srv_ = create_service<cybergear_multi_driver::srv::SetMotorGain>(
      "~/set_kd",
      [this](
          const std::shared_ptr<
              cybergear_multi_driver::srv::SetMotorGain::Request>
              req,
          std::shared_ptr<cybergear_multi_driver::srv::SetMotorGain::Response>
              res) { setKdCb(req, res); });
  enable_torque_srv_ = create_service<std_srvs::srv::SetBool>(
      "~/enable_torque",
      [this](const std::shared_ptr<std_srvs::srv::SetBool::Request> req,
             std::shared_ptr<std_srvs::srv::SetBool::Response> res) {
        enableTorqueCb(req, res);
      });

  // ---- send timer ----
  const auto period = std::chrono::duration<double>(1.0 / send_frequency_);
  send_timer_ = create_wall_timer(period, [this]() { sendTimerCb(); });

  RCLCPP_INFO(get_logger(),
              "CybergearMultiDriverNode started with %zu motor(s), "
              "send_freq=%.1f Hz, timeout=%.0f ms",
              motors_.size(), send_frequency_, command_timeout_ms_);
}

CybergearMultiDriverNode::~CybergearMultiDriverNode() {
  // Disable torque on all motors before shutting down
  for (size_t i = 0; i < motors_.size(); ++i) {
    try {
      disableMotorTorque(i);
    } catch (...) {
    }
  }
  shutdownCanInterfaces();
}

// ============================================================================
// Parameter loading
// ============================================================================

void CybergearMultiDriverNode::loadParameters() {
  send_frequency_ = declare_parameter<double>("send_frequency", 100.0);
  command_timeout_ms_ = declare_parameter<double>("command_timeout_ms", 500.0);

  // Motor parameters are flat arrays – all arrays must have the same length.
  // Example YAML:
  //   motor_can_ids:        [1, 2]
  //   motor_can_interfaces: ["can0", "can0"]
  //   motor_primary_ids:    [0, 0]
  //   motor_names:          ["left_hip", "right_hip"]
  //   motor_kps:            [10.0, 10.0]
  //   motor_kds:            [0.5, 0.5]
  const auto can_ids =
      declare_parameter<std::vector<int64_t>>("motor_can_ids", {127});
  const auto can_interfaces = declare_parameter<std::vector<std::string>>(
      "motor_can_interfaces", {"can0"});
  const auto primary_ids =
      declare_parameter<std::vector<int64_t>>("motor_primary_ids", {0});
  const auto names =
      declare_parameter<std::vector<std::string>>("motor_names", {"motor_0"});
  const auto kps = declare_parameter<std::vector<double>>("motor_kps", {10.0});
  const auto kds = declare_parameter<std::vector<double>>("motor_kds", {0.5});

  const size_t n = can_ids.size();
  motors_.reserve(n);

  for (size_t i = 0; i < n; ++i) {
    MotorConfig cfg;
    cfg.can_id = static_cast<int>(can_ids[i]);
    cfg.can_interface =
        (i < can_interfaces.size()) ? can_interfaces[i] : "can0";
    cfg.primary_id =
        (i < primary_ids.size()) ? static_cast<int>(primary_ids[i]) : 0;
    cfg.name = (i < names.size() && !names[i].empty())
                   ? names[i]
                   : "motor_" + std::to_string(i);
    cfg.kp = (i < kps.size()) ? kps[i] : 10.0;
    cfg.kd = (i < kds.size()) ? kds[i] : 0.5;

    if (cfg.can_id == cfg.primary_id) {
      RCLCPP_ERROR(
          get_logger(),
          "Motor %zu: can_id (%d) must differ from primary_id (%d). Skipping.",
          i, cfg.can_id, cfg.primary_id);
      continue;
    }

    // emplace_back() constructs in-place, avoiding move of non-movable
    // unique_ptr<Packet> makes MotorState movable; push_back is fine.
    motors_.push_back(MotorState{});
    auto& state = motors_.back();
    state.config = cfg;
    state.kp = static_cast<float>(cfg.kp);
    state.kd = static_cast<float>(cfg.kd);

    RCLCPP_INFO(get_logger(),
                "  Motor[%zu] name='%s' can_id=%d iface='%s' primary_id=%d "
                "kp=%.2f kd=%.4f",
                motors_.size() - 1, state.config.name.c_str(),
                state.config.can_id, state.config.can_interface.c_str(),
                state.config.primary_id, state.config.kp, state.config.kd);
  }

  if (motors_.empty()) {
    RCLCPP_WARN(get_logger(),
                "No motors configured. Set 'motor_can_ids', "
                "'motor_can_interfaces', etc.");
  }
}

// ============================================================================
// Packet initialisation
// ============================================================================

void CybergearMultiDriverNode::initPackets() {
  for (auto& motor : motors_) {
    cybergear_driver_core::CybergearPacketParam param;
    param.device_id = motor.config.can_id;
    param.primary_id = motor.config.primary_id;
    motor.packet =
        std::make_unique<cybergear_driver_core::CybergearPacket>(param);
  }

  // Build interface → device_id → motor_index lookup table
  for (size_t i = 0; i < motors_.size(); ++i) {
    const auto& cfg = motors_[i].config;
    iface_device_map_[cfg.can_interface][static_cast<uint8_t>(cfg.can_id)] = i;
  }
}

// ============================================================================
// CAN socket management
// ============================================================================

void CybergearMultiDriverNode::initCanInterfaces() {
  // Collect unique interface names required by configured motors
  for (const auto& motor : motors_) {
    const auto& iface_name = motor.config.can_interface;
    if (can_ifaces_.find(iface_name) == can_ifaces_.end()) {
      can_ifaces_[iface_name].name = iface_name;
    }
  }

  for (auto& [name, iface] : can_ifaces_) {
    try {
      iface.sender_ =
          std::make_unique<drivers::socketcan::SocketCanSender>(name, false);
      iface.receiver_ =
          std::make_unique<drivers::socketcan::SocketCanReceiver>(name, false);
      // apply CAN filters
      iface.receiver_->SetCanFilters(
          drivers::socketcan::SocketCanReceiver::CanFilterList(iface.can_filters_));
      RCLCPP_DEBUG(get_logger(), "applied filters: %s", iface.can_filters_.c_str());
    } catch (const std::exception& ex) {
      RCLCPP_ERROR(get_logger(), "Error opening CAN interface '%s': %s",
                   name.c_str(), ex.what());
      continue;
    }

    iface.running.store(true);
    iface.rx_thread =
        std::thread([this, name]() { receiveLoop(name); });
    RCLCPP_INFO(get_logger(), "CAN interface '%s' opened.", name.c_str());
  }
}

bool CybergearMultiDriverNode::openCanSocket(CanIface& /* iface */) {
  // Socket management is handled by ros2_socketcan in initCanInterfaces
  return true;
}

void CybergearMultiDriverNode::closeCanSocket(CanIface& /* iface */) {
  // Socket management is handled by ros2_socketcan in shutdownCanInterfaces
}

void CybergearMultiDriverNode::shutdownCanInterfaces() {
  for (auto& [name, iface] : can_ifaces_) {
    iface.running.store(false);
    if (iface.rx_thread.joinable()) {
      iface.rx_thread.join();
    }
  }
}

// ============================================================================
// Send helper
// ============================================================================

bool CybergearMultiDriverNode::sendFrame(
    const std::string& iface_name,
    const cybergear_driver_core::CanFrame& frame) {
  // Look up the interface
  const auto iface_it = can_ifaces_.find(iface_name);
  if (iface_it == can_ifaces_.end()) {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000,
                         "CAN interface '%s' not found", iface_name.c_str());
    return false;
  }
  const auto& iface = iface_it->second;

  // Build the CAN ID
  using drivers::socketcan::CanId;
  using drivers::socketcan::ExtendedFrame;
  using drivers::socketcan::FrameType;
  CanId send_id(frame.id, 0, FrameType::DATA, ExtendedFrame);

  try {
    iface.sender_->send(frame.data.data(), frame.data.size(), send_id,
                        iface.timeout_ns_);
  } catch (const std::exception& ex) {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000,
                         "Error sending CAN message on '%s': %s",
                         iface_name.c_str(), ex.what());
    return false;
  }

  return true;
}

// ============================================================================
// Motor initialisation
// ============================================================================

  cybergear_driver_core::CanFrame CybergearMultiDriverNode::makeControlFrame(
      uint32_t id) {
    cybergear_driver_core::CanFrame f;
    f.id = id;
    f.data.fill(0);
    return f;
  }

  void CybergearMultiDriverNode::initMotor(size_t idx) {
    auto& motor = motors_[idx];
    const auto& iface = motor.config.can_interface;

    // 1. Reset torque: puts motor into reset/idle state
    sendFrame(iface,
              makeControlFrame(motor.packet->frameId().getResetTorqueId()));
    rclcpp::sleep_for(20ms);

    // TODO: Make optional - Reset position to zero
    sendFrame(iface, motor.packet->createZeroPosition());
    rclcpp::sleep_for(20ms);

    // 2. Switch to operation (MIT) mode
    sendFrame(iface, motor.packet->createChangeRunMode(
                         cybergear_driver_core::run_modes::OPERATION));
    rclcpp::sleep_for(20ms);

    // 3. Enable torque
    sendFrame(iface,
              makeControlFrame(motor.packet->frameId().getEnableTorqueId()));
    rclcpp::sleep_for(10ms);

    RCLCPP_INFO(get_logger(),
                "Motor[%zu] '%s' (can_id=%d) initialised in OPERATION mode.",
                idx, motor.config.name.c_str(), motor.config.can_id);
  }

  void CybergearMultiDriverNode::disableMotorTorque(size_t idx) {
    auto& motor = motors_[idx];
    sendFrame(motor.config.can_interface,
              makeControlFrame(motor.packet->frameId().getResetTorqueId()));
  }

  // ============================================================================
  // Receive loop (one per CAN interface, runs in its own thread)
  // ============================================================================

  void CybergearMultiDriverNode::receiveLoop(const std::string& iface_name) {
    auto& iface = can_ifaces_.at(iface_name);
    RCLCPP_DEBUG(get_logger(), "RX thread started for '%s'",
                 iface_name.c_str());

    // Look up the device-id→motor-index map for this interface
    const auto map_it = iface_device_map_.find(iface_name);
    if (map_it == iface_device_map_.end()) {
      RCLCPP_ERROR(get_logger(), "No motors mapped to interface '%s'",
                   iface_name.c_str());
      return;
    }
    const auto& device_map = map_it->second;

    while (rclcpp::ok()) {
      while (!iface.running.load() && rclcpp::ok()) {
        std::this_thread::yield();
      }

      drivers::socketcan::CanId can_id;
      std::array<uint8_t, 8> rx_data{};

      try {
        can_id = iface.receiver_->receive(rx_data.data(), iface.interval_ns_);
      } catch (const std::exception& ex) {
        if (!iface.running.load()) {
          break;  // interface shut down
        }
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                             "CAN receive error on '%s': %s",
                             iface_name.c_str(), ex.what());
        continue;
      }

      // Get the raw 29-bit ID
      const uint32_t raw_id = can_id.identifier() & CAN_EFF_MASK;

      // Identify the sending device by the bits[8..15] of the extended ID
      // (matches CybergearFrameId::DEVICE_ID_MASK >> DEVICE_ID_OFFSET)
      const uint8_t sender_device_id =
          static_cast<uint8_t>((raw_id & 0x0000ff00u) >> 8u);

      // Find which motor this frame belongs to
      const auto motor_it = device_map.find(sender_device_id);
      if (motor_it == device_map.end()) {
        continue;  // frame not from one of our motors
      }
      const size_t motor_idx = motor_it->second;
      auto& motor = motors_[motor_idx];

      // Check it is a feedback frame (command byte == FEEDBACK == 2)
      const uint8_t cmd_byte =
          static_cast<uint8_t>((raw_id & 0xff000000u) >> 24u);
      if (cmd_byte != cybergear_driver_core::commands::FEEDBACK) {
        if (cmd_byte == cybergear_driver_core::commands::FAULT_FEEDBACK) {
          RCLCPP_WARN_THROTTLE(
              get_logger(), *get_clock(), 1000,
              "Motor[%zu] '%s' reported a fault frame (id=0x%08X)", motor_idx,
              motor.config.name.c_str(), raw_id);
        }
        continue;
      }

      // Parse feedback using the cybergear_driver_core packet
      cybergear_driver_core::CanData data;
      std::copy(rx_data.begin(), rx_data.end(), data.begin());

      const float pos = motor.packet->parsePosition(data);
      const float vel = motor.packet->parseVelocity(data);
      const float eff = motor.packet->parseEffort(data);
      const float temp = motor.packet->parseTemperature(data);

      {
        std::lock_guard<std::mutex> lk(feedback_mutex_);
        motor.fb_pos = pos;
        motor.fb_vel = vel;
        motor.fb_eff = eff;
        motor.fb_temp = temp;
        motor.fb_valid = true;
      }
    }

    RCLCPP_DEBUG(get_logger(), "RX thread exited for '%s'", iface_name.c_str());
  }

  // ============================================================================
  // Send timer callback
  // ============================================================================

  void CybergearMultiDriverNode::sendTimerCb() {
    const int64_t now_ns = this->get_clock()->now().nanoseconds();
    const int64_t timeout_ns =
        static_cast<int64_t>(command_timeout_ms_ * 1'000'000LL);

    for (size_t i = 0; i < motors_.size(); ++i) {
      auto& motor = motors_[i];

      cybergear_driver_core::MoveParam param;

      {
        std::lock_guard<std::mutex> clk(cmd_mutex_);
        const bool timed_out = (motor.last_cmd_ns == 0) ||
                               ((now_ns - motor.last_cmd_ns) > timeout_ns);

        if (timed_out) {
          // Hold at last known feedback position; zero vel and feedforward
          // effort
          std::lock_guard<std::mutex> flk(feedback_mutex_);
          param.position = motor.fb_valid ? motor.fb_pos : 0.0f;
          param.velocity = 0.0f;
          param.effort = 0.0f;
        } else {
          param.position = motor.cmd_pos;
          param.velocity = motor.cmd_vel;
          param.effort = motor.cmd_eff;
        }
        param.kp = motor.kp;
        param.kd = motor.kd;
      }

      const auto frame = motor.packet->createMoveCommand(param);
      sendFrame(motor.config.can_interface, frame);
    }

    publishJointStates();
  }

  // ============================================================================
  // Joint state publisher
  // ============================================================================

  void CybergearMultiDriverNode::publishJointStates() {
    sensor_msgs::msg::JointState msg;
    msg.header.stamp = this->get_clock()->now();
    msg.name.reserve(motors_.size());
    msg.position.reserve(motors_.size());
    msg.velocity.reserve(motors_.size());
    msg.effort.reserve(motors_.size());

    for (const auto& motor : motors_) {
      msg.name.push_back(motor.config.name);
      msg.position.push_back(static_cast<double>(motor.fb_pos));
      msg.velocity.push_back(static_cast<double>(motor.fb_vel));
      msg.effort.push_back(static_cast<double>(motor.fb_eff));
    }

    joint_state_pub_->publish(msg);
  }

  // ============================================================================
  // Subscription callbacks
  // ============================================================================

  void CybergearMultiDriverNode::positionCb(
      const std_msgs::msg::Float64MultiArray::SharedPtr msg) {
    const int64_t now_ns = this->get_clock()->now().nanoseconds();
    const size_t n = std::min(msg->data.size(), motors_.size());
    std::lock_guard<std::mutex> lk(cmd_mutex_);
    for (size_t i = 0; i < n; ++i) {
      motors_[i].cmd_pos = static_cast<float>(msg->data[i]);
      motors_[i].last_cmd_ns = now_ns;
    }
    if (msg->data.size() != motors_.size()) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
                           "position topic size (%zu) != motor count (%zu); "
                           "extra entries ignored.",
                           msg->data.size(), motors_.size());
    }
  }

  void CybergearMultiDriverNode::velocityCb(
      const std_msgs::msg::Float64MultiArray::SharedPtr msg) {
    const int64_t now_ns = this->get_clock()->now().nanoseconds();
    const size_t n = std::min(msg->data.size(), motors_.size());
    std::lock_guard<std::mutex> lk(cmd_mutex_);
    for (size_t i = 0; i < n; ++i) {
      motors_[i].cmd_vel = static_cast<float>(msg->data[i]);
      motors_[i].last_cmd_ns = now_ns;
    }
  }

  void CybergearMultiDriverNode::effortCb(
      const std_msgs::msg::Float64MultiArray::SharedPtr msg) {
    const int64_t now_ns = this->get_clock()->now().nanoseconds();
    const size_t n = std::min(msg->data.size(), motors_.size());
    std::lock_guard<std::mutex> lk(cmd_mutex_);
    for (size_t i = 0; i < n; ++i) {
      motors_[i].cmd_eff = static_cast<float>(msg->data[i]);
      motors_[i].last_cmd_ns = now_ns;
    }
  }

  // ============================================================================
  // Service callbacks
  // ============================================================================

  void CybergearMultiDriverNode::setKpCb(
      const std::shared_ptr<cybergear_multi_driver::srv::SetMotorGain::Request>
          req,
      std::shared_ptr<cybergear_multi_driver::srv::SetMotorGain::Response>
          res) {
    const int idx = req->motor_index;
    if (idx < 0 || static_cast<size_t>(idx) >= motors_.size()) {
      res->success = false;
      res->message = "Invalid motor_index " + std::to_string(idx);
      return;
    }
    if (req->value < 0.0 || req->value > 500.0) {
      res->success = false;
      res->message = "kp must be in [0, 500]";
      return;
    }
    {
      std::lock_guard<std::mutex> lk(cmd_mutex_);
      motors_[idx].kp = static_cast<float>(req->value);
    }
    res->success = true;
    res->message = "kp set to " + std::to_string(req->value) + " for motor " +
                   motors_[idx].config.name;
    RCLCPP_INFO(get_logger(), "%s", res->message.c_str());
  }

  void CybergearMultiDriverNode::setKdCb(
      const std::shared_ptr<cybergear_multi_driver::srv::SetMotorGain::Request>
          req,
      std::shared_ptr<cybergear_multi_driver::srv::SetMotorGain::Response>
          res) {
    const int idx = req->motor_index;
    if (idx < 0 || static_cast<size_t>(idx) >= motors_.size()) {
      res->success = false;
      res->message = "Invalid motor_index " + std::to_string(idx);
      return;
    }
    if (req->value < 0.0 || req->value > 5.0) {
      res->success = false;
      res->message = "kd must be in [0, 5]";
      return;
    }
    {
      std::lock_guard<std::mutex> lk(cmd_mutex_);
      motors_[idx].kd = static_cast<float>(req->value);
    }
    res->success = true;
    res->message = "kd set to " + std::to_string(req->value) + " for motor " +
                   motors_[idx].config.name;
    RCLCPP_INFO(get_logger(), "%s", res->message.c_str());
  }

  void CybergearMultiDriverNode::enableTorqueCb(
      const std::shared_ptr<std_srvs::srv::SetBool::Request> req,
      std::shared_ptr<std_srvs::srv::SetBool::Response> res) {
    if (req->data) {
      for (size_t i = 0; i < motors_.size(); ++i) {
        initMotor(i);
      }
      res->message = "Torque enabled on all motors.";
    } else {
      for (size_t i = 0; i < motors_.size(); ++i) {
        disableMotorTorque(i);
      }
      res->message = "Torque disabled on all motors.";
    }
    res->success = true;
    RCLCPP_INFO(get_logger(), "%s", res->message.c_str());
  }

}  // namespace cybergear_multi_driver

// ============================================================================
// Entry point
// ============================================================================

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  auto node =
      std::make_shared<cybergear_multi_driver::CybergearMultiDriverNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
