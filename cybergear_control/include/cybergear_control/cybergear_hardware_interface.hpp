#pragma once

#include <stdint.h>

#include <atomic>
#include <cstdint>
#include <thread>

#include "cybergear_driver_core/cybergear_packet.hpp"
#include "hardware_interface/actuator_interface.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "realtime_buffer.hpp"
#include "ros2_socketcan/socket_can_receiver.hpp"
#include "ros2_socketcan/socket_can_sender.hpp"

namespace cybergear_control {

using hardware_interface::ActuatorInterface;
using hardware_interface::CallbackReturn;
using hardware_interface::CommandInterface;
using hardware_interface::return_type;
using hardware_interface::StateInterface;

using rclcpp_lifecycle::LifecycleNode;

class CybergearActuator : public ActuatorInterface {
public:
  RCLCPP_SHARED_PTR_DEFINITIONS(CybergearActuator)

  // Lifecycle Callbacks
  CallbackReturn on_configure(const rclcpp_lifecycle::State&) override;
  CallbackReturn on_activate(const rclcpp_lifecycle::State&) override;
  CallbackReturn on_deactivate(const rclcpp_lifecycle::State&) override;
  CallbackReturn on_cleanup(const rclcpp_lifecycle::State&) override;
  CallbackReturn on_shutdown(const rclcpp_lifecycle::State&) override;
  CallbackReturn on_error(const rclcpp_lifecycle::State&) override;

  // Hardware Interface Callbacks
  CallbackReturn on_init(const hardware_interface::HardwareInfo&) override;

  std::vector<StateInterface> export_state_interfaces() override;
  std::vector<CommandInterface> export_command_interfaces() override;

  return_type prepare_command_mode_switch(
      const std::vector<std::string>&,
      const std::vector<std::string>&) override;
  return_type perform_command_mode_switch(
      const std::vector<std::string>&,
      const std::vector<std::string>&) override;

  return_type read(const rclcpp::Time&, const rclcpp::Duration&) override;
  return_type write(const rclcpp::Time&, const rclcpp::Duration&) override;

private:
  void receive();
  return_type send(const cybergear_driver_core::CanFrame&);

  return_type switchCommandInterface(uint8_t);
  void requestFeedback();

private:
  static constexpr uint8_t MOTOR_DISABLED = 255;

  static constexpr uint8_t SIF_POSITION = 0;
  static constexpr uint8_t SIF_VELOCITY = 1;
  static constexpr uint8_t SIF_TORQUE = 2;
  static constexpr uint8_t SIF_TEMPERATURE = 3;
  static constexpr uint8_t SIF_PROPORTIONAL_GAIN = 4;
  static constexpr uint8_t SIF_INTEGRAL_GAIN = 5;
  static constexpr uint8_t SIF_DERIVATIVE_GAIN = 6;

  static constexpr uint8_t HIF_POSITION = 0;
  static constexpr uint8_t HIF_VELOCITY = 1;
  static constexpr uint8_t HIF_EFFORT = 2;
  static constexpr uint8_t HIF_CURRENT = 3;

  static constexpr float TEMPERATURE_SCALE = 0.1;

  std::atomic_bool is_active_ = false;
  std::atomic_bool is_initialized_ = false;

  cybergear_driver_core::CybergearPacketParam params_;
  std::unique_ptr<cybergear_driver_core::CybergearPacket> packet_;

  uint8_t active_interface_;
  uint8_t command_mode_;

  bool state_interface_position_ = false;
  bool state_interface_velocity_ = false;
  bool state_interface_torque_ = false;

  std::vector<double> joint_commands_;
  std::vector<double> last_joint_commands_;
  std::vector<double> joint_states_;

  std::string can_filters_ = "0:0";
  std::string can_interface_;

  std::chrono::nanoseconds timeout_ns_;
  std::unique_ptr<drivers::socketcan::SocketCanSender> sender_;

  std::chrono::nanoseconds interval_ns_;
  std::unique_ptr<drivers::socketcan::SocketCanReceiver> receiver_;
  std::thread receiver_thread_;

  struct Feedback {
    cybergear_driver_core::CanData data;
    bool fault;
    bool error;
    rclcpp::Time stamp;
  };

  realtime_tools::RealtimeBuffer<Feedback> rtb_feedback_;
};

}  // namespace cybergear_control
