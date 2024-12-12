#include "cybergear_control/cybergear_hardware_interface.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <ctime>
#include <exception>
#include <memory>
#include <string>

#include "cybergear_driver_core/cybergear_packet.hpp"
#include "cybergear_driver_core/cybergear_packet_param.hpp"
#include "cybergear_driver_core/protocol_constant.hpp"
#include "hardware_interface/actuator_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/logging.hpp"
#include "rclcpp/macros.hpp"
#include "realtime_buffer.hpp"
#include "ros2_socketcan/socket_can_id.hpp"

using namespace std::chrono_literals;

namespace cybergear_control {
bool stob(std::string s) {
  auto result = false;  // failure to assert is false

  std::transform(s.begin(), s.end(), s.begin(), ::tolower);
  std::istringstream is(s);
  // first try simple integer conversion
  is >> result;

  if (is.fail()) {
    // simple integer failed; try boolean
    is.clear();
    is >> std::boolalpha >> result;
  }

  if (is.fail()) {
    throw std::invalid_argument(s.append(" is not convertable to bool"));
  }

  return result;
}

CallbackReturn CybergearActuator::on_configure(
    const rclcpp_lifecycle::State& /*previous_state*/) {
  can_interface_ = info_.hardware_parameters["can_interface"];

  double timeout_sec = std::stod(info_.hardware_parameters["timeout_sec"]);
  timeout_ns_ = std::chrono::duration_cast<std::chrono::nanoseconds>(
      std::chrono::duration<double>(timeout_sec));

  double interval_sec = std::stod(info_.hardware_parameters["interval_sec"]);
  interval_ns_ = std::chrono::duration_cast<std::chrono::nanoseconds>(
      std::chrono::duration<double>(interval_sec));

  RCLCPP_INFO(get_logger(), "interface: %s", can_interface_.c_str());
  RCLCPP_INFO(get_logger(), "timeout(s): %f", timeout_sec);
  RCLCPP_INFO(get_logger(), "interval(s): %f", interval_sec);

  params_.device_id = std::stoi(info_.hardware_parameters["device_id"]);
  params_.primary_id = std::stoi(info_.hardware_parameters["primary_id"]);
  RCLCPP_INFO(get_logger(), "device_id: %d", params_.device_id);
  RCLCPP_INFO(get_logger(), "primary_id: %d", params_.primary_id);

  // Using the default values
  params_.max_position = 4 * M_PI;
  params_.min_position = -4 * M_PI;
  params_.max_velocity = 30;
  params_.min_velocity = -30;
  params_.max_effort = 12;
  params_.min_effort = -12;
  params_.max_gain_kp = 500;
  params_.min_gain_kp = 0;
  params_.max_gain_kd = 5;
  params_.min_gain_kd = 0;
  params_.max_current = 23;
  params_.min_current = -23;
  params_.temperature_scale = 0.1;

  packet_ = std::make_unique<cybergear_driver_core::CybergearPacket>(params_);

  try {
    sender_ = std::make_unique<drivers::socketcan::SocketCanSender>(
        can_interface_, false);
  } catch (const std::exception& ex) {
    RCLCPP_ERROR(get_logger(), "Error opening CAN sender: %s - %s",
                 can_interface_.c_str(), ex.what());
    return CallbackReturn::FAILURE;
  }

  try {
    receiver_ = std::make_unique<drivers::socketcan::SocketCanReceiver>(
        can_interface_, false);
    // apply CAN filters
    receiver_->SetCanFilters(
        drivers::socketcan::SocketCanReceiver::CanFilterList(can_filters_));
    RCLCPP_DEBUG(get_logger(), "applied filters: %s", can_filters_.c_str());
  } catch (const std::exception& ex) {
    RCLCPP_ERROR(get_logger(), "Error opening CAN receiver: %s - %s",
                 can_interface_.c_str(), ex.what());
    return CallbackReturn::FAILURE;
  }
  receiver_thread_ = std::thread(&CybergearActuator::receive, this);

  RCLCPP_DEBUG(get_logger(), "Cybergear driver successfully configured.");
  return CallbackReturn::SUCCESS;
}

CallbackReturn CybergearActuator::on_activate(
    const rclcpp_lifecycle::State& /*previous_state*/) {
  // TODO: Send motor enable over CAN
  is_active_.store(true, std::memory_order_release);

  switchCommandInterface(active_interface_);

  RCLCPP_INFO(get_logger(), "Cybergear driver activated.");
  return CallbackReturn::SUCCESS;
}

CallbackReturn CybergearActuator::on_deactivate(
    const rclcpp_lifecycle::State& /*previous_state*/) {
  // TODO: Send motor disable over CAN
  is_active_.store(false, std::memory_order_release);

  // do not reset active_interface_, becase this will be reclaimed on_activate
  switchCommandInterface(MOTOR_DISABLED);

  RCLCPP_DEBUG(get_logger(), "Cybergear driver deactivate.");
  return CallbackReturn::SUCCESS;
}

CallbackReturn CybergearActuator::on_cleanup(
    const rclcpp_lifecycle::State& /*previous_state*/) {
  if (receiver_thread_.joinable()) {
    receiver_thread_.join();
  }

  RCLCPP_DEBUG(get_logger(), "Cybergear driver cleaned up.");
  return CallbackReturn::SUCCESS;
}

CallbackReturn CybergearActuator::on_shutdown(
    const rclcpp_lifecycle::State& /*previous_state*/) {
  RCLCPP_DEBUG(get_logger(), "Cybergear driver shutting down.");
  return CallbackReturn::SUCCESS;
}

CallbackReturn CybergearActuator::on_error(
    const rclcpp_lifecycle::State& /*previous_state*/) {
  RCLCPP_ERROR(get_logger(), "Cybergear driver error.");
  return CallbackReturn::FAILURE;
}

CallbackReturn CybergearActuator::on_init(
    const hardware_interface::HardwareInfo& info) {
  if (ActuatorInterface::on_init(info) != CallbackReturn::SUCCESS) {
    return CallbackReturn::ERROR;
  }

  // only one joint can be controlled
  if (info.joints.size() != 1) {
    RCLCPP_FATAL(get_logger(),
                 "Hardware interface has '%zu' joints. 1 expected.",
                 info.joints.size());
    return CallbackReturn::ERROR;
  }

  const hardware_interface::ComponentInfo& joint = info.joints[0];

  for (const auto& state_interface : joint.state_interfaces) {
    if (state_interface.name != hardware_interface::HW_IF_POSITION &&
        state_interface.name != hardware_interface::HW_IF_VELOCITY &&
        state_interface.name != hardware_interface::HW_IF_TORQUE &&
        state_interface.name != hardware_interface::HW_IF_TEMPERATURE) {
      RCLCPP_FATAL(get_logger(),
                   "Joint '%s' has incompatible state interface '%s'. Any "
                   "combination of '%s', '%s', '%s' and '%s' expected.",
                   joint.name.c_str(), state_interface.name.c_str(),
                   hardware_interface::HW_IF_POSITION,
                   hardware_interface::HW_IF_VELOCITY,
                   hardware_interface::HW_IF_TORQUE,
                   hardware_interface::HW_IF_TEMPERATURE);
      return CallbackReturn::ERROR;
    }
  }

  uint8_t mode_mask = 0;
  for (const auto& command_interface : joint.command_interfaces) {
    if (command_interface.name == hardware_interface::HW_IF_POSITION) {
      mode_mask |= 0x8;
    } else if (command_interface.name == hardware_interface::HW_IF_VELOCITY) {
      mode_mask |= 0x4;
    } else if (command_interface.name == hardware_interface::HW_IF_EFFORT) {
      mode_mask |= 0x2;
    } else if (command_interface.name == hardware_interface::HW_IF_CURRENT) {
      mode_mask |= 0x1;
    } else {
      RCLCPP_FATAL(get_logger(),
                   "Joint '%s' tries to claim incompatible combination of "
                   "command interfaces.",
                   joint.name.c_str());
      return hardware_interface::CallbackReturn::ERROR;
    }
  }
  switch (mode_mask) {
    case 0xE:
      active_interface_ = cybergear_driver_core::run_modes::OPERATION;
      break;
    case 0x1:
      active_interface_ = cybergear_driver_core::run_modes::CURRENT;
      break;
    case 0x4:
      active_interface_ = cybergear_driver_core::run_modes::SPEED;
      break;
    case 0x8:
      active_interface_ = cybergear_driver_core::run_modes::POSITION;
      break;
    case 0:
      active_interface_ = MOTOR_DISABLED;
      break;
    default:
      RCLCPP_FATAL(
          get_logger(),
          "Joint '%s' can not be controlled with this command interfaces.",
          joint.name.c_str());
      return CallbackReturn::ERROR;
  }
  command_mode_ = MOTOR_DISABLED;

  RCLCPP_INFO(get_logger(), "Using '%s' in mode '%d'", joint.name.c_str(),
              active_interface_);

  joint_states_.assign(4, std::numeric_limits<double>::quiet_NaN());
  joint_commands_.assign(4, std::numeric_limits<double>::quiet_NaN());
  last_joint_commands_ = joint_commands_;

  rtb_feedback_ = realtime_tools::RealtimeBuffer<Feedback>(
      {cybergear_driver_core::CanData(), false, false, get_clock()->now()});

  is_initialized_.store(true, std::memory_order_release);
  return CallbackReturn::SUCCESS;
}

std::vector<StateInterface> CybergearActuator::export_state_interfaces() {
  std::vector<StateInterface> state_interfaces;

  state_interfaces.emplace_back(hardware_interface::StateInterface(
      info_.joints[0].name, hardware_interface::HW_IF_POSITION,
      &joint_states_[SIF_POSITION]));
  state_interfaces.emplace_back(hardware_interface::StateInterface(
      info_.joints[0].name, hardware_interface::HW_IF_VELOCITY,
      &joint_states_[SIF_VELOCITY]));
  state_interfaces.emplace_back(hardware_interface::StateInterface(
      info_.joints[0].name, hardware_interface::HW_IF_TORQUE,
      &joint_states_[SIF_TORQUE]));
  state_interfaces.emplace_back(hardware_interface::StateInterface(
      info_.joints[0].name, hardware_interface::HW_IF_TEMPERATURE,
      &joint_states_[SIF_TEMPERATURE]));

  return state_interfaces;
}

std::vector<CommandInterface> CybergearActuator::export_command_interfaces() {
  std::vector<CommandInterface> command_interfaces;

  command_interfaces.emplace_back(hardware_interface::CommandInterface(
      info_.joints[0].name, hardware_interface::HW_IF_POSITION,
      &joint_commands_[HIF_POSITION]));
  command_interfaces.emplace_back(hardware_interface::CommandInterface(
      info_.joints[0].name, hardware_interface::HW_IF_VELOCITY,
      &joint_commands_[HIF_VELOCITY]));
  command_interfaces.emplace_back(hardware_interface::CommandInterface(
      info_.joints[0].name, hardware_interface::HW_IF_EFFORT,
      &joint_commands_[HIF_EFFORT]));
  command_interfaces.emplace_back(hardware_interface::CommandInterface(
      info_.joints[0].name, hardware_interface::HW_IF_CURRENT,
      &joint_commands_[HIF_CURRENT]));

  return command_interfaces;
}

hardware_interface::return_type CybergearActuator::prepare_command_mode_switch(
    const std::vector<std::string>& start_interfaces,
    const std::vector<std::string>& stop_interfaces) {
  uint8_t mode_mask = 0;
  switch (command_mode_) {
    case cybergear_driver_core::run_modes::OPERATION:
      mode_mask = 0xE;
      break;
    case cybergear_driver_core::run_modes::CURRENT:
      mode_mask = 0x1;
      break;
    case cybergear_driver_core::run_modes::SPEED:
      mode_mask = 0x4;
      break;
    case cybergear_driver_core::run_modes::POSITION:
      mode_mask = 0x8;
      break;
  }

  const std::string position_key =
      info_.joints[0].name + "/" + hardware_interface::HW_IF_POSITION;
  const std::string velocity_key =
      info_.joints[0].name + "/" + hardware_interface::HW_IF_VELOCITY;
  const std::string effort_key =
      info_.joints[0].name + "/" + hardware_interface::HW_IF_EFFORT;
  const std::string current_key =
      info_.joints[0].name + "/" + hardware_interface::HW_IF_CURRENT;

  for (const std::string& key : start_interfaces) {
    if (key == position_key) {
      mode_mask |= 0x8;
    } else if (key == velocity_key) {
      mode_mask |= 0x4;
    } else if (key == effort_key) {
      mode_mask |= 0x2;
    } else if (key == current_key) {
      mode_mask |= 0x1;
    }
  }

  for (const std::string& key : stop_interfaces) {
    if (key == position_key) {
      mode_mask &= ~0x8;
    } else if (key == velocity_key) {
      mode_mask &= ~0x4;
    } else if (key == effort_key) {
      mode_mask &= ~0x2;
    } else if (key == current_key) {
      mode_mask &= ~0x1;
    }
  }

  switch (mode_mask) {
    case 0xE:
    case 0x1:
    case 0x4:
    case 0x8:
    case 0x0:
      return hardware_interface::return_type::OK;
    default:
      return hardware_interface::return_type::ERROR;
  }
}

hardware_interface::return_type CybergearActuator::perform_command_mode_switch(
    const std::vector<std::string>& start_interfaces,
    const std::vector<std::string>& stop_interfaces) {
  uint8_t mode_mask = 0;
  switch (command_mode_) {
    case cybergear_driver_core::run_modes::OPERATION:
      mode_mask = 0xE;
      break;
    case cybergear_driver_core::run_modes::CURRENT:
      mode_mask = 0x1;
      break;
    case cybergear_driver_core::run_modes::SPEED:
      mode_mask = 0x4;
      break;
    case cybergear_driver_core::run_modes::POSITION:
      mode_mask = 0x8;
      break;
  }

  const std::string position_key =
      info_.joints[0].name + "/" + hardware_interface::HW_IF_POSITION;
  const std::string velocity_key =
      info_.joints[0].name + "/" + hardware_interface::HW_IF_VELOCITY;
  const std::string effort_key =
      info_.joints[0].name + "/" + hardware_interface::HW_IF_EFFORT;
  const std::string current_key =
      info_.joints[0].name + "/" + hardware_interface::HW_IF_CURRENT;

  for (const std::string& key : start_interfaces) {
    if (key == position_key) {
      mode_mask |= 0x8;
    } else if (key == velocity_key) {
      mode_mask |= 0x4;
    } else if (key == effort_key) {
      mode_mask |= 0x2;
    } else if (key == current_key) {
      mode_mask |= 0x1;
    }
  }

  for (const std::string& key : stop_interfaces) {
    if (key == position_key) {
      mode_mask &= ~0x8;
    } else if (key == velocity_key) {
      mode_mask &= ~0x4;
    } else if (key == effort_key) {
      mode_mask &= ~0x2;
    } else if (key == current_key) {
      mode_mask &= ~0x1;
    }
  }

  switch (mode_mask) {
    case 0xE:
      switchCommandInterface(cybergear_driver_core::run_modes::OPERATION);
      return hardware_interface::return_type::OK;
    case 0x1:
      switchCommandInterface(cybergear_driver_core::run_modes::CURRENT);
      return hardware_interface::return_type::OK;
    case 0x4:
      switchCommandInterface(cybergear_driver_core::run_modes::SPEED);
      return hardware_interface::return_type::OK;
    case 0x8:
      switchCommandInterface(cybergear_driver_core::run_modes::POSITION);
      return hardware_interface::return_type::OK;
    case 0:
      switchCommandInterface(MOTOR_DISABLED);
      return hardware_interface::return_type::OK;
    default:
      return hardware_interface::return_type::ERROR;
  }
}

return_type CybergearActuator::read(const rclcpp::Time& /*time*/,
                                    const rclcpp::Duration& /*period*/) {
  auto feedback = rtb_feedback_.readFromRT();
  joint_states_[SIF_POSITION] = packet_->parsePosition(feedback->data);
  joint_states_[SIF_VELOCITY] = packet_->parseVelocity(feedback->data);
  joint_states_[SIF_TORQUE] = packet_->parseEffort(feedback->data);
  joint_states_[SIF_TEMPERATURE] = packet_->parseTemperature(feedback->data);
  // RCLCPP_INFO(get_logger(),
  //             "State %x %x %x %x %x %x %x %x, Position: %f, Speed: %f, "
  //             "Current: %f, Temp: %f",
  //             feedback->data[7], feedback->data[6], feedback->data[5],
  //             feedback->data[4], feedback->data[3], feedback->data[2],
  //             feedback->data[1], feedback->data[0], joint_states_[0],
  //             joint_states_[1], joint_states_[2], joint_states_[3]);

  if (feedback->fault || feedback->error) {
    return return_type::ERROR;
  }

  // TODO: new param to define timeout time
  const auto duration = get_clock()->now() - feedback->stamp;
  // RCLCPP_INFO(get_logger(), "Last feedback %f seconds old",
  // duration.seconds());

  requestFeedback();
  return return_type::OK;
}

return_type CybergearActuator::write(const rclcpp::Time& /*time*/,
                                     const rclcpp::Duration& /*period*/) {
  switch (command_mode_) {
    case MOTOR_DISABLED:
      return return_type::OK;
    case cybergear_driver_core::run_modes::OPERATION:
      if (std::isnan(joint_commands_[HIF_POSITION]) ||
          std::isnan(joint_commands_[HIF_VELOCITY]) ||
          std::isnan(joint_commands_[HIF_EFFORT])) {
        return return_type::OK;
      }
      break;
    case cybergear_driver_core::run_modes::CURRENT:
      if (std::isnan(joint_commands_[HIF_CURRENT])) {
        return return_type::OK;
      }
      break;
    case cybergear_driver_core::run_modes::SPEED:
      if (std::isnan(joint_commands_[HIF_VELOCITY])) {
        return return_type::OK;
      }
      break;
    case cybergear_driver_core::run_modes::POSITION:
      if (std::isnan(joint_commands_[HIF_POSITION])) {
        return return_type::OK;
      }
      break;
  }

  // the cybergear motor has its own motor controller which doesn't need to be
  // updated with the same value again.
  if (last_joint_commands_[command_mode_] == joint_commands_[command_mode_])
    return return_type::OK;

  cybergear_driver_core::CanFrame frame;
  cybergear_driver_core::MoveParam param;
  switch (command_mode_) {
    case cybergear_driver_core::run_modes::OPERATION:
      param.position = joint_commands_[HIF_POSITION];
      param.velocity = joint_commands_[HIF_VELOCITY];
      param.effort = joint_commands_[HIF_EFFORT];
      // TODO: Use params for this?
      param.kp = 0;
      param.kd = 0;
      frame = packet_->createMoveCommand(param);
      break;
    case cybergear_driver_core::run_modes::CURRENT:
      frame = packet_->createCurrentCommand(joint_commands_[HIF_CURRENT]);
      break;
    case cybergear_driver_core::run_modes::SPEED:
      frame = packet_->createVelocityCommand(joint_commands_[HIF_VELOCITY]);
      break;
    case cybergear_driver_core::run_modes::POSITION:
      frame = packet_->createPositionCommand(joint_commands_[HIF_POSITION]);
      break;
  }

  return send(frame);
}

void CybergearActuator::receive() {
  while (!is_initialized_.load(std::memory_order_acquire)) {
    std::this_thread::yield();
  }

  drivers::socketcan::CanId can_id;
  const auto frame_id = packet_->frameId();
  const auto clock = get_clock();

  Feedback feedback = *rtb_feedback_.readFromNonRT();

  while (rclcpp::ok()) {
    while (!is_active_.load(std::memory_order_acquire) && rclcpp::ok()) {
      std::this_thread::yield();
    }

    try {
      can_id = receiver_->receive(feedback.data.data(), interval_ns_);
    } catch (const std::exception& ex) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000,
                           "Error receiving CAN message: %s - %s",
                           can_interface_.c_str(), ex.what());
      continue;
    }
    uint32_t id = can_id.identifier();
    if (!frame_id.isDevice(id)) {
      continue;
    }

    feedback.stamp = clock->now();

    // Detect fault state
    if (frame_id.isFault(id)) {
      RCLCPP_ERROR(get_logger(), "Detect fault state from cybergear");
      feedback.fault = true;
      rtb_feedback_.writeFromNonRT(feedback);
    }

    if (frame_id.isFeedback(id)) {
      if (frame_id.hasError(id)) {
        RCLCPP_ERROR(get_logger(), "Detect fault state from cybergear");
        feedback.error = true;
      }

      rtb_feedback_.writeFromNonRT(feedback);
    }
  }
}

return_type CybergearActuator::send(
    const cybergear_driver_core::CanFrame& msg) {
  using drivers::socketcan::CanId;
  using drivers::socketcan::ExtendedFrame;
  using drivers::socketcan::FrameType;

  CanId send_id(msg.id, 0, FrameType::DATA, ExtendedFrame);
  try {
    sender_->send(msg.data.data(), msg.data.size(), send_id, timeout_ns_);
  } catch (const std::exception& ex) {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000,
                         "Error sending CAN message: %s - %s",
                         can_interface_.c_str(), ex.what());
    return return_type::ERROR;
  }

  return return_type::OK;
}

// TODO: Evaluate if successful
return_type CybergearActuator::switchCommandInterface(
    uint8_t new_command_mode) {
  RCLCPP_INFO(get_logger(), "Send reset torque id");
  cybergear_driver_core::CanFrame frame;
  frame.id = packet_->frameId().getResetTorqueId();
  send(frame);

  RCLCPP_INFO(get_logger(), "Send change run mode from %d to %d", command_mode_,
              new_command_mode);
  switch (new_command_mode) {
    case cybergear_driver_core::run_modes::OPERATION:
      frame = packet_->createChangeToOperationModeCommand();
      break;
    case cybergear_driver_core::run_modes::POSITION:
      frame = packet_->createChangeToPositionModeCommand();
      break;
    case cybergear_driver_core::run_modes::SPEED:
      frame = packet_->createChangeToVelocityModeCommand();
      break;
    case cybergear_driver_core::run_modes::CURRENT:
      frame = packet_->createChangeToCurrentModeCommand();
      break;
  }
  send(frame);

  if (new_command_mode != MOTOR_DISABLED) {
    RCLCPP_INFO(get_logger(), "Send enable torque");
    frame.id = packet_->frameId().getEnableTorqueId();
    send(frame);
  }

  command_mode_ = new_command_mode;
  return return_type::OK;
}

void CybergearActuator::requestFeedback() {
  cybergear_driver_core::CanFrame frame;
  frame.id = packet_->frameId().getFeedbackId();
  send(frame);
  frame.id = packet_->frameId().getReadParameterId();
}

}  // namespace cybergear_control

#include "pluginlib/class_list_macros.hpp"  // NOLINT
PLUGINLIB_EXPORT_CLASS(cybergear_control::CybergearActuator,
                       hardware_interface::ActuatorInterface)
