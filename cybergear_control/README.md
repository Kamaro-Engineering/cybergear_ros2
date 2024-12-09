# HW Interfaces and operating modes
List of hardware interfaces:
- Position
- Velocity
- Effort
- Current

List of operating modes accoring to the [translated manual](https://github.com/belovictor/cybergear-docs/blob/main/instructionmanual/instructionmanual.md):
- Operation Control Mode
- Current Mode
- Speed Mode
- Location Mode

Table of claimable hardware interfaces and the resulting operating mode:
| | Position | Velocity | Effort | Current |
|--:|:--:|:--:|:--:|:--:|
| Operation Control Mode | ✔ | ✔ | ✔ | ✘ |
| Current Mode | ✘ | ✘ | ✘ | ✔ |
| Speed Mode | ✘ | ✔ | ✘ | ✘ |
| Location Mode | ✔ | ✘ | ✘ | ✘ |

All other combinations of hardware interface claims are invalid.

# Links
- [Cybergear CAN Protocol](https://github.com/belovictor/cybergear-docs/blob/main/instructionmanual/instructionmanual.md)
- [ROS2 Control - Write Hardware Interface](http://docs.ros.org/en/rolling/p/hardware_interface/user_docs/writing_new_hardware_component.html)
- [ROS2 Lifecycle Node](https://docs.ros.org/en/ros2_packages/rolling/api/lifecycle/)
