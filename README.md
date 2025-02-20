# cybergear_ros2

This repository is for controlling CyberGear servo motors with ROS 2.
Currently, it includes the following packages:

+ [cybergear_driver_core](https://github.com/NaokiTakahashi12/cybergear_ros2/tree/main/cybergear_driver_core)

  This package provides manipulation of CyberGear's CAN frames independently of ROS 2.

+ [cybergear_driver_msgs](https://github.com/NaokiTakahashi12/cybergear_ros2/tree/main/cybergear_driver_msgs)

  This package provides messages used by the cybergear_socketcan_driver.

+ [cybergear_socketcan_driver](https://github.com/NaokiTakahashi12/cybergear_ros2/tree/main/cybergear_socketcan_driver)

  This package provides functionality to operate CyberGear using CAN frame messages.

+ [cybergear_maintenance_tools]

  To use this package, you have to use ros2 run and pass the parameters

  '''ros2 run cybergear_maintenance_tools change_cybergear_id_node --ros-args -p target_id:=126 -p device_id:=127'''

  In this example you change the ID of cybergear motor with id 127 to 126.
  Also remember to start the ros2_socketcan bridge and run ./can.sh with sudo!