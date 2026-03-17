cmake_minimum_required(VERSION 3.14)
project(cybergear_multi_driver)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -O2)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(cybergear_driver_core REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# Generate service interfaces (no additional dependencies needed for simple types)
rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/SetMotorGain.srv"
  DEPENDENCIES
)

rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")

# Main driver node – link order matters: typesupport first, then interface libs
add_executable(cybergear_multi_driver_node
  src/cybergear_multi_driver_node.cpp
)

target_include_directories(cybergear_multi_driver_node
  PRIVATE
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
)

target_link_libraries(cybergear_multi_driver_node
  ${cpp_typesupport_target}
  cybergear_driver_core   # INTERFACE IMPORTED: brings in include dirs transitively
  ${sensor_msgs_TARGETS}
  ${std_msgs_TARGETS}
  ${std_srvs_TARGETS}
  rclcpp::rclcpp
  rclcpp_components::component
  rclcpp_components::component_manager
  sensor_msgs::sensor_msgs_library
)

install(TARGETS cybergear_multi_driver_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  config
  launch
  DESTINATION share/${PROJECT_NAME}
  OPTIONAL
)

ament_package()
