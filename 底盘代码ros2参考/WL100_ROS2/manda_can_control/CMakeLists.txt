cmake_minimum_required(VERSION 3.5)
project(manda_can_control)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/BatteryFb.msg"
  "msg/ControlModeCtrl.msg"
  "msg/FaultFb.msg"
  "msg/MotionCtrl.msg"
  "msg/MotionFb.msg"
  "msg/SpeedCtrl.msg"
  "msg/SpeedFb.msg"
  "msg/SteerCtrl.msg"
  "msg/SteerFb.msg"
  "msg/SystemstateFb.msg"
  "srv/DoCtrl.srv"
  "srv/MotionMode.srv"
)

add_executable(${PROJECT_NAME}_node src/manda_can_control/manda_can_control.cpp)
target_include_directories(${PROJECT_NAME}_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

ament_target_dependencies(${PROJECT_NAME}_node rclcpp)
rosidl_target_interfaces(${PROJECT_NAME}_node
  ${PROJECT_NAME} "rosidl_typesupport_cpp")

install(TARGETS ${PROJECT_NAME}_node
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY include/
  DESTINATION include)

ament_export_include_directories(include)
ament_export_dependencies(rosidl_default_runtime)
ament_export_dependencies(rclcpp)

ament_package()
