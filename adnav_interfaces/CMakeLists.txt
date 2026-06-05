cmake_minimum_required(VERSION 3.8)
project(adnav_interfaces)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)

# Message and service definitions live in the an-ros-common submodule (shared
# with the ROS 1 driver). List them explicitly rather than globbing so a
# configure run isn't required when files are added/removed in the submodule.
set(_AN_ROS_COMMON ../an-ros-common)
set(msg_files
  ${_AN_ROS_COMMON}/msg/ANPPHeader.msg
  ${_AN_ROS_COMMON}/msg/DeviceInformationPacket.msg
  ${_AN_ROS_COMMON}/msg/FilterStatus.msg
  ${_AN_ROS_COMMON}/msg/GNSSFixStatus.msg
  ${_AN_ROS_COMMON}/msg/LLH.msg
  ${_AN_ROS_COMMON}/msg/NED.msg
  ${_AN_ROS_COMMON}/msg/PacketPeriod.msg
  ${_AN_ROS_COMMON}/msg/RPH.msg
  ${_AN_ROS_COMMON}/msg/RawAcknowledge.msg
  ${_AN_ROS_COMMON}/msg/RawStatusPacket.msg
  ${_AN_ROS_COMMON}/msg/RequestPacket.msg
  ${_AN_ROS_COMMON}/msg/SerialInterface.msg
  ${_AN_ROS_COMMON}/msg/SystemStatus.msg
)
set(srv_files
  ${_AN_ROS_COMMON}/srv/Ntrip.srv
  ${_AN_ROS_COMMON}/srv/PacketPeriods.srv
  ${_AN_ROS_COMMON}/srv/PacketTimerPeriod.srv
  ${_AN_ROS_COMMON}/srv/RequestPackets.srv
)

rosidl_generate_interfaces(${PROJECT_NAME}
  ${msg_files}
  ${srv_files}
  DEPENDENCIES std_msgs geometry_msgs builtin_interfaces
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_dependencies(rosidl_default_runtime)

ament_package()