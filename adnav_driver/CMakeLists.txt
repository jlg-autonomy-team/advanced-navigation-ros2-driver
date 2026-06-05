cmake_minimum_required(VERSION 3.10)
project(adnav_driver)

# Default to release builds; honour user-supplied CMAKE_BUILD_TYPE first.
if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
  set(CMAKE_BUILD_TYPE Release CACHE STRING "Build type" FORCE)
endif()

if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra)
endif()

# Avoid clashing with Windows max()/min() macros. See rclcpp issue #1054.
if(WIN32)
  add_definitions(-DNOMINMAX)
endif()

# --- Dependencies ----------------------------------------------------------
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(diagnostic_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(adnav_interfaces REQUIRED)

set(DEPENDENCIES
  rclcpp
  std_msgs
  std_srvs
  sensor_msgs
  diagnostic_msgs
  geometry_msgs
  tf2
  adnav_interfaces
)

# --- Sources ---------------------------------------------------------------
# We depend on `an-ros-common` (a sibling submodule) for the ANPP C library
# and shared helper classes. List the files explicitly rather than globbing
# so a configure run isn't required when files are added/removed in the
# submodule and so we get accurate dep-tracking.

set(ANROSCOMMON_DIR ${CMAKE_CURRENT_SOURCE_DIR}/../an-ros-common)

set(ANROSCOMMON_SRC
  ${ANROSCOMMON_DIR}/src/adnav_comms.cpp
  ${ANROSCOMMON_DIR}/src/adnav_ntrip.cpp
  ${ANROSCOMMON_DIR}/src/adnav_utils.cpp
  ${ANROSCOMMON_DIR}/src/an_packet_protocol.c
  ${ANROSCOMMON_DIR}/src/ins_packets.c
  ${ANROSCOMMON_DIR}/src/rs232.c
)

set(DRIVER_SRC
  src/adnav_driver.cpp
)

# --- Library ---------------------------------------------------------------
add_library(${PROJECT_NAME}_lib ${DRIVER_SRC} ${ANROSCOMMON_SRC})

target_include_directories(${PROJECT_NAME}_lib
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include/adnav_driver>
    $<BUILD_INTERFACE:${ANROSCOMMON_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(${PROJECT_NAME}_lib ${DEPENDENCIES})

# Explicit dep edge on adnav_interfaces' generated IDL targets so a parallel
# `colcon build` doesn't race the .hpp generation. (B3)
if(TARGET adnav_interfaces__rosidl_typesupport_cpp)
  add_dependencies(${PROJECT_NAME}_lib adnav_interfaces__rosidl_typesupport_cpp)
endif()

# --- Executable ------------------------------------------------------------
add_executable(${PROJECT_NAME} src/main.cpp)
target_link_libraries(${PROJECT_NAME} ${PROJECT_NAME}_lib)
ament_target_dependencies(${PROJECT_NAME} ${DEPENDENCIES})

# --- Install ---------------------------------------------------------------
install(TARGETS ${PROJECT_NAME}
  DESTINATION lib/${PROJECT_NAME}
)
install(
  DIRECTORY include/
  DESTINATION include
)

if(BUILD_TESTING)
  find_package(ament_cmake_cpplint REQUIRED)
  ament_cpplint(
    FILTERS
      "-build/include_subdir"    # headers are included by filename, not subdir path
      "-build/include_order"
      "-whitespace/newline"
      "-whitespace/blank_line"   # redundant blank lines at end of blocks
      "-whitespace/ending_newline" # missing newline at end of file
      "-whitespace/tab"          # existing code uses tabs in some files
      "-whitespace/line_length"  # line length is fine
      "-whitespace/comments"
      "-whitespace/operators"
      "-readability/braces"
      "-build/header_guard"

  )
endif()

ament_export_include_directories(include)
ament_package()
