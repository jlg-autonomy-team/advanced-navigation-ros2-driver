cmake_minimum_required(VERSION 3.5)
set(CMAKE_BUILD_TYPE release)
project(adnav_driver)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra)
endif()

# Avoid Confusing max() and min() for Windows Macros. See rclcpp issue #1054
if(WIN32)
  add_definitions(-DNOMINMAX)
endif()

#find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(tf2 REQUIRED)
find_package(adnav_interfaces REQUIRED)

# Build
include_directories(
  include/adnav_driver
  ../an-ros-common/include
)

aux_source_directory(src src_MAIN)
aux_source_directory(../an-ros-common/src src_COMMON)

add_library(${PROJECT_NAME}_lib
  ${src_MAIN}
  ${src_COMMON}
)

set(DEPENDENCIES
  "rclcpp"
  "std_msgs"
  "std_srvs"
  "sensor_msgs"
  "geometry_msgs"
  "tf2"
  "adnav_interfaces"
)

set(EXECUTABLE_NAME "adnav_driver")

ament_target_dependencies(${PROJECT_NAME}_lib ${DEPENDENCIES})

add_executable(${EXECUTABLE_NAME} src/main.cpp)
target_link_libraries(${EXECUTABLE_NAME} ${PROJECT_NAME}_lib)
ament_target_dependencies(${EXECUTABLE_NAME} ${DEPENDENCIES})

# add_executable(srv_test src/srv_test.cpp)
# target_link_libraries(srv_test ${PROJECT_NAME}_lib)
# ament_target_dependencies(srv_test ${DEPENDENCIES})

install(TARGETS ${EXECUTABLE_NAME}
  DESTINATION lib/${PROJECT_NAME}
)

# install(TARGETS srv_test
#   DESTINATION lib/${PROJECT_NAME}
# )

install(
  DIRECTORY include/
  DESTINATION include
)

if(BUILD_TESTING)
  # The driver contains legacy formatting outside this change. Keep functional regression tests
  # enabled without making that historical lint debt a gate for packet/status changes.
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_packet_contract test/test_packet_contract.cpp)
  target_include_directories(test_packet_contract PRIVATE include ../an-ros-common/include)
endif()

ament_export_include_directories(include)
ament_package()
