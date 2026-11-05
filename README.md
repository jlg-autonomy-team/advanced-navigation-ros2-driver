# Advanced Navigation ROS 2 driver

**Status: Implemented.** This submodule contains the `adnav_driver` device node, its `adnav_interfaces` message/service package, and `adnav_launch` launch/configuration package. It is a ROS 2 driver for Advanced Navigation inertial navigation system (INS) devices speaking Advanced Navigation Packet Protocol (ANPP), including the Certus Mini-D. ANPP packet IDs identify device payloads; the implemented raw/decoded stream includes packets 20, 23–28, 33, 35–43 and the acknowledgement/device-information packets needed for setup.

## Purpose and scope

The driver opens the configured serial, TCP client/server, or UDP transport; exchanges and decodes ANPP packets; publishes ROS messages; accepts services for packet configuration, one-shot packet requests, NTRIP and installation alignment; and writes ANPP/RTCM session logs. NTRIP means Networked Transport of RTCM via Internet Protocol; RTCM is the correction-data format. The driver passes correction bytes to the device using ANPP packet 55.

**Implemented capabilities**

- Serial, TCP client, TCP server and UDP transport selection through supplied launch profiles.
- ROS decoding/publication for supported ANPP packet IDs; unsupported IDs are not translated into ROS messages.
- Device packet period/timer configuration and one-time packet requests.
- Built-in NTRIP client control and installation-alignment service.
- ANPP and RTCM logging, with bounded device-information handshake timeout.

**Non-goals / boundaries**

- This is not a general decoder for every ANPP packet and does not publish arbitrary packet payloads as ROS topics.
- `atlas_ins_odom_bridge` owns the derived, covariance-aware odometry interface; the driver publishes sensor/device data and does not provide fused vehicle state.
- `atlas_ntrip_client` is an optional operational wrapper around this driver's NTRIP service; it does not replace the device connection or implement another NTRIP transport.
- `an-ros-common` is a vendored Advanced Navigation SDK submodule and is not maintained by this README's code changes.

## Data flow

The device emits ANPP packets over the selected transport. `adnav_driver` decodes supported packet IDs and publishes ROS messages; service requests and NTRIP corrections travel in the opposite direction. Topics use queue depth 10 (`rclcpp` default reliable/volatile QoS profile), unless noted by the ROS service profile.

## Public ROS API

The launch profile names the node `adnav_node` (the C++ node's own default name is `adnav_driver`). The topic table lists the fully-qualified names with the supplied launch profile.

| Interface | Type | QoS / details |
|---|---|---|
| `/adnav_node/imu` | `sensor_msgs/msg/Imu` | Keep-last depth 10; device orientation/angular-rate values where decoded; covariance `-1` marks unavailable estimates. |
| `/adnav_node/imu_raw` | `sensor_msgs/msg/Imu` | Keep-last depth 10; raw-sensor packet values. |
| `/adnav_node/nav_sat_fix` | `sensor_msgs/msg/NavSatFix` | Keep-last depth 10; packet 20 navigation solution. |
| `/adnav_node/magnetic_field` | `sensor_msgs/msg/MagneticField` | Keep-last depth 10; raw-sensors packet. |
| `/adnav_node/barometric_pressure` | `sensor_msgs/msg/FluidPressure` | Keep-last depth 10. |
| `/adnav_node/temperature` | `sensor_msgs/msg/Temperature` | Keep-last depth 10. |
| `/adnav_node/twist` | `geometry_msgs/msg/Twist` | Keep-last depth 10; packet-20 linear velocity. |
| `/adnav_node/pose` | `geometry_msgs/msg/Pose` | Keep-last depth 10. |
| `/adnav_node/system_status` | `adnav_interfaces/msg/SystemStatus` | Keep-last depth 10. |
| `/filter_status` | `adnav_interfaces/msg/FilterStatus` | Keep-last depth 10. |
| `/status` | `adnav_interfaces/msg/StatusPacket` | Keep-last depth 10. |
| `/adnav_node/position_std_dev` | `adnav_interfaces/msg/PositionStdDev` | Keep-last depth 10; packet 24. |
| `/adnav_node/velocity_std_dev` | `adnav_interfaces/msg/VelocityStdDev` | Keep-last depth 10; packet 25. |
| `/adnav_node/ned_velocity` | `adnav_interfaces/msg/NedVelocity` | Keep-last depth 10; packet 35. NED = north-east-down. |
| `/adnav_node/quaternion_std_dev` | `adnav_interfaces/msg/QuaternionStdDev` | Keep-last depth 10; packet 27. |
| `/adnav_node/euler_std_dev` | `adnav_interfaces/msg/EulerStdDev` | Keep-last depth 10; packet 26. |
| `/adnav_node/body_velocity` | `adnav_interfaces/msg/BodyVelocity` | Keep-last depth 10; packet 36. |
| `/adnav_node/body_acceleration` | `adnav_interfaces/msg/BodyAcceleration` | Keep-last depth 10; packet 38. |
| `/adnav_node/quaternion_orientation` | `adnav_interfaces/msg/QuaternionOrientation` | Keep-last depth 10; packet 40. |
| `/adnav_node/euler_orientation` | `adnav_interfaces/msg/EulerOrientation` | Keep-last depth 10; packet 39. |
| `/adnav_node/angular_velocity` | `adnav_interfaces/msg/AngularVelocity` | Keep-last depth 10; packet 42. |
| `/adnav_node/angular_acceleration` | `adnav_interfaces/msg/AngularAcceleration` | Keep-last depth 10; packet 43. |
| `/adnav_node/packet_periods` | `adnav_interfaces/srv/PacketPeriods` | Service QoS: `rmw_qos_profile_services_default`. |
| `/adnav_node/packet_timer_period` | `adnav_interfaces/srv/PacketTimerPeriod` | Service QoS: default service profile. |
| `/adnav_node/request_packet` | `adnav_interfaces/srv/RequestPackets` | Service QoS: default service profile. |
| `/adnav_node/ntrip` | `adnav_interfaces/srv/Ntrip` | Service QoS: default service profile. Credentials are request fields, not launch YAML defaults. |
| `/adnav_node/installation_alignment` | `adnav_interfaces/srv/InstallationAlignment` | Service QoS: default service profile. |

The node also provides standard ROS 2 parameter services. All public parameters are read-only after startup except `read_us`, `publish_us`, `packet_request`, and `packet_timer_period`, which have update callbacks. Defaults below come from the node declaration; the checked-in launch YAML overrides several.

| Parameter | Default | Meaning |
|---|---:|---|
| `baud_rate` | `115200` | Serial baud rate. |
| `com_port` | `ttyUSB0` | Serial device; the communicator resolves the device path. |
| `read_us` | `20000` | Packet polling timer in microseconds. |
| `publish_us` | `20000` | ROS publication timer in microseconds; must be at least `read_us`. |
| `packet_request` | `[20,10,23,10,24,10,25,10,26,10,28,10,39,10,42,10]` | Alternating ANPP packet ID and period values. |
| `packet_timer_period` | `10000` | Device packet timer in microseconds (valid range 1000–65535). |
| `ip_address` | `0.0.0.0` | IP endpoint address. |
| `port` | `0` | IP endpoint port. |
| `log_path` | `~/.ros/log/` | Path used for driver log files. |
| `comm_select` | `0` | Transport: 0 serial, 1 TCP client, 2 TCP server, 3 UDP, 4 CAN (not supported by this build). |

The supplied serial YAML sets `publish_us=20000`, `read_us=1000`, `comm_select=0`, `baud_rate=1000000`, `com_port=ttyUSB0`, and packet timer 5000 µs, with its own packet request list. Check the selected YAML before relying on parameter defaults.

## Prerequisites

- ROS 2 Jazzy on Ubuntu 24.04 and a C++ build toolchain (`colcon`, CMake, and the ROS 2 development packages).
- `adnav_driver/package.xml` dependencies: `rclcpp`, `std_msgs`, `sensor_msgs`, `geometry_msgs`, `tf2`, `std_srvs`, `adnav_interfaces`; testing uses `ament_lint_common`.
- Build the sibling `adnav_interfaces` package as part of this submodule. Its generated interfaces use the message/service definitions in the vendored `an-ros-common` checkout.
- Device access and a compatible serial/network connection for hardware operation.

## Build and run

From the workspace root, after sourcing the ROS 2 Jazzy underlay:

```bash
source /opt/ros/jazzy/setup.bash
colcon build --packages-select adnav_interfaces adnav_driver adnav_launch
source install/setup.bash
ros2 launch adnav_launch adnav_serial.launch.py
```

Other exact launch files are `adnav_tcp_client.launch.py`, `adnav_tcp_server.launch.py`, and `adnav_udp_client.launch.py`. Use the corresponding YAML in `adnav_launch/config/`; verify the remote endpoint and transport settings before starting. Direct executable invocation is `ros2 run adnav_driver adnav_driver` with suitable ROS parameters.

Inspect interfaces and live data from another sourced terminal:

```bash
ros2 node info /adnav_node
ros2 topic list
ros2 topic echo /adnav_node/imu
ros2 topic hz /adnav_node/nav_sat_fix
ros2 service list
ros2 interface show adnav_interfaces/srv/Ntrip
```

Run package tests from the workspace root:

```bash
colcon test --packages-select adnav_interfaces adnav_driver adnav_launch
colcon test-result --verbose
```

The driver test target covers supported packet IDs/request validation, source update mapping, and packet timestamps; it does not require physical hardware.

## Limitations

- Only packet IDs handled by the current decoder produce ROS output; `packet_request` validation rejects IDs without driver decoders.
- ROS publication and device packet periods are separate settings. A device may reject a configuration; check the driver's acknowledgement logs and observed topic rate.
- The SDK supports a CAN selection value in the parameter contract, but CAN is not implemented in this workspace build.
- IMU covariance is unavailable from source packets for some quantities and is explicitly marked unknown (`-1` diagonal).
- Device behavior, correction validity, and achievable update rate depend on the attached INS, transport, configuration, and correction caster.

## Further reading

- Optional workspace integration notes: [`../../docs/02-data-and-frame-contracts.md`](../../docs/02-data-and-frame-contracts.md) and [`../../docs/07-plan.md`](../../docs/07-plan.md).
