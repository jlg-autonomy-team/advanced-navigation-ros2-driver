#ifndef ADNAV_DRIVER_PACKET_CONTRACT_HPP_
#define ADNAV_DRIVER_PACKET_CONTRACT_HPP_

#include <cstdint>
#include <vector>

#include <ins_packets.h>

namespace adnav {

constexpr uint32_t kPacket20Update = 1u << 0;
constexpr uint32_t kPacket23Update = 1u << 1;
constexpr uint32_t kPacket24Update = 1u << 2;
constexpr uint32_t kPacket25Update = 1u << 3;
constexpr uint32_t kPacket26Update = 1u << 4;
constexpr uint32_t kPacket27Update = 1u << 5;
constexpr uint32_t kPacket28Update = 1u << 6;
constexpr uint32_t kPacket33Update = 1u << 7;
constexpr uint32_t kPacket35Update = 1u << 8;
constexpr uint32_t kPacket36Update = 1u << 9;
constexpr uint32_t kPacket38Update = 1u << 10;
constexpr uint32_t kPacket39Update = 1u << 11;
constexpr uint32_t kPacket40Update = 1u << 12;
constexpr uint32_t kPacket42Update = 1u << 13;
constexpr uint32_t kPacket43Update = 1u << 14;

// Packet 39 is the live orientation source used before heading initialization.
inline bool isRosDriverPacketId(int64_t packet_id) {
  switch (packet_id) {
  case packet_id_device_information:
  case packet_id_acknowledge:
  case packet_id_system_state:
  case packet_id_quaternion_orientation_standard_deviation:
  case packet_id_euler_orientation_standard_deviation:
  case packet_id_raw_sensors:
  case packet_id_status:
  case packet_id_position_standard_deviation:
  case packet_id_velocity_standard_deviation:
  case packet_id_ecef_position:
  case packet_id_ned_velocity:
  case packet_id_body_velocity:
  case packet_id_body_acceleration:
  case packet_id_euler_orientation:
  case packet_id_quaternion_orientation:
  case packet_id_angular_velocity:
  case packet_id_angular_acceleration:
    return true;
  default:
    return false;
  }
}

inline uint32_t rosDriverPendingUpdateForPacket(int64_t packet_id) {
  switch (packet_id) {
  case packet_id_system_state:
    return kPacket20Update;
  case packet_id_status:
    return kPacket23Update;
  case packet_id_position_standard_deviation:
    return kPacket24Update;
  case packet_id_velocity_standard_deviation:
    return kPacket25Update;
  case packet_id_euler_orientation_standard_deviation:
    return kPacket26Update;
  case packet_id_quaternion_orientation_standard_deviation:
    return kPacket27Update;
  case packet_id_raw_sensors:
    return kPacket28Update;
  case packet_id_ecef_position:
    return kPacket33Update;
  case packet_id_ned_velocity:
    return kPacket35Update;
  case packet_id_body_velocity:
    return kPacket36Update;
  case packet_id_body_acceleration:
    return kPacket38Update;
  case packet_id_euler_orientation:
    return kPacket39Update;
  case packet_id_quaternion_orientation:
    return kPacket40Update;
  case packet_id_angular_velocity:
    return kPacket42Update;
  case packet_id_angular_acceleration:
    return kPacket43Update;
  default:
    return 0;
  }
}

inline bool isRosDriverPacketRequest(const std::vector<int64_t> &request,
                                     int64_t minimum_period,
                                     int64_t maximum_period) {
  if (request.empty() || request.size() % 2 != 0) {
    return false;
  }
  for (size_t index = 0; index < request.size(); index += 2) {
    if (!isRosDriverPacketId(request[index]) ||
        request[index + 1] < minimum_period ||
        request[index + 1] > maximum_period) {
      return false;
    }
  }
  return true;
}

inline int64_t packet20StampNanoseconds(uint32_t unix_time_seconds,
                                        uint32_t microseconds,
                                        bool utc_time_initialised,
                                        int64_t receipt_time_nanoseconds) {
  if (utc_time_initialised && unix_time_seconds != 0 &&
      microseconds <= 999999) {
    return static_cast<int64_t>(unix_time_seconds) * 1000000000LL +
           static_cast<int64_t>(microseconds) * 1000LL;
  }
  return receipt_time_nanoseconds;
}

inline int64_t monotonicStampNanoseconds(int64_t candidate, int64_t previous) {
  if (candidate > 0 && previous > 0 && candidate <= previous) {
    return previous + 1;
  }
  return candidate;
}

} // namespace adnav

#endif // ADNAV_DRIVER_PACKET_CONTRACT_HPP_
