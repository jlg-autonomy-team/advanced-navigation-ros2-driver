#ifndef ADNAV_DRIVER_ORIENTATION_CONVERSION_HPP_
#define ADNAV_DRIVER_ORIENTATION_CONVERSION_HPP_

#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Vector3.h>

namespace adnav {

inline tf2::Vector3 frdToFlu(const tf2::Vector3 & vector) {
  return tf2::Vector3(vector.x(), -vector.y(), -vector.z());
}

inline tf2::Quaternion nedFrdToEnuFlu(double roll, double pitch, double heading) {
  static const tf2::Quaternion ned_to_enu(0.70710678118, 0.70710678118,
                                          0.0, 0.0);
  static const tf2::Quaternion frd_to_flu(1.0, 0.0, 0.0, 0.0);
  tf2::Quaternion orientation_ned_frd;
  orientation_ned_frd.setRPY(roll, pitch, heading);
  tf2::Quaternion orientation_enu_flu =
      ned_to_enu * orientation_ned_frd * frd_to_flu;
  orientation_enu_flu.normalize();
  return orientation_enu_flu;
}

}  // namespace adnav

#endif  // ADNAV_DRIVER_ORIENTATION_CONVERSION_HPP_
