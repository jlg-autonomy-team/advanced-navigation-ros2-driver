#include <gtest/gtest.h>

#include <cmath>

#include "orientation_conversion.hpp"
#include "packet_contract.hpp"

namespace adnav {
namespace {

TEST(PacketDefinition, Packet39IsAcceptedAndMappedToOrientationUpdate) {
  EXPECT_TRUE(isRosDriverPacketId(packet_id_euler_orientation));
  EXPECT_EQ(rosDriverPendingUpdateForPacket(packet_id_euler_orientation),
            kPacket39Update);
}

TEST(Packet39, ConvertsNedFrdAttitudeToEnuFlu) {
  constexpr double kHalfPi = 1.5707963267948966;
  const auto facing_east = nedFrdToEnuFlu(0.0, 0.0, kHalfPi);
  EXPECT_NEAR(facing_east.x(), 0.0, 1.0e-6);
  EXPECT_NEAR(facing_east.y(), 0.0, 1.0e-6);
  EXPECT_NEAR(facing_east.z(), 0.0, 1.0e-6);
  EXPECT_NEAR(std::abs(facing_east.w()), 1.0, 1.0e-6);

  const auto facing_north = nedFrdToEnuFlu(0.0, 0.0, 0.0);
  EXPECT_NEAR(facing_north.x(), 0.0, 1.0e-6);
  EXPECT_NEAR(facing_north.y(), 0.0, 1.0e-6);
  EXPECT_NEAR(std::abs(facing_north.z()), 0.70710678, 1.0e-6);
  EXPECT_NEAR(std::abs(facing_north.w()), 0.70710678, 1.0e-6);

  const auto vector_flu = frdToFlu(tf2::Vector3(1.0, 2.0, 3.0));
  EXPECT_NEAR(vector_flu.x(), 1.0, 1.0e-6);
  EXPECT_NEAR(vector_flu.y(), -2.0, 1.0e-6);
  EXPECT_NEAR(vector_flu.z(), -3.0, 1.0e-6);
}

TEST(PacketDefinition, EcefPacketIsHandledByTheDecoder) {
  EXPECT_TRUE(isRosDriverPacketId(packet_id_ecef_position));
  EXPECT_EQ(rosDriverPendingUpdateForPacket(packet_id_ecef_position),
            kPacket33Update);
}

TEST(PacketDefinition, RawGnssPacketIsAcceptedForDiagnosticsOnly) {
  EXPECT_TRUE(isRosDriverPacketId(packet_id_raw_gnss));
  EXPECT_EQ(rosDriverPendingUpdateForPacket(packet_id_raw_gnss), 0u);
}

TEST(PacketDefinition, UnknownPacketIsNotAccepted) {
  EXPECT_FALSE(isRosDriverPacketId(179));
  EXPECT_EQ(rosDriverPendingUpdateForPacket(179), 0u);
}

TEST(PacketDefinition, ConfiguredPacket39RequestIsAccepted) {
  const std::vector<int64_t> request{20, 10, 23, 10, 24, 10, 25, 10,
                                     26, 10, 28, 10, 29, 10, 39, 10, 42, 10};
  EXPECT_TRUE(isRosDriverPacketRequest(request, 1, 65535));
}

TEST(PacketTimestamp, ZeroOrInvalidUtcUsesReceiptTime) {
  constexpr int64_t receipt = 9876543210LL;
  EXPECT_EQ(packet20StampNanoseconds(0, 12, true, receipt), receipt);
  EXPECT_EQ(packet20StampNanoseconds(1700000000, 1000000, true, receipt),
            receipt);
  EXPECT_EQ(packet20StampNanoseconds(1700000000, 12, false, receipt), receipt);
}

TEST(PacketTimestamp, ValidUtcIsUsed) {
  EXPECT_EQ(packet20StampNanoseconds(1700000000, 123456, true, 1),
            1700000000123456000LL);
}

TEST(PublicationUpdates, UnrelatedPacketsDoNotSetOtherSources) {
  EXPECT_EQ(rosDriverPendingUpdateForPacket(packet_id_raw_sensors),
            kPacket28Update);
  EXPECT_EQ(rosDriverPendingUpdateForPacket(packet_id_status), kPacket23Update);
  EXPECT_EQ(rosDriverPendingUpdateForPacket(packet_id_euler_orientation),
            kPacket39Update);
  EXPECT_EQ(kPacket28Update & kPacket23Update, 0u);
}

TEST(PacketTimestamp, MonotonicAdjustmentOnlyMovesBackwardsOrEqualValues) {
  EXPECT_EQ(monotonicStampNanoseconds(100, 0), 100);
  EXPECT_EQ(monotonicStampNanoseconds(100, 100), 101);
  EXPECT_EQ(monotonicStampNanoseconds(99, 100), 101);
}

} // namespace
} // namespace adnav
