#include <gtest/gtest.h>

#include "packet_contract.hpp"

namespace adnav {
namespace {

TEST(PacketDefinition, Packet39IsAcceptedAndMappedToOrientationUpdate) {
  EXPECT_TRUE(isRosDriverPacketId(packet_id_euler_orientation));
  EXPECT_EQ(rosDriverPendingUpdateForPacket(packet_id_euler_orientation),
            kPacket39Update);
}

TEST(PacketDefinition, EcefPacketIsHandledByTheDecoder) {
  EXPECT_TRUE(isRosDriverPacketId(packet_id_ecef_position));
  EXPECT_EQ(rosDriverPendingUpdateForPacket(packet_id_ecef_position),
            kPacket33Update);
}

TEST(PacketDefinition, UnknownPacketIsNotAccepted) {
  EXPECT_FALSE(isRosDriverPacketId(179));
  EXPECT_EQ(rosDriverPendingUpdateForPacket(179), 0u);
}

TEST(PacketDefinition, ConfiguredPacket39RequestIsAccepted) {
  const std::vector<int64_t> request{20, 10, 23, 10, 24, 10, 25, 10,
                                     26, 10, 28, 10, 39, 10, 42, 10};
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
