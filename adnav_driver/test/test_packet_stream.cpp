#include <cstdint>
#include <cstring>
#include <vector>

#include <gtest/gtest.h>

#include "an_packet_protocol.h"

namespace
{

struct DecodedPacket
{
  std::uint8_t id;
  std::vector<std::uint8_t> payload;
};

std::vector<std::uint8_t> encodePacket(
    std::uint8_t id, const std::vector<std::uint8_t> &payload)
{
  auto *packet = an_packet_allocate(static_cast<std::uint8_t>(payload.size()), id);
  if (packet == nullptr) {
    ADD_FAILURE() << "Unable to allocate test packet";
    return {};
  }
  std::memcpy(packet->data, payload.data(), payload.size());
  an_packet_encode(packet);
  const auto *begin = an_packet_pointer(packet);
  std::vector<std::uint8_t> encoded(begin, begin + an_packet_size(packet));
  an_packet_free(&packet);
  return encoded;
}

void receiveChunk(
    an_decoder_t &decoder, const std::uint8_t *data, std::size_t size,
    std::vector<DecodedPacket> &decoded)
{
  ASSERT_LE(size, an_decoder_size(&decoder));
  if (size > 0) {
    std::memcpy(an_decoder_pointer(&decoder), data, size);
    an_decoder_increment(&decoder, size);
  }
  while (auto *packet = an_packet_decode(&decoder)) {
    decoded.push_back(
        {packet->id, std::vector<std::uint8_t>(packet->data, packet->data + packet->length)});
    an_packet_free(&packet);
  }
}

TEST(PacketStream, RetainsPacketAcrossEveryReadBoundary)
{
  std::vector<std::uint8_t> payload(AN_MAXIMUM_PACKET_SIZE);
  for (std::size_t index = 0; index < payload.size(); ++index) {
    payload[index] = static_cast<std::uint8_t>(index);
  }
  const auto encoded = encodePacket(20, payload);
  ASSERT_EQ(encoded.size(), AN_PACKET_HEADER_SIZE + payload.size());

  for (std::size_t boundary = 1; boundary < encoded.size(); ++boundary) {
    SCOPED_TRACE(boundary);
    an_decoder_t decoder{};
    an_decoder_initialise(&decoder);
    std::vector<DecodedPacket> decoded;
    receiveChunk(decoder, encoded.data(), boundary, decoded);
    EXPECT_TRUE(decoded.empty());
    receiveChunk(decoder, nullptr, 0, decoded);
    EXPECT_TRUE(decoded.empty());
    receiveChunk(
        decoder, encoded.data() + boundary, encoded.size() - boundary, decoded);
    ASSERT_EQ(decoded.size(), 1U);
    EXPECT_EQ(decoded.front().id, 20U);
    EXPECT_EQ(decoded.front().payload, payload);
    EXPECT_EQ(decoder.buffer_length, 0U);
  }
}

TEST(PacketStream, RetainsTrailingPacketAfterDecodingCompletePacket)
{
  const std::vector<std::uint8_t> first_payload{1, 2, 3, 4};
  const std::vector<std::uint8_t> second_payload{5, 6, 7};
  auto stream = encodePacket(23, first_payload);
  const auto second = encodePacket(28, second_payload);
  stream.insert(stream.end(), second.begin(), second.end());

  an_decoder_t decoder{};
  an_decoder_initialise(&decoder);
  std::vector<DecodedPacket> decoded;
  const auto first_chunk_size = stream.size() - 2;
  receiveChunk(decoder, stream.data(), first_chunk_size, decoded);
  ASSERT_EQ(decoded.size(), 1U);
  EXPECT_EQ(decoded.front().payload, first_payload);

  receiveChunk(
      decoder, stream.data() + first_chunk_size, stream.size() - first_chunk_size, decoded);
  ASSERT_EQ(decoded.size(), 2U);
  EXPECT_EQ(decoded[1].id, 28U);
  EXPECT_EQ(decoded[1].payload, second_payload);
  EXPECT_EQ(decoder.buffer_length, 0U);
}

TEST(PacketStream, DecodesPacketsFromSingleByteReads)
{
  const std::vector<std::uint8_t> payload{0, 255, 16, 32};
  const auto encoded = encodePacket(39, payload);
  ASSERT_FALSE(encoded.empty());
  an_decoder_t decoder{};
  an_decoder_initialise(&decoder);
  std::vector<DecodedPacket> decoded;
  for (const auto &byte : encoded) {
    receiveChunk(decoder, &byte, 1, decoded);
  }
  ASSERT_EQ(decoded.size(), 1U);
  EXPECT_EQ(decoded.front().id, 39U);
  EXPECT_EQ(decoded.front().payload, payload);
}

}  // namespace
