#include <gtest/gtest.h>
#include "crc8.h"

// Helper: compute CRC8 of a byte buffer in one call
static crc8_t crc8_of(const uint8_t* data, size_t len) {
    return crc8_finalize(crc8_update(crc8_init(), data, len));
}

TEST(CRC8, InitReturnsZero) {
    EXPECT_EQ(crc8_init(), 0x00);
}

TEST(CRC8, FinalizeIsIdentity) {
    // XorOut = 0x00, finalize changes nothing
    EXPECT_EQ(crc8_finalize(0x00), 0x00);
    EXPECT_EQ(crc8_finalize(0x42), 0x42);
    EXPECT_EQ(crc8_finalize(0xFF), 0xFF);
}

TEST(CRC8, EmptyDataReturnsSeed) {
    EXPECT_EQ(crc8_of(nullptr, 0), 0x00);
}

TEST(CRC8, SingleByteZero) {
    // table[0] = 0x00, so CRC({0x00}) = 0x00
    uint8_t data[] = {0x00};
    EXPECT_EQ(crc8_of(data, 1), 0x00);
}

TEST(CRC8, SingleByteKnownValues) {
    // Verify against table: CRC({x}) = table[x] for single byte
    uint8_t d1[] = {0x01};
    EXPECT_EQ(crc8_of(d1, 1), 0x5E);

    uint8_t d2[] = {0x02};
    EXPECT_EQ(crc8_of(d2, 1), 0xBC);

    uint8_t d3[] = {0x03};
    EXPECT_EQ(crc8_of(d3, 1), 0xE2);
}

TEST(CRC8, LBPCookieResponse) {
    // LBPCookie = 0x5A, CRC = table[0x5A]
    uint8_t data[] = {0x5A};
    crc8_t crc = crc8_of(data, 1);
    EXPECT_NE(crc, 0x00);
    // CRC must be deterministic
    EXPECT_EQ(crc, crc8_of(data, 1));
}

TEST(CRC8, IncrementalEqualsFullComputation) {
    uint8_t data[] = {0xDF, 0x5A, 0x01, 0x02};

    crc8_t full = crc8_of(data, 4);

    crc8_t step = crc8_init();
    step = crc8_update(step, data, 2);
    step = crc8_update(step, data + 2, 2);
    step = crc8_finalize(step);

    EXPECT_EQ(full, step);
}

TEST(CRC8, ByteByByteEqualsBlock) {
    uint8_t data[] = {0xBB, 0x0A, 0x02, 0xA3, 0x04, 0xF9};

    crc8_t block = crc8_of(data, sizeof(data));

    crc8_t byte_by_byte = crc8_init();
    for (size_t i = 0; i < sizeof(data); i++) {
        byte_by_byte = crc8_update(byte_by_byte, &data[i], 1);
    }
    byte_by_byte = crc8_finalize(byte_by_byte);

    EXPECT_EQ(block, byte_by_byte);
}

TEST(CRC8, DifferentDataProducesDifferentCRC) {
    uint8_t a[] = {0x01, 0x02};
    uint8_t b[] = {0x01, 0x03};
    EXPECT_NE(crc8_of(a, 2), crc8_of(b, 2));
}
