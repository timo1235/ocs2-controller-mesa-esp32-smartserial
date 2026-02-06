#include <gtest/gtest.h>
#include "sserial.h"

// --- Command Type Parsing ---

TEST(LBPTypes, LocalReadCommand) {
    lbp_t cmd;
    cmd.byte = LBPCookieCMD;  // 0xDF
    EXPECT_EQ(cmd.ct, CT_LOCAL);
    EXPECT_EQ(cmd.wr, 0);
}

TEST(LBPTypes, LocalWriteCommand) {
    lbp_t cmd;
    cmd.byte = 0xFF;  // Reset parser (local write)
    EXPECT_EQ(cmd.ct, CT_LOCAL);
    EXPECT_EQ(cmd.wr, 1);
}

TEST(LBPTypes, StatusIsLocalRead) {
    lbp_t cmd;
    cmd.byte = LBPStatusCMD;  // 0xC1
    EXPECT_EQ(cmd.ct, CT_LOCAL);
    EXPECT_EQ(cmd.wr, 0);
}

// --- RPC Commands ---

TEST(LBPTypes, DiscoveryRPC) {
    lbp_t cmd;
    cmd.byte = DiscoveryRPC;  // 0xBB
    EXPECT_EQ(cmd.ct, CT_RPC);
}

TEST(LBPTypes, UnitNumberRPC) {
    lbp_t cmd;
    cmd.byte = UnitNumberRPC;  // 0xBC
    EXPECT_EQ(cmd.ct, CT_RPC);
}

TEST(LBPTypes, ProcessDataRPC) {
    lbp_t cmd;
    cmd.byte = ProcessDataRPC;  // 0xBD
    EXPECT_EQ(cmd.ct, CT_RPC);
}

// --- R/W Commands ---

TEST(LBPTypes, RWReadCurrentAddr1Byte) {
    // ct=01, wr=0, rid=0, ai=0, as=0, ds=00
    lbp_t cmd;
    cmd.byte = 0x40;
    EXPECT_EQ(cmd.ct, CT_RW);
    EXPECT_EQ(cmd.wr, 0);
    EXPECT_EQ(cmd.as, 0);
    EXPECT_EQ(cmd.ai, 0);
    EXPECT_EQ(cmd.ds, 0);
    EXPECT_EQ(1 << cmd.ds, 1);  // 1 byte
}

TEST(LBPTypes, RWWriteWithAddr2Bytes) {
    // ct=01, wr=1, rid=0, ai=0, as=1, ds=01
    lbp_t cmd;
    cmd.byte = (CT_RW << 6) | (1 << 5) | (1 << 2) | 1;
    EXPECT_EQ(cmd.ct, CT_RW);
    EXPECT_EQ(cmd.wr, 1);
    EXPECT_EQ(cmd.as, 1);
    EXPECT_EQ(cmd.ds, 1);
    EXPECT_EQ(1 << cmd.ds, 2);  // 2 bytes
}

TEST(LBPTypes, RWReadAutoIncrement4Bytes) {
    // ct=01, wr=0, rid=0, ai=1, as=1, ds=10
    lbp_t cmd;
    cmd.byte = (CT_RW << 6) | (1 << 3) | (1 << 2) | 2;
    EXPECT_EQ(cmd.ct, CT_RW);
    EXPECT_EQ(cmd.wr, 0);
    EXPECT_EQ(cmd.ai, 1);
    EXPECT_EQ(cmd.as, 1);
    EXPECT_EQ(cmd.ds, 2);
    EXPECT_EQ(1 << cmd.ds, 4);  // 4 bytes
}

// --- Data Sizes ---

TEST(LBPTypes, DataSizeDecoding) {
    lbp_t cmd;
    for (int ds = 0; ds < 4; ds++) {
        cmd.byte = (CT_RW << 6) | ds;
        EXPECT_EQ(cmd.ds, ds);
        EXPECT_EQ(1 << cmd.ds, 1 << ds);
    }
}

// --- Struct Sizes ---

TEST(LBPTypes, DiscoveryRPCStructSize) {
    EXPECT_EQ(sizeof(discovery_rpc_t), 6);
}

TEST(LBPTypes, UnitNumberStructSize) {
    EXPECT_EQ(sizeof(unit_no_t), 4);
}

TEST(LBPTypes, UnitNumberByteAccess) {
    unit_no_t u;
    u.unit = 0x04030201;
    EXPECT_EQ(u.byte[0], 0x01);
    EXPECT_EQ(u.byte[1], 0x02);
    EXPECT_EQ(u.byte[2], 0x03);
    EXPECT_EQ(u.byte[3], 0x04);
}

TEST(LBPTypes, DiscoveryRPCFieldLayout) {
    discovery_rpc_t d;
    d.input  = 10;
    d.output = 2;
    d.ptocp  = 0x04A3;
    d.gtocp  = 0x04F9;

    uint8_t* raw = reinterpret_cast<uint8_t*>(&d);
    EXPECT_EQ(raw[0], 10);   // input
    EXPECT_EQ(raw[1], 2);    // output
    // ptocp little-endian
    EXPECT_EQ(raw[2], 0xA3);
    EXPECT_EQ(raw[3], 0x04);
    // gtocp little-endian
    EXPECT_EQ(raw[4], 0xF9);
    EXPECT_EQ(raw[5], 0x04);
}

// --- LBP Constants ---

TEST(LBPTypes, CookieConstants) {
    EXPECT_EQ(LBPCookieCMD, 0xDF);
    EXPECT_EQ(LBPCookie, 0x5A);
}

TEST(LBPTypes, CardNameConstants) {
    EXPECT_EQ(LBPCardName0Cmd, 0xD0);
    EXPECT_EQ(LBPCardName3Cmd, 0xD3);
    EXPECT_EQ(LBPCardName3Cmd - LBPCardName0Cmd, 3);
}
