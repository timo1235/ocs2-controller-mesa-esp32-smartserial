#include <gtest/gtest.h>
#include <cstring>
#include "sserial_internal.h"

// --- Output Process Data ---

TEST(ProcessData, OutputStructSize) {
    EXPECT_EQ(sizeof(sserial_out_process_data_t), 2);
}

TEST(ProcessData, OutputAllZero) {
    sserial_out_process_data_t out;
    memset(&out, 0, sizeof(out));
    uint16_t raw;
    memcpy(&raw, &out, 2);
    EXPECT_EQ(raw, 0x0000);
}

TEST(ProcessData, OutputBitPositions) {
    // Verify each output bit maps to the expected position
    sserial_out_process_data_t out;
    uint16_t raw;

    memset(&out, 0, sizeof(out));
    out.out1 = 1;
    memcpy(&raw, &out, 2);
    EXPECT_EQ(raw, 0x0001);  // bit 0

    memset(&out, 0, sizeof(out));
    out.out8 = 1;
    memcpy(&raw, &out, 2);
    EXPECT_EQ(raw, 0x0080);  // bit 7

    memset(&out, 0, sizeof(out));
    out.ena = 1;
    memcpy(&raw, &out, 2);
    EXPECT_EQ(raw, 0x0100);  // bit 8

    memset(&out, 0, sizeof(out));
    out.spindel = 1;
    memcpy(&raw, &out, 2);
    EXPECT_EQ(raw, 0x0200);  // bit 9
}

TEST(ProcessData, OutputDeserialization) {
    // Simulate 2 bytes received from Mesa: out1=1, out3=1, ena=1
    uint8_t bytes[] = {0x05, 0x01};  // bits 0,2,8
    sserial_out_process_data_t out;
    memcpy(&out, bytes, 2);

    EXPECT_EQ(out.out1, 1);
    EXPECT_EQ(out.out2, 0);
    EXPECT_EQ(out.out3, 1);
    EXPECT_EQ(out.out4, 0);
    EXPECT_EQ(out.ena, 1);
    EXPECT_EQ(out.spindel, 0);
}

// --- Input Process Data ---

TEST(ProcessData, InputStructSize) {
    EXPECT_EQ(sizeof(sserial_in_process_data_t), 9);
}

TEST(ProcessData, InputJoystickRange) {
    sserial_in_process_data_t in;
    memset(&in, 0, sizeof(in));

    in.joy_x = -128;
    EXPECT_EQ(in.joy_x, -128);

    in.joy_x = 127;
    EXPECT_EQ(in.joy_x, 127);

    in.joy_x = 0;
    EXPECT_EQ(in.joy_x, 0);
}

TEST(ProcessData, InputAnalogFieldOffsets) {
    sserial_in_process_data_t in;
    memset(&in, 0, sizeof(in));

    in.joy_x    = 0x11;
    in.joy_y    = 0x22;
    in.joy_z    = 0x33;
    in.feedrate = 0x44;
    in.rotation = 0x55;

    uint8_t* raw = reinterpret_cast<uint8_t*>(&in);
    EXPECT_EQ(raw[0], 0x11);  // joy_x at offset 0
    EXPECT_EQ(raw[1], 0x22);  // joy_y at offset 1
    EXPECT_EQ(raw[2], 0x33);  // joy_z at offset 2
    EXPECT_EQ(raw[3], 0x44);  // feedrate at offset 3
    EXPECT_EQ(raw[4], 0x55);  // rotation at offset 4
}

TEST(ProcessData, InputDigitalBits) {
    sserial_in_process_data_t in;
    memset(&in, 0, sizeof(in));

    in.in1 = 1;
    in.in16 = 1;

    uint8_t* raw = reinterpret_cast<uint8_t*>(&in);
    // in1 is bit 0 of byte 5, in16 is bit 7 of byte 6
    EXPECT_EQ(raw[5] & 0x01, 1);      // in1
    EXPECT_EQ((raw[6] >> 7) & 0x01, 1); // in16
}

TEST(ProcessData, InputControlBits) {
    sserial_in_process_data_t in;
    memset(&in, 0, sizeof(in));

    in.alarm = 1;
    in.ok = 1;
    in.motorstart = 1;

    // These are in byte 7 (after 5 analog + 2 digital bytes)
    uint8_t* raw = reinterpret_cast<uint8_t*>(&in);
    // alarm=bit0, ok=bit1, motorstart=bit2 of byte 7
    EXPECT_EQ(raw[7] & 0x01, 1);        // alarm
    EXPECT_EQ((raw[7] >> 1) & 0x01, 1); // ok
    EXPECT_EQ((raw[7] >> 2) & 0x01, 1); // motorstart
}

TEST(ProcessData, InputSerialization) {
    // Build input data as the SmartSerial handler would send it
    sserial_in_process_data_t in;
    memset(&in, 0, sizeof(in));

    in.joy_x    = -50;
    in.joy_y    = 100;
    in.feedrate = 200;
    in.in1      = 1;
    in.in5      = 1;
    in.alarm    = 1;

    // Verify the struct can be copied as raw bytes (as processIncomingData does)
    uint8_t buf[9];
    memcpy(buf, &in, 9);

    sserial_in_process_data_t in2;
    memcpy(&in2, buf, 9);

    EXPECT_EQ(in2.joy_x, -50);
    EXPECT_EQ(in2.joy_y, 100);
    EXPECT_EQ(in2.feedrate, 200);
    EXPECT_EQ(in2.in1, 1);
    EXPECT_EQ(in2.in5, 1);
    EXPECT_EQ(in2.alarm, 1);
    EXPECT_EQ(in2.in2, 0);
    EXPECT_EQ(in2.ok, 0);
}

// --- Analog Conversion (bit-shift approximation from sserial_io.cpp) ---

TEST(AnalogConversion, JoystickMapping) {
    // (x >> 4) - 128: maps 0..4095 to -128..127
    auto map_joy = [](int16_t x) -> int8_t {
        return (int8_t)((x >> 4) - 128);
    };

    EXPECT_EQ(map_joy(0), -128);
    EXPECT_EQ(map_joy(4095), 127);
    EXPECT_EQ(map_joy(2048), 0);
    // Near center
    EXPECT_NEAR(map_joy(2000), -3, 1);
}

TEST(AnalogConversion, FeedrateMapping) {
    // x >> 4: maps 0..4095 to 0..255
    auto map_feed = [](int16_t x) -> uint8_t {
        return (uint8_t)(x >> 4);
    };

    EXPECT_EQ(map_feed(0), 0);
    EXPECT_EQ(map_feed(4095), 255);
    EXPECT_EQ(map_feed(2048), 128);
}
