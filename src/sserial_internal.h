#pragma once

// Internal shared state for sserial module.
// Not part of the public API - only included by sserial_*.cpp files.

#include "sserial.h"
#include <Arduino.h>

// --- Constants ---
#define WDT_TIMEOUT_SEC    5
#define SSERIAL_TIMEOUT_MS 5000
#define BYTE_TIMEOUT_US    200  // Max wait for remaining packet bytes at 2.5 MBaud (~4µs/byte)

// --- Process data types ---
#pragma pack(push, 1)
typedef struct {
    uint16_t out1 : 1;
    uint16_t out2 : 1;
    uint16_t out3 : 1;
    uint16_t out4 : 1;
    uint16_t out5 : 1;
    uint16_t out6 : 1;
    uint16_t out7 : 1;
    uint16_t out8 : 1;
    uint16_t ena : 1;
    uint16_t spindel : 1;
    uint16_t padding : 6;
} sserial_out_process_data_t;
static_assert(sizeof(sserial_out_process_data_t) == 2, "sserial_out_process_data_t must be 2 bytes");

typedef struct {
    int8_t  joy_x;
    int8_t  joy_y;
    int8_t  joy_z;
    uint8_t feedrate;
    uint8_t rotation;
    uint8_t in1 : 1;
    uint8_t in2 : 1;
    uint8_t in3 : 1;
    uint8_t in4 : 1;
    uint8_t in5 : 1;
    uint8_t in6 : 1;
    uint8_t in7 : 1;
    uint8_t in8 : 1;
    uint8_t in9 : 1;
    uint8_t in10 : 1;
    uint8_t in11 : 1;
    uint8_t in12 : 1;
    uint8_t in13 : 1;
    uint8_t in14 : 1;
    uint8_t in15 : 1;
    uint8_t in16 : 1;
    uint8_t alarm : 1;
    uint8_t ok : 1;
    uint8_t motorstart : 1;
    uint8_t programmstart : 1;
    uint8_t auswahlx : 1;
    uint8_t auswahly : 1;
    uint8_t auswahlz : 1;
    uint8_t speed1 : 1;
    uint8_t speed2 : 1;
    uint8_t padding : 7;
} sserial_in_process_data_t;
static_assert(sizeof(sserial_in_process_data_t) == 9, "sserial_in_process_data_t must be 9 bytes");
#pragma pack(pop)

// --- Shared state (defined in sserial.cpp) ---
extern volatile uint8_t            txbuf[128];
extern uint16_t                    address;
extern lbp_t                       lbp;
extern uint8_t                     crc_error_count;
extern const char                  name[];
extern unit_no_t                   unit;
extern uint8_t                     sserial_slave[];
extern const size_t                sserial_slave_size;
extern const discovery_rpc_t       discovery;
extern sserial_out_process_data_t  data_out;
extern sserial_in_process_data_t   data_in;

// --- Internal functions (defined in sserial.cpp) ---
void send(uint8_t len, uint8_t docrc);
void emptySerialBuffer();
bool waitForBytes(int count);
