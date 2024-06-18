#pragma once

#include <ADCManager.h>
#include <Arduino.h>
#include <IORegister.h>
#include <MCP23S17.h>
#include <SPI.h>
#include <crc8.h>
#include <debugHelper.h>
#include <pinmap.h>
#include <sserial.h>

// #define SHOW_DEBUG
// #define SHOW_VERBOSE
// #define SHOW_PDATA_IN

// #define USE_DEBUG

#ifdef SHOW_DEBUG
    #define DEBUG_PRINTF tprint
// #define DEBUG_PRINTF(f_, ...) do {Serial1.printf((f_), ##__VA_ARGS__);} while (0)
#else
    #define DEBUG_PRINTF(...)
#endif

// Externals

extern IORegister ioRegister;
extern ADCManager adcManager;
extern Debug      debug;
extern bool       sserial_timeoutFlag;

#define SSERIAL_CPU 1
#define DEFAULT_CPU 0
