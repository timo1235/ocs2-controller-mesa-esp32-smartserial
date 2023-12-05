#pragma once

#include <ADCManager.h>
#include <Adafruit_ADS1X15.h>
#include <Arduino.h>
#include <SPI.h>
#include <crc8.h>
#include <debugHelper.h>
#include <pinmap.h>
#include <registerIn.h>
#include <registerOut.h>
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

extern RegisterOut outRegister;
extern RegisterIn  inRegister;
extern ADCManager  adcManager;
extern Debug       debug;
extern bool        sserial_timeoutFlag;

// CPU allocation
// TODO: Test tskNO_AFFINITY as Core ID, then a task runs on both cores and can switch between them
// SSerial needs to run on CPU 1 togehter with setup and loop -> make sure loop is not used for anything else
// Only CPU1 is suitable for SSerial, since it can run without calling vtaskDelay, meaning, it can be blocking
// without triggering the watchdog.
#define SSERIAL_CPU 1
// All other tasks can run on CPU 0 to free CPU 1 for SSerial
// CPU 0 needs to call vtaskDelay, otherwise the watchdog triggered
#define DEFAULT_CPU 0
