#pragma once

#include <Arduino.h>
#include <crc8.h>
#include <debugHelper.h>
#include <pinmap.h>
#include <registerIn.h>
#include <registerOut.h>
#include <sserial.h>

// #define SHOW_DEBUG
// #define SHOW_VERBOSE
// #define SHOW_PDATA_IN

#ifdef SHOW_DEBUG
    #define DEBUG_PRINTF tprint
// #define DEBUG_PRINTF(f_, ...) do {Serial1.printf((f_), ##__VA_ARGS__);} while (0)
#else
    #define DEBUG_PRINTF(...)
#endif

#ifdef SHOW_VERBOSE
    #define VERB_PRINTF tprint
#else
    #define VERB_PRINTF(...)
#endif

#if 1
    #define SERIAL1_FLUSH Serial1.flush
#else
    #define SERIAL1_FLUSH(x)                                                                                                               \
        do {                                                                                                                               \
        } while (0)
#endif

// Externals

extern RegisterOut outRegister;
extern RegisterIn  inRegister;
