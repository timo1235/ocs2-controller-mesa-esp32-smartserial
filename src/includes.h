#pragma once

#include <Arduino.h>
#include <ADCManager.h>
#include <ioRegister.h>
#include <debugHelper.h>
#include <pinmap.h>

// Externals
extern IORegister ioRegister;
extern ADCManager adcManager;
extern Debug      debug;
extern bool       sserial_timeoutFlag;

#define SSERIAL_CPU 1
#define DEFAULT_CPU 0
