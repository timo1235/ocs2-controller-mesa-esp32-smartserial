#pragma once

#include <Arduino.h>
#include <pinmap.h>

enum OutputPin { OUT8 = 8, OUT4 = 9, OUT7 = 10, OUT3 = 11, OUT6 = 12, OUT2 = 13, OUT5 = 14, OUT1 = 15, SPINDEL_ON_OFF = 0, ENA = 1 };

// Bitfield views for individual pin access
union Mcp0Bits {
    uint16_t raw;
    struct {
        uint16_t in12 : 1;
        uint16_t in4 : 1;
        uint16_t in11 : 1;
        uint16_t in3 : 1;
        uint16_t in10 : 1;
        uint16_t in2 : 1;
        uint16_t in9 : 1;
        uint16_t in1 : 1;
        uint16_t in16 : 1;
        uint16_t in8 : 1;
        uint16_t in15 : 1;
        uint16_t in7 : 1;
        uint16_t in14 : 1;
        uint16_t in6 : 1;
        uint16_t in13 : 1;
        uint16_t in5 : 1;
    } bits;
};

union Mcp1Bits {
    uint16_t raw;
    struct {
        uint16_t motorStart : 1;
        uint16_t programmStart : 1;
        uint16_t OK : 1;
        uint16_t IO1 : 1;
        uint16_t IO2 : 1;
        uint16_t IO3 : 1;
        uint16_t IO4 : 1;
        uint16_t IO5 : 1;
        uint16_t auswahlX : 1;
        uint16_t alarmAll : 1;
        uint16_t auswahlY : 1;
        uint16_t speed1 : 1;
        uint16_t auswahlZ : 1;
        uint16_t speed2 : 1;
        uint16_t IO6 : 1;
        uint16_t IO7 : 1;
    } bits;
};

// Thread-safe snapshot of both MCP input expanders
struct InputSnapshot {
    Mcp0Bits mcp0;
    Mcp1Bits mcp1;
};

class IORegister {
  public:
    IORegister();
    void init();
    void setOutput(OutputPin pin, uint8_t value);
    void setAllOutputs(uint16_t bits);

    // Thread-safe: returns consistent snapshot of both input expanders
    InputSnapshot getInputSnapshot();

  private:
    void updateInputs();

    // Input snapshot - written by Core 0 (inputUpdateTask), read by Core 1 (sserial)
    InputSnapshot _inputSnapshot;
    portMUX_TYPE  _inputMux;

    // Output state - written by Core 1 (sserial via setOutput), read by Core 0 (outputUpdateTask)
    volatile uint16_t _pendingOutput;
    uint16_t          _currentOutput;
    portMUX_TYPE      _outputMux;

    static volatile bool mcp0InterruptFlag;
    static volatile bool mcp1InterruptFlag;

    static void           inputUpdateTask(void *pvParameters);
    static void           outputUpdateTask(void *pvParameters);
    static void IRAM_ATTR handleInterruptMCP0();
    static void IRAM_ATTR handleInterruptMCP1();
};
