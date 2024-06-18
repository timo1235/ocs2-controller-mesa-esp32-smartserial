#pragma once

#include <Arduino.h>

// enum OutputPin { OUT8 = 8, OUT4 = 12, OUT7 = 9, OUT3 = 13, OUT6 = 10, OUT2 = 14, OUT5 = 11, OUT1 = 15, SPINDEL_ON_OFF = 0, ENA = 1 };
enum OutputPin { OUT8 = 8, OUT4 = 9, OUT7 = 10, OUT3 = 11, OUT6 = 12, OUT2 = 13, OUT5 = 14, OUT1 = 15, SPINDEL_ON_OFF = 0, ENA = 1 };

#define MCP0_INTA 33
#define MCP0_INTB 25
#define MCP1_INTA 26
#define MCP1_INTB 27

class IORegister {
  public:
    IORegister();
    void updateInputs();
    void init();
    void setOutput(OutputPin pin, uint8_t value);

    union {
        uint16_t inputData;
        struct {
            uint32_t in12 : 1;
            uint32_t in4 : 1;
            uint32_t in11 : 1;
            uint32_t in3 : 1;
            uint32_t in10 : 1;
            uint32_t in2 : 1;
            uint32_t in9 : 1;
            uint32_t in1 : 1;
            uint32_t in16 : 1;
            uint32_t in8 : 1;
            uint32_t in15 : 1;
            uint32_t in7 : 1;
            uint32_t in14 : 1;
            uint32_t in6 : 1;
            uint32_t in13 : 1;
            uint32_t in5 : 1;
        } bits;
    } mcp0Data;

    union {
        uint16_t inputData;
        struct {
            uint32_t motorStart : 1;
            uint32_t programmStart : 1;
            uint32_t OK : 1;
            uint32_t IO1 : 1;
            uint32_t IO2 : 1;
            uint32_t IO3 : 1;
            uint32_t IO4 : 1;
            uint32_t IO5 : 1;
            uint32_t auswahlX : 1;
            uint32_t alarmAll : 1;
            uint32_t auswahlY : 1;
            uint32_t speed1 : 1;
            uint32_t auswahlZ : 1;
            uint32_t speed2 : 1;
            uint32_t IO6 : 1;
            uint32_t IO7 : 1;
        } bits;
    } mcp1Data;

  private:
    static volatile bool mcp0InterruptFlag;
    static volatile bool mcp1InterruptFlag;
    static uint16_t      outputStatus;
    uint16_t             localOutputStatus;

    static void           inputUpdateTask(void *pvParameters);
    static void           outputUpdateTask(void *pvParameters);
    static void IRAM_ATTR handleInterruptMCP0();
    static void IRAM_ATTR handleInterruptMCP1();
};