#pragma once

// If the code is compiled for the OCS2, the following defines are used.
#if CM_S2 == 1

    #define SSERIAL_RXD 8
    #define SSERIAL_TXD 9

    // 74HC597 IN Shift Register
    #define REGISTER_IN_LATCH 34
    #define REGISTER_IN_CLK   35
    #define REGISTER_IN_DATA  33

    // 74HC595 OUT Shift Register
    #define REGISTER_OUT_LATCH 37
    #define REGISTER_OUT_CLK   36
    #define REGISTER_OUT_DATA  38

    // ESP32 I2C from OCS2
    #define I2C_OCS2_SDA 39
    #define I2C_OCS2_SCL 40

    // I2C for ADC
    #define I2C_SDA 17
    #define I2C_SCL 18

    // I2C addresses for ADC
    #define ADS1_ADDRESS 0x48
    #define ADS2_ADDRESS 0x49
#else
// For devkit V1
//     #define SSERIAL_RXD        16
//     #define SSERIAL_TXD        17

// // 74HC597 IN Shift Register
//     #define REGISTER_IN_LATCH  4
//     #define REGISTER_IN_CLK    5
//     #define REGISTER_IN_DATA   27
//     #define REGISTER_IN_LOAD   26

// // 74HC595 OUT Shift Register
//     #define REGISTER_OUT_LATCH 20
//     #define REGISTER_OUT_CLK   20
//     #define REGISTER_OUT_DATA  20

// // ESP32 I2C from OCS2
//     #define I2C_OCS2_SDA       20
//     #define I2C_OCS2_SCL       20

// // I2C for ADC
//     #define I2C_SDA            20
//     #define I2C_SCL            20

// // I2C addresses for ADC
//     #define ADS1_ADDRESS       0x48
//     #define ADS2_ADDRESS       0x49

// Version 1.2
    #define SSERIAL_RXD 16
    #define SSERIAL_TXD 17

    #define SPI_MISO           12
    #define SPI_MOSI           13
    #define SPI_SCK            14

// MCP23S17
    #define CS_MCP23S17        15
    #define MCP0_INTA          33
    #define MCP0_INTB          25
    #define MCP1_INTA          26
    #define MCP1_INTB          27

// 74HC595 OUT Shift Register
    #define REGISTER_OUT_LATCH 32
    #define REGISTER_OUT_CLK   33
    #define REGISTER_OUT_DATA  13

// ESP32 I2C from OCS2
    #define I2C_OCS2_SDA       18
    #define I2C_OCS2_SCL       19

// I2C for ADC
    #define I2C_SDA            21
    #define I2C_SCL            22

// I2C addresses for ADC
    #define ADS1_ADDRESS       0x48
    #define ADS2_ADDRESS       0x49

    // Analog Pins
    #define FEEDRATE           32
    #define ROTATION_SPEED     35
    #define JOYSTICK_X         34
    #define JOYSTICK_Y         39
    #define JOYSTICK_Z         36

#endif