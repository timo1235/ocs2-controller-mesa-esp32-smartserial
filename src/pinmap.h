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

    #define SSERIAL_RXD        16
    #define SSERIAL_TXD        17

// 74HC597 IN Shift Register
    #define REGISTER_IN_LATCH  4
    #define REGISTER_IN_CLK    5
    #define REGISTER_IN_DATA   27
    #define REGISTER_IN_LOAD   26

// 74HC595 OUT Shift Register
    #define REGISTER_OUT_LATCH 20
    #define REGISTER_OUT_CLK   20
    #define REGISTER_OUT_DATA  20

// ESP32 I2C from OCS2
    #define I2C_OCS2_SDA       20
    #define I2C_OCS2_SCL       20

// I2C for ADC
    #define I2C_SDA            20
    #define I2C_SCL            20

// I2C addresses for ADC
    #define ADS1_ADDRESS       0x48
    #define ADS2_ADDRESS       0x49

#endif