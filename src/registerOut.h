#pragma once

#include <Arduino.h>

/**
 * @file RegisterOut.h
 * @brief Header file for the RegisterOut class.
 *
 * This file contains the declaration of the RegisterOut class, which provides an interface for controlling
 * a shift register with multiple outputs.
 */
class RegisterOut {
  public:
    /**
     * @brief Enumeration of the available outputs.
     *
     */
    enum class Output { out1, out2, out3, out4, out5, out6, out7, out8, spindelOnOff, ena, out10, out11, out12, out13, out14, out15 };

    /**
     * @brief Constructor for RegisterOut class.
     *
     * @param dataPin The data pin of the shift register.
     * @param clockPin The clock pin of the shift register.
     * @param latchPin The latch pin of the shift register.
     */
    RegisterOut(int dataPin, int clockPin, int latchPin);

    /**
     * @brief Set the value of an output.
     *
     * @param output The output to set.
     * @param value The value to set the output to.
     */
    void set(Output output, bool value);

    /**
     * @brief Update the outputs of the shift register.
     *
     */
    void updateOutputs();

    /**
     * @brief Initialize the shift register.
     *
     */
    void init();

  private:
    int         dataPin, clockPin, latchPin;
    static void outputUpdateTask(void *pvParameters);

    /**
     * @brief Union for storing the register data.
     *
     */
    union {
        struct {
            uint8_t firstByte;    // First byte for the first 8 outputs
            uint8_t secondByte;   // Second byte for the next 8 outputs
        } bytes;

        struct {
            uint8_t out8 : 1;
            uint8_t out4 : 1;
            uint8_t out3 : 1;
            uint8_t out7 : 1;
            uint8_t out2 : 1;
            uint8_t out6 : 1;
            uint8_t out1 : 1;
            uint8_t out5 : 1;
            uint8_t spindelOnOff : 1;
            uint8_t ena : 1;
            uint8_t out10 : 1;
            uint8_t out11 : 1;
            uint8_t out12 : 1;
            uint8_t out13 : 1;
            uint8_t out14 : 1;
            uint8_t out15 : 1;
        } bits;
    } regData;
};