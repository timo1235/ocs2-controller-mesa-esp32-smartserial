#pragma once

#include <Arduino.h>

/**
 * @file registerIn.h
 * @brief Header file for the RegisterIn class representing an input register.
 *
 */
class RegisterIn {
  public:
    /**
     * @brief Enum representing the input pins.
     *
     */
    enum class Input {
        in1,
        in2,
        in3,
        in4,
        in5,
        in6,
        in7,
        in8,
        in9,
        in10,
        in11,
        in12,
        in13,
        in14,
        in15,
        in16,
        alarmAll,
        programmStart,
        motorStart,
        ok,
        d4_1,
        d5_1,
        d6_1,
        d7_1,
        d0_2,
        auswahlZ,
        speed2,
        auswahlY,
        speed1,
        auswahlX,
        d6_2,
        d7_2
    };

    /**
     * @brief Construct a new RegisterIn object.
     *
     * @param dataPin The data pin.
     * @param clockPin The clock pin.
     * @param loadPin The load pin.
     */
    RegisterIn(int dataPin, int clockPin, int loadPin);

    /**
     * @brief Update the inputs.
     *
     */
    void updateInputs();

    /**
     * @brief Get the input value.
     *
     * @param input The input pin.
     * @return true The input is high.
     * @return false The input is low.
     */
    bool get(Input input) const;

    /**
     * @brief Initialize the input register.
     *
     */
    void init();

  private:
    int dataPin, clockPin, loadPin;

    /**
     * @brief Task to update the inputs.
     *
     * @param pvParameters The parameters for the task.
     */
    static void inputUpdateTask(void *pvParameters);

    union {
        uint32_t inputData;
        struct {
            uint8_t byte3;   // Daten der ersten 8 Eingänge (in0-in7)
            uint8_t byte2;   // Daten der nächsten 8 Eingänge (in8-in15)
            uint8_t byte1;   // Daten der nächsten 8 Eingänge (in16-in23)
            uint8_t byte0;   // Daten der letzten 8 Eingänge (in24-in31)
        } bytes;

        struct {
            uint32_t alarmAll : 1;
            uint32_t programmStart : 1;
            uint32_t motorStart : 1;
            uint32_t ok : 1;
            uint32_t d4_1 : 1;
            uint32_t d5_1 : 1;
            uint32_t d6_1 : 1;
            uint32_t d7_1 : 1;
            uint32_t d0_2 : 1;
            uint32_t auswahlZ : 1;
            uint32_t speed2 : 1;
            uint32_t auswahlY : 1;
            uint32_t speed1 : 1;
            uint32_t auswahlX : 1;
            uint32_t d6_2 : 1;
            uint32_t d7_2 : 1;
            uint32_t in16 : 1;
            uint32_t in15 : 1;
            uint32_t in14 : 1;
            uint32_t in13 : 1;
            uint32_t in12 : 1;
            uint32_t in11 : 1;
            uint32_t in10 : 1;
            uint32_t in9 : 1;
            uint32_t in8 : 1;
            uint32_t in7 : 1;
            uint32_t in6 : 1;
            uint32_t in5 : 1;
            uint32_t in4 : 1;
            uint32_t in3 : 1;
            uint32_t in2 : 1;
            uint32_t in1 : 1;
        } bits;
    } regData;
};