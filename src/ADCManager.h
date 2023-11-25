#pragma once

#include <Adafruit_ADS1X15.h>

class ADCManager {
  public:
    ADCManager();

    void init();
    // 0V = 0 | 5V ~ 26000
    int16_t readJoystickX();
    // 0V = 0 | 5V ~ 26000
    int16_t readJoystickY();
    // 0V = 0 | 5V ~ 26000
    int16_t readJoystickZ();
    // 0V = 0 | 5V ~ 26000
    int16_t readFeedrate();
    // 0V = 0 | 5V ~ 26000
    int16_t readRotationSpeed();

    int16_t getJoystickX() const;
    int16_t getJoystickY() const;
    int16_t getJoystickZ() const;
    int16_t getFeedrate() const;
    int16_t getRotationSpeed() const;

  private:
    static void      readInputsTask(void *pvParameters);
    Adafruit_ADS1115 adc1;   // ADC mit Adresse 0x48
    Adafruit_ADS1115 adc2;   // ADC mit Adresse 0x49
    volatile int16_t joystickX;
    volatile int16_t joystickY;
    volatile int16_t joystickZ;
    volatile int16_t feedrate;
    volatile int16_t rotationSpeed;
};
