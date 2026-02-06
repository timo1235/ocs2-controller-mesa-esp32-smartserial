#pragma once

#include <Arduino.h>

struct ADCSnapshot {
    int16_t joystickX;
    int16_t joystickY;
    int16_t joystickZ;
    int16_t feedrate;
    int16_t rotationSpeed;
};

class ADCManager {
  public:
    ADCManager();

    void init();

    // Thread-safe: returns consistent snapshot of all ADC values
    ADCSnapshot getSnapshot();

  private:
    static void  readInputsTask(void *pvParameters);
    ADCSnapshot  _snapshot;
    portMUX_TYPE _adcMux;
};
