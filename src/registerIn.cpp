#include <includes.h>

RegisterIn::RegisterIn(int dataPin, int clockPin, int loadPin) : dataPin(dataPin), clockPin(clockPin), loadPin(loadPin) {
    pinMode(dataPin, INPUT);
    pinMode(clockPin, OUTPUT);
    pinMode(loadPin, OUTPUT);
}

void RegisterIn::updateInputs() {
    digitalWrite(loadPin, LOW);
    delayMicroseconds(5);
    digitalWrite(loadPin, HIGH);

    regData.inputData = 0;
    for (int i = 0; i < 4; ++i) {
        regData.inputData <<= 8;
        regData.inputData |= shiftIn(dataPin, clockPin, MSBFIRST);
    }
}

bool RegisterIn::get(Input input) const { return (regData.inputData >> static_cast<int>(input)) & 0x1; }

void RegisterIn::init() {
    xTaskCreatePinnedToCore(RegisterIn::inputUpdateTask,   // Task-Funktion
                            "InputUpdateTask",             // Name des Tasks
                            2048,                          // Stackgröße
                            this,                          // Parameter für die Task-Funktion
                            1,                             // Priorität
                            NULL,                          // Task-Handle
                            DEFAULT_CPU                    // Core-ID
    );
}

void RegisterIn::inputUpdateTask(void *pvParameters) {
    RegisterIn *registerInstance = static_cast<RegisterIn *>(pvParameters);
    for (;;) {
        registerInstance->updateInputs();
        vTaskDelay(pdMS_TO_TICKS(5));   // Warte 5 Millisekunden
    }
}