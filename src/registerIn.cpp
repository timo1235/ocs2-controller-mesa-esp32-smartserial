#include <includes.h>

RegisterIn::RegisterIn(int dataPin, int clockPin, int latchPin, int loadPin) : dataPin(dataPin), clockPin(clockPin), latchPin(latchPin), loadPin(loadPin) {
    pinMode(dataPin, INPUT);
    pinMode(clockPin, OUTPUT);
    pinMode(latchPin, OUTPUT);
    pinMode(loadPin, OUTPUT);

    digitalWrite(loadPin, HIGH);
    digitalWrite(clockPin, LOW);
    digitalWrite(latchPin, LOW);
}

void RegisterIn::updateInputs() {
    // Load the data into the shift register
    digitalWrite(latchPin, HIGH);
    delayMicroseconds(1);   // Short delay to ensure the load operation is completed
    digitalWrite(latchPin, LOW);
    delayMicroseconds(1);   // Short delay to ensure the load operation is completed
    digitalWrite(loadPin, LOW);
    delayMicroseconds(1);   // Short delay to ensure the load operation is completed
    digitalWrite(loadPin, HIGH);

    // Clear inputData
    data.inputData = 0;

    // Read the data from the shift register
    for (int i = 0; i < 32; i++) {
        delayMicroseconds(1);   // Short delay to ensure stable clock signal
        // Read the current bit
        if (digitalRead(dataPin)) {
            data.inputData |= (1UL << i);
        }
        digitalWrite(clockPin, HIGH);
        delayMicroseconds(5);   // Short delay to ensure stable clock signal
        digitalWrite(clockPin, LOW);
    }
}

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

        vTaskDelay(pdMS_TO_TICKS(6));   // wait 5 ms
    }
}