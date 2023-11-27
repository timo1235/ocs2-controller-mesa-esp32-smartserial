#include <includes.h>

RegisterIn::RegisterIn(int dataPin, int clockPin, int latchPin, int loadPin)
    : dataPin(dataPin), clockPin(clockPin), latchPin(latchPin), loadPin(loadPin) {
    pinMode(dataPin, INPUT);
    pinMode(clockPin, OUTPUT);
    pinMode(latchPin, OUTPUT);
    pinMode(REGISTER_IN_LOAD, OUTPUT);
}

void RegisterIn::updateInputs() {
    // Setzt PL-Pin kurz auf LOW, um parallele Daten zu übertragen
    digitalWrite(REGISTER_IN_LOAD, LOW);
    delayMicroseconds(5);
    digitalWrite(REGISTER_IN_LOAD, HIGH);
    // Setzt STCP-Pin kurz auf HIGH, um Daten ins Schieberegister zu laden
    digitalWrite(latchPin, LOW);
    delayMicroseconds(5);
    digitalWrite(latchPin, HIGH);

    // Reset Datenbytes
    data.bytes.byte0 = 0;
    data.bytes.byte1 = 0;
    data.bytes.byte2 = 0;
    data.bytes.byte3 = 0;

    // Lesen der Daten aus allen drei Registern
    for (int j = 0; j < 4; j++) {
        for (int i = 0; i < 8; i++) {
            // Ein Bit einlesen
            int bitVal = digitalRead(dataPin);
            switch (j) {
            case 0:
                data.bytes.byte0 |= (bitVal << (7 - i));
                break;
            case 1:
                data.bytes.byte1 |= (bitVal << (7 - i));
                break;
            case 2:
                data.bytes.byte2 |= (bitVal << (7 - i));
                break;
            case 3:
                data.bytes.byte3 |= (bitVal << (7 - i));
                break;
            }

            // Nächstes Bit vorbereiten
            digitalWrite(clockPin, HIGH);
            delayMicroseconds(5);
            digitalWrite(clockPin, LOW);
        }
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

        vTaskDelay(pdMS_TO_TICKS(10));
    }
}