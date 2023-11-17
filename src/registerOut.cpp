#include <registerOut.h>

RegisterOut::RegisterOut(int dataPin, int clockPin, int latchPin) : dataPin(dataPin), clockPin(clockPin), latchPin(latchPin) {
    pinMode(dataPin, OUTPUT);
    pinMode(clockPin, OUTPUT);
    pinMode(latchPin, OUTPUT);
}

void RegisterOut::set(Output output, bool value) {
    switch (output) {
    case Output::out1:
        regData.bits.out1 = value;
        break;
    case Output::out2:
        regData.bits.out2 = value;
        break;
    case Output::out3:
        regData.bits.out3 = value;
        break;
    case Output::out4:
        regData.bits.out4 = value;
        break;
    case Output::out5:
        regData.bits.out5 = value;
        break;
    case Output::out6:
        regData.bits.out6 = value;
        break;
    case Output::out7:
        regData.bits.out7 = value;
        break;
    case Output::out8:
        regData.bits.out8 = value;
        break;
    case Output::spindelOnOff:
        regData.bits.spindelOnOff = value;
        break;
    case Output::ena:
        regData.bits.ena = value;
        break;
    case Output::out10:
        regData.bits.out10 = value;
        break;
    case Output::out11:
        regData.bits.out11 = value;
        break;
    case Output::out12:
        regData.bits.out12 = value;
        break;
    case Output::out13:
        regData.bits.out13 = value;
        break;
    case Output::out14:
        regData.bits.out14 = value;
        break;
    case Output::out15:
        regData.bits.out15 = value;
        break;
    }
    // updateOutputs();   // Sofort die Änderungen anwenden
}

void RegisterOut::updateOutputs() {
    digitalWrite(latchPin, LOW);
    shiftOut(dataPin, clockPin, MSBFIRST, regData.bytes.secondByte);
    shiftOut(dataPin, clockPin, MSBFIRST, regData.bytes.firstByte);
    digitalWrite(latchPin, HIGH);
}

void RegisterOut::init() {
    xTaskCreatePinnedToCore(RegisterOut::outputUpdateTask,   // Task-Funktion
                            "OutputUpdateTask",              // Name des Tasks
                            2048,                            // Stackgröße
                            this,                            // Parameter für die Task-Funktion
                            1,                               // Priorität
                            NULL,                            // Task-Handle
                            1                                // Core-ID
    );
}

void RegisterOut::outputUpdateTask(void *pvParameters) {
    RegisterOut *registerInstance = static_cast<RegisterOut *>(pvParameters);
    for (;;) {
        registerInstance->updateOutputs();
        vTaskDelay(pdMS_TO_TICKS(5));   // Warte 5 Millisekunden
    }
}