#include <includes.h>

ADCManager::ADCManager() {}

void ADCManager::init() {

    xTaskCreatePinnedToCore(readInputsTask,    // Task-Funktion
                            "ReadADCInputs",   // Name des Tasks
                            10000,             // Stackgröße
                            this,              // Parameter für die Task-Funktion
                            1,                 // Priorität
                            NULL,              // Task-Handle
                            DEFAULT_CPU        // Core-ID
    );
}

void ADCManager::readInputsTask(void *pvParameters) {
    ADCManager *adcManager = static_cast<ADCManager *>(pvParameters);
    for (;;) {
        adcManager->joystickX     = analogRead(JOYSTICK_X);
        adcManager->joystickY     = analogRead(JOYSTICK_Y);
        adcManager->joystickZ     = analogRead(JOYSTICK_Z);
        adcManager->feedrate      = analogRead(FEEDRATE);
        adcManager->rotationSpeed = analogRead(ROTATION_SPEED);

        // Serial.print("Joystick X: " + String(adcManager->joystickX) + " \t");
        // Serial.print("Joystick Y: " + String(adcManager->joystickY) + " \t");
        // Serial.print("Joystick Z: " + String(adcManager->joystickZ) + " \t");
        // Serial.print("Feedrate: " + String(adcManager->feedrate) + " \t");
        // Serial.print("Rotation Speed: " + String(adcManager->rotationSpeed) + " \n");

        vTaskDelay(pdMS_TO_TICKS(5));
    }
}

int16_t ADCManager::readJoystickX() { return analogRead(JOYSTICK_X); }
int16_t ADCManager::readJoystickY() { return analogRead(JOYSTICK_Y); }
int16_t ADCManager::readJoystickZ() { return analogRead(JOYSTICK_Z); }
int16_t ADCManager::readFeedrate() { return analogRead(FEEDRATE); }
int16_t ADCManager::readRotationSpeed() { return analogRead(ROTATION_SPEED); }

int16_t ADCManager::getJoystickX() const { return joystickX; }
int16_t ADCManager::getJoystickY() const { return joystickY; }
int16_t ADCManager::getJoystickZ() const { return joystickZ; }
int16_t ADCManager::getFeedrate() const { return feedrate; }
int16_t ADCManager::getRotationSpeed() const { return rotationSpeed; }