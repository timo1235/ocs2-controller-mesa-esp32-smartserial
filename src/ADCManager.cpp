#include <includes.h>

ADCManager::ADCManager() {}

void ADCManager::init() {
    Wire.setPins(I2C_SDA, I2C_SCL);

    adc1.setGain(GAIN_TWOTHIRDS);
    adc1.begin(ADS1_ADDRESS, &Wire);
    adc1.setDataRate(RATE_ADS1115_250SPS);
    delay(10);
    adc2.setGain(GAIN_TWOTHIRDS);
    adc2.begin(ADS2_ADDRESS, &Wire);
    adc2.setDataRate(RATE_ADS1115_250SPS);
    Wire.setClock(3400000);

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
        adcManager->joystickX     = map(adcManager->readJoystickX(), -100, 27000, 0, 4095);
        adcManager->joystickY     = map(adcManager->readJoystickY(), -100, 27000, 0, 4095);
        adcManager->joystickZ     = map(adcManager->readJoystickZ(), -100, 27000, 0, 4095);
        adcManager->feedrate      = map(adcManager->readFeedrate(), -100, 27000, 0, 4095);
        adcManager->rotationSpeed = map(adcManager->readRotationSpeed(), -100, 27000, 0, 4095);

        vTaskDelay(pdMS_TO_TICKS(50));
    }
}

int16_t ADCManager::readJoystickX() { return adc1.readADC_SingleEnded(0); }
int16_t ADCManager::readJoystickY() { return adc1.readADC_SingleEnded(1); }
int16_t ADCManager::readJoystickZ() { return adc1.readADC_SingleEnded(2); }
int16_t ADCManager::readFeedrate() { return adc2.readADC_SingleEnded(0); }
int16_t ADCManager::readRotationSpeed() { return adc2.readADC_SingleEnded(1); }

int16_t ADCManager::getJoystickX() const { return joystickX; }
int16_t ADCManager::getJoystickY() const { return joystickY; }
int16_t ADCManager::getJoystickZ() const { return joystickZ; }
int16_t ADCManager::getFeedrate() const { return feedrate; }
int16_t ADCManager::getRotationSpeed() const { return rotationSpeed; }