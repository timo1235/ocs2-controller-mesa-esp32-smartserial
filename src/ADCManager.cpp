#include <includes.h>

ADCManager::ADCManager() {}

void ADCManager::init() {
    Serial.println("Initializing ADCs...");
    // Wire.setPins(I2C_SDA, I2C_SCL);
    // Wire.setClock(1000000);

    // adc1.setGain(GAIN_TWOTHIRDS);
    // adc1.begin(ADS1_ADDRESS, &Wire);
    // adc1.setDataRate(RATE_ADS1115_250SPS);

    // adc2.setGain(GAIN_TWOTHIRDS);
    // adc2.begin(ADS2_ADDRESS, &Wire);
    // adc2.setDataRate(RATE_ADS1115_250SPS);

    xTaskCreatePinnedToCore(readInputsTask,    // Task-Funktion
                            "ReadADCInputs",   // Name des Tasks
                            10000,             // Stackgröße
                            this,              // Parameter für die Task-Funktion
                            1,                 // Priorität
                            NULL,              // Task-Handle
                            DEFAULT_CPU        // Core-ID
    );
}

uint32_t counter = 0;

void ADCManager::readInputsTask(void *pvParameters) {
    ADCManager *adcManager = static_cast<ADCManager *>(pvParameters);
    for (;;) {
        counter++;
        adcManager->joystickX     = random(0, 26000);
        adcManager->joystickY     = random(0, 26000);
        adcManager->joystickZ     = random(0, 26000);
        adcManager->feedrate      = random(0, 26000);
        adcManager->rotationSpeed = random(0, 26000);
        // adcManager->joystickX     = adcManager->readJoystickX();
        // adcManager->joystickY     = adcManager->readJoystickY();
        // adcManager->joystickZ     = adcManager->readJoystickZ();
        // adcManager->feedrate      = adcManager->readFeedrate();
        // adcManager->rotationSpeed = adcManager->readRotationSpeed();
        // if (sserial_timeoutFlag) {
        debug.print("Joystick X: %d", map(adcManager->joystickX, 0, 26000, -127, 127));
        debug.print("Counter: %d", counter);
        // }

        vTaskDelay(pdMS_TO_TICKS(10000));
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