#include <includes.h>

ADCManager::ADCManager() : _snapshot{0, 0, 0, 0, 0}, _adcMux(portMUX_INITIALIZER_UNLOCKED) {}

void ADCManager::init() {
    xTaskCreatePinnedToCore(readInputsTask, "ReadADCInputs", 4096, this, 1, NULL, DEFAULT_CPU);
}

void ADCManager::readInputsTask(void *pvParameters) {
    ADCManager *self = static_cast<ADCManager *>(pvParameters);
    for (;;) {
        // Read all ADC channels outside the lock (slow I/O)
        int16_t jx = analogRead(JOYSTICK_X);
        int16_t jy = analogRead(JOYSTICK_Y);
        int16_t jz = analogRead(JOYSTICK_Z);
        int16_t fr = analogRead(FEEDRATE);
        int16_t rs = analogRead(ROTATION_SPEED);

        // Update snapshot atomically
        portENTER_CRITICAL(&self->_adcMux);
        self->_snapshot.joystickX     = jx;
        self->_snapshot.joystickY     = jy;
        self->_snapshot.joystickZ     = jz;
        self->_snapshot.feedrate      = fr;
        self->_snapshot.rotationSpeed = rs;
        portEXIT_CRITICAL(&self->_adcMux);

        vTaskDelay(pdMS_TO_TICKS(5));
    }
}

ADCSnapshot ADCManager::getSnapshot() {
    ADCSnapshot snap;
    portENTER_CRITICAL(&_adcMux);
    snap = _snapshot;
    portEXIT_CRITICAL(&_adcMux);
    return snap;
}
