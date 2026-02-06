#include <includes.h>
#include <sserial.h>

IORegister ioRegister;
ADCManager adcManager;
Debug      debug;
bool       sserial_timeoutFlag = true;

#define DEBUG_BAUD   115200
#define SSERIAL_BAUD 2500000

void setup() {
    pinMode(I2C_OCS2_SDA, INPUT);
    pinMode(I2C_OCS2_SCL, INPUT);

    Serial.begin(DEBUG_BAUD);
    Serial1.begin(SSERIAL_BAUD, SERIAL_8N1, SSERIAL_RXD, SSERIAL_TXD);

    debug.init();
    ioRegister.init();
    adcManager.init();

    delay(1000);

    sserial_init();
}

void loop() {}
