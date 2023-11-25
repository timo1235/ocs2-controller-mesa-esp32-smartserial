#include <includes.h>

RegisterOut outRegister(REGISTER_OUT_DATA, REGISTER_OUT_CLK, REGISTER_OUT_LATCH);
RegisterIn  inRegister(REGISTER_IN_DATA, REGISTER_IN_CLK, REGISTER_IN_LATCH);
ADCManager  adcManager;
Debug       debug;

void setup() {
    // pinMode(I2C_OCS2_SDA, INPUT);
    // pinMode(I2C_OCS2_SCL, INPUT);
    // Serial for debugging - too much output interferes with the smart serial communication
    // since it needs to be very fast and reliable. So, use debug output wisely.
    Serial.begin(115200);

    // Start serial instance for smart serial communication
    // Baudrate is 2.5MBit/s
    Serial1.begin(2500000, SERIAL_8N1, SSERIAL_RXD, SSERIAL_TXD);

    debug.init();

    // inRegister.init();
    // outRegister.init();

    delay(5000);

    // adcManager.init();
    sserial_init();

    // outRegister.set(RegisterOut::Output::ena, true);
}

void loop() {}
