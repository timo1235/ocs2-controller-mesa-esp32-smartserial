#include <includes.h>

IORegister ioRegister;
ADCManager adcManager;
Debug      debug;
bool       sserial_timeoutFlag = true;

void setup() {
    pinMode(I2C_OCS2_SDA, INPUT);
    pinMode(I2C_OCS2_SCL, INPUT);
    // Serial for debugging - too much output interferes with the smart serial communication
    // since it needs to be very fast and reliable. So, use debug output wisely.
    Serial.begin(115200);

    // Start serial instance for smart serial communication
    // Baudrate is 2.5MBit/s
    Serial1.begin(2500000, SERIAL_8N1, SSERIAL_RXD, SSERIAL_TXD);

    debug.init();

    ioRegister.init();
    adcManager.init();

    delay(1000);

    sserial_init();
}

void loop() {
    // ioRegister.setOutput(OutputPin::SPINDEL_ON_OFF, HIGH);
    // delay(1000);
    // ioRegister.setOutput(OutputPin::SPINDEL_ON_OFF, LOW);
    // delay(1000);
}
