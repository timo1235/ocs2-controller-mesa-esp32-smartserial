#include <includes.h>

RegisterOut outRegister(REGISTER_OUT_DATA, REGISTER_OUT_CLK, REGISTER_OUT_LATCH);
RegisterIn  inRegister(REGISTER_IN_DATA, REGISTER_IN_CLK, REGISTER_IN_LATCH);

void setup() {
    // Serial for debugging - too much output interferes with the smart serial communication
    // since it needs to be very fast and reliable. So, use debug output wisely.
    Serial.begin(115200);

    // Start serial instance for smart serial communication
    // Baudrate is 2.5MBit/s
    Serial1.begin(2500000, SERIAL_8N1, SSERIAL_RXD, SSERIAL_TXD);

    inRegister.init();
    outRegister.init();

    delay(5000);

    // sserial_init();

    // outRegister.set(RegisterOut::Output::ena, true);
}

void loop() {
    // Read inputs
    digitalWrite(REGISTER_IN_LATCH, LOW);
    delayMicroseconds(5);

    u_int8_t first  = shiftIn(REGISTER_IN_DATA, REGISTER_IN_CLK, MSBFIRST);
    u_int8_t second = shiftIn(REGISTER_IN_DATA, REGISTER_IN_CLK, MSBFIRST);
    u_int8_t third  = shiftIn(REGISTER_IN_DATA, REGISTER_IN_CLK, MSBFIRST);
    u_int8_t fourth = shiftIn(REGISTER_IN_DATA, REGISTER_IN_CLK, MSBFIRST);
    digitalWrite(REGISTER_IN_LATCH, HIGH);

    Serial.printf("First: %d, Second: %d, Third: %d, Fourth: %d\n", first, second, third, fourth);
    delay(1000);
}

void toggleOuts() {
    digitalWrite(REGISTER_OUT_LATCH, LOW);
    shiftOut(REGISTER_OUT_DATA, REGISTER_OUT_CLK, MSBFIRST, 0b00000000000000000000000000000000);
    digitalWrite(REGISTER_OUT_LATCH, HIGH);

    delay(1000);

    digitalWrite(REGISTER_OUT_LATCH, LOW);
    shiftOut(REGISTER_OUT_DATA, REGISTER_OUT_CLK, MSBFIRST, 0b11111111111111111111111111111111);
    digitalWrite(REGISTER_OUT_LATCH, HIGH);

    delay(1000);
}