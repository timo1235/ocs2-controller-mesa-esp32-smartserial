#include "sserial_internal.h"
#include <includes.h>
#include <string.h>

void processDataInputs() {
    // Thread-safe snapshot of digital inputs from Core 0
    InputSnapshot inputs = ioRegister.getInputSnapshot();

    data_in.in1           = inputs.mcp0.bits.in1;
    data_in.in2           = inputs.mcp0.bits.in2;
    data_in.in3           = inputs.mcp0.bits.in3;
    data_in.in4           = inputs.mcp0.bits.in4;
    data_in.in5           = inputs.mcp0.bits.in5;
    data_in.in6           = inputs.mcp0.bits.in6;
    data_in.in7           = inputs.mcp0.bits.in7;
    data_in.in8           = inputs.mcp0.bits.in8;
    data_in.in9           = inputs.mcp0.bits.in9;
    data_in.in10          = inputs.mcp0.bits.in10;
    data_in.in11          = inputs.mcp0.bits.in11;
    data_in.in12          = inputs.mcp0.bits.in12;
    data_in.in13          = inputs.mcp0.bits.in13;
    data_in.in14          = inputs.mcp0.bits.in14;
    data_in.in15          = inputs.mcp0.bits.in15;
    data_in.in16          = inputs.mcp0.bits.in16;
    data_in.ok            = inputs.mcp1.bits.OK;
    data_in.motorstart    = inputs.mcp1.bits.motorStart;
    data_in.programmstart = inputs.mcp1.bits.programmStart;
    data_in.auswahlx      = inputs.mcp1.bits.auswahlX;
    data_in.auswahly      = inputs.mcp1.bits.auswahlY;
    data_in.auswahlz      = inputs.mcp1.bits.auswahlZ;
    data_in.speed1        = inputs.mcp1.bits.speed1;
    data_in.speed2        = inputs.mcp1.bits.speed2;
    data_in.alarm         = inputs.mcp1.bits.alarmAll;

    // Thread-safe snapshot of analog inputs from Core 0
    ADCSnapshot adc = adcManager.getSnapshot();
    data_in.joy_x    = (int8_t)((adc.joystickX >> 4) - 128);
    data_in.joy_y    = (int8_t)((adc.joystickY >> 4) - 128);
    data_in.joy_z    = (int8_t)((adc.joystickZ >> 4) - 128);
    data_in.feedrate = (uint8_t)(adc.feedrate >> 4);
    data_in.rotation = (uint8_t)(adc.rotationSpeed >> 4);
}

void processIncomingData() {
    bool read_error = false;
    for (int i = 0; i < discovery.output; i++) {
        int byte = Serial1.read();
        if (byte < 0) {
            read_error = true;
            break;
        }
        ((uint8_t *) (&data_out))[i] = (uint8_t) byte;
    }

    // Consume host CRC byte (LBP spec: CRC appended to all commands)
    Serial1.read();

    txbuf[0] = read_error ? 0x01 : 0x00;
    for (int i = 0; i < (discovery.input - 1); i++) {
        txbuf[i + 1] = ((uint8_t *) (&data_in))[i];
    }
    send(discovery.input, 1);
}

void updateOutputPins() {
    uint16_t bits = 0;
    if (data_out.out1)    bits |= (1 << OUT1);
    if (data_out.out2)    bits |= (1 << OUT2);
    if (data_out.out3)    bits |= (1 << OUT3);
    if (data_out.out4)    bits |= (1 << OUT4);
    if (data_out.out5)    bits |= (1 << OUT5);
    if (data_out.out6)    bits |= (1 << OUT6);
    if (data_out.out7)    bits |= (1 << OUT7);
    if (data_out.out8)    bits |= (1 << OUT8);
    if (data_out.ena)     bits |= (1 << ENA);
    if (data_out.spindel) bits |= (1 << SPINDEL_ON_OFF);
    ioRegister.setAllOutputs(bits);
}

void safeState() {
    ioRegister.setAllOutputs(0);
    memset(&data_out, 0, sizeof(data_out));
}
