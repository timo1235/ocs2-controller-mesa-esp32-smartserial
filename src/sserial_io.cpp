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
    data_in.joy_x    = map(adc.joystickX, 0, 4095, -127, 127);
    data_in.joy_y    = map(adc.joystickY, 0, 4095, -127, 127);
    data_in.joy_z    = map(adc.joystickZ, 0, 4095, -127, 127);
    data_in.feedrate = map(adc.feedrate, 0, 4095, 0, 255);
    data_in.rotation = map(adc.rotationSpeed, 0, 4095, 0, 255);
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

    txbuf[0] = read_error ? 0x01 : 0x00;
    for (int i = 0; i < (discovery.input - 1); i++) {
        txbuf[i + 1] = ((uint8_t *) (&data_in))[i];
    }
    send(discovery.input, 1);
}

void updateOutputPins() {
    ioRegister.setOutput(OutputPin::ENA, data_out.ena);
    ioRegister.setOutput(OutputPin::OUT1, data_out.out1);
    ioRegister.setOutput(OutputPin::OUT2, data_out.out2);
    ioRegister.setOutput(OutputPin::OUT3, data_out.out3);
    ioRegister.setOutput(OutputPin::OUT4, data_out.out4);
    ioRegister.setOutput(OutputPin::OUT5, data_out.out5);
    ioRegister.setOutput(OutputPin::OUT6, data_out.out6);
    ioRegister.setOutput(OutputPin::OUT7, data_out.out7);
    ioRegister.setOutput(OutputPin::OUT8, data_out.out8);
    ioRegister.setOutput(OutputPin::SPINDEL_ON_OFF, data_out.spindel);
}

void safeState() {
    ioRegister.setOutput(OutputPin::ENA, LOW);
    ioRegister.setOutput(OutputPin::SPINDEL_ON_OFF, LOW);
    ioRegister.setOutput(OutputPin::OUT1, LOW);
    ioRegister.setOutput(OutputPin::OUT2, LOW);
    ioRegister.setOutput(OutputPin::OUT3, LOW);
    ioRegister.setOutput(OutputPin::OUT4, LOW);
    ioRegister.setOutput(OutputPin::OUT5, LOW);
    ioRegister.setOutput(OutputPin::OUT6, LOW);
    ioRegister.setOutput(OutputPin::OUT7, LOW);
    ioRegister.setOutput(OutputPin::OUT8, LOW);
    memset(&data_out, 0, sizeof(data_out));
}
