#include <includes.h>

MCP23S17 MCP0(15, 0, &SPI);
MCP23S17 MCP1(15, 1, &SPI);
MCP23S17 MCP2(15, 2, &SPI);

volatile bool IORegister::mcp0InterruptFlag = true;
volatile bool IORegister::mcp1InterruptFlag = true;
uint16_t      IORegister::outputStatus      = 0x0000;

IORegister::IORegister() : localOutputStatus(0x0000) {}

void IORegister::updateInputs() {
    if (mcp0InterruptFlag) {
        Serial.println("MCP0 Interrupt");
        mcp0InterruptFlag  = false;
        mcp0Data.inputData = MCP0.read16();
    }
    if (mcp1InterruptFlag) {
        Serial.println("MCP1 Interrupt");
        mcp1InterruptFlag  = false;
        mcp1Data.inputData = MCP1.read16();
    }
}

void IORegister::setOutput(OutputPin pin, uint8_t value) {
    uint16_t mask = 1 << pin;   // Shift the bit to the correct position
    if (value == HIGH) {
        localOutputStatus |= mask;
    } else {
        localOutputStatus &= ~mask;
    }
}

void IORegister::init() {
    attachInterrupt(digitalPinToInterrupt(MCP0_INTA), IORegister::handleInterruptMCP0, FALLING);
    attachInterrupt(digitalPinToInterrupt(MCP0_INTB), IORegister::handleInterruptMCP0, FALLING);
    attachInterrupt(digitalPinToInterrupt(MCP1_INTA), IORegister::handleInterruptMCP1, FALLING);
    attachInterrupt(digitalPinToInterrupt(MCP1_INTB), IORegister::handleInterruptMCP1, FALLING);

    SPI.begin(SPI_SCK, SPI_MISO, SPI_MOSI);

    MCP0.begin();
    MCP1.begin();
    MCP2.begin();

    MCP0.enableHardwareAddress();
    MCP1.enableHardwareAddress();
    MCP2.enableHardwareAddress();

    MCP0.pinMode16(0xFFFF);
    MCP1.pinMode16(0xFFFF);
    // Pins 1-10 = Output, Pins 11-16 = Input
    MCP2.pinMode16(0b0000000011111100);

    // Set all outputs off
    MCP2.write16(0x0000);

    MCP0.enableInterrupt16(0xFFFF, CHANGE);
    MCP1.enableInterrupt16(0xFFFF, CHANGE);
    // MCP1.enableInterrupt(0, CHANGE);
    // MCP1.enableInterrupt(1, CHANGE);

    outputStatus = 0x0000;   // Initialize the output status
    xTaskCreatePinnedToCore(IORegister::outputUpdateTask, "OutputUpdateTask", 4096, this, 1, NULL, DEFAULT_CPU);
    xTaskCreatePinnedToCore(IORegister::inputUpdateTask, "InputUpdateTask", 4096, this, 1, NULL, DEFAULT_CPU);
}

void IORegister::inputUpdateTask(void *pvParameters) {
    IORegister *instance = static_cast<IORegister *>(pvParameters);
    for (;;) {
        vTaskDelay(pdMS_TO_TICKS(1));
        instance->updateInputs();
    }
}

void IORegister::outputUpdateTask(void *pvParameters) {
    IORegister *instance = static_cast<IORegister *>(pvParameters);
    while (true) {
        vTaskDelay(pdMS_TO_TICKS(1));
        if (instance->localOutputStatus != IORegister::outputStatus) {
            IORegister::outputStatus = instance->localOutputStatus;
            MCP2.write16(IORegister::outputStatus);
        }

        // Bits to Ports when sending 16bit like 0b1111111111111111 it sets these pins in this order:
        // GPA7, GPA6, GPA5, GPA4, GPA3, GPA2, GPA1, GPA0, GPB7, GPB6, GPB5, GPB4, GPB3, GPB2, GPB1, GPB0

        // MCP2.write16(0b11111111 11111111);
        // vTaskDelay(pdMS_TO_TICKS(200));
        // MCP2.write16(0b0000000000000000);
    }
}

void IRAM_ATTR IORegister::handleInterruptMCP0() { mcp0InterruptFlag = true; }

void IRAM_ATTR IORegister::handleInterruptMCP1() { mcp1InterruptFlag = true; }