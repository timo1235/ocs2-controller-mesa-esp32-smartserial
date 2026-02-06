#include <includes.h>
#include <MCP23S17.h>
#include <SPI.h>

MCP23S17 MCP0(15, 0, &SPI);
MCP23S17 MCP1(15, 1, &SPI);
MCP23S17 MCP2(15, 2, &SPI);

volatile bool IORegister::mcp0InterruptFlag = true;
volatile bool IORegister::mcp1InterruptFlag = true;

IORegister::IORegister()
    : _inputSnapshot{}, _inputMux(portMUX_INITIALIZER_UNLOCKED), _pendingOutput(0), _currentOutput(0),
      _outputMux(portMUX_INITIALIZER_UNLOCKED) {}

void IORegister::updateInputs() {
    bool readMcp0 = false;
    bool readMcp1 = false;
    uint16_t mcp0Raw = 0;
    uint16_t mcp1Raw = 0;

    if (mcp0InterruptFlag) {
        mcp0InterruptFlag = false;
        mcp0Raw = MCP0.read16();
        readMcp0 = true;
    }
    if (mcp1InterruptFlag) {
        mcp1InterruptFlag = false;
        mcp1Raw = MCP1.read16();
        readMcp1 = true;
    }

    if (readMcp0 || readMcp1) {
        portENTER_CRITICAL(&_inputMux);
        if (readMcp0) _inputSnapshot.mcp0.raw = mcp0Raw;
        if (readMcp1) _inputSnapshot.mcp1.raw = mcp1Raw;
        portEXIT_CRITICAL(&_inputMux);
    }
}

InputSnapshot IORegister::getInputSnapshot() {
    InputSnapshot snapshot;
    portENTER_CRITICAL(&_inputMux);
    snapshot = _inputSnapshot;
    portEXIT_CRITICAL(&_inputMux);
    return snapshot;
}

void IORegister::setOutput(OutputPin pin, uint8_t value) {
    portENTER_CRITICAL(&_outputMux);
    uint16_t mask = 1 << pin;
    if (value == HIGH) {
        _pendingOutput |= mask;
    } else {
        _pendingOutput &= ~mask;
    }
    portEXIT_CRITICAL(&_outputMux);
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
    for (;;) {
        vTaskDelay(pdMS_TO_TICKS(1));

        uint16_t pending;
        portENTER_CRITICAL(&instance->_outputMux);
        pending = instance->_pendingOutput;
        portEXIT_CRITICAL(&instance->_outputMux);

        if (pending != instance->_currentOutput) {
            instance->_currentOutput = pending;
            MCP2.write16(instance->_currentOutput);
        }
    }
}

void IRAM_ATTR IORegister::handleInterruptMCP0() { mcp0InterruptFlag = true; }

void IRAM_ATTR IORegister::handleInterruptMCP1() { mcp1InterruptFlag = true; }
