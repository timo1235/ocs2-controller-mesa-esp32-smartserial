#include "sserial_internal.h"
#include <string.h>

void handleLocalRead(uint8_t available) {
    if (available >= 2) {
        emptySerialBuffer();
        switch (lbp.byte) {
        case LBPCookieCMD: // 0xDF
            txbuf[0] = LBPCookie;
            break;
        case LBPStatusCMD: // 0xC1
            txbuf[0] = 0x00;
            break;
        case LBPCardName0Cmd ... LBPCardName3Cmd: // 0xD0..0xD3
            txbuf[0] = name[lbp.byte - LBPCardName0Cmd];
            break;
        case 0xC0: // Unit address
            txbuf[0] = 0x00;
            break;
        case 0xC2: // CRC enable status (always enabled)
            txbuf[0] = 0x01;
            break;
        case 0xC3: // CRC error count
            txbuf[0] = 0x00;
            break;
        case 0xD8: // Get low address byte
            txbuf[0] = address & 0xFF;
            break;
        case 0xD9: // Get high address byte
            txbuf[0] = (address >> 8) & 0xFF;
            break;
        case 0xDA: // LBP version
            txbuf[0] = 0x02;
            break;
        default:
            txbuf[0] = 0x00;
        }
        send(1, 1);
    }
}

void handleLocalWrite(uint8_t available) {
    if (lbp.byte == 0xFF) {
        // Reset LBP parser - no data byte follows, only CRC
        emptySerialBuffer();
        // No response per spec (parser is being reset)
        return;
    }

    // All other local writes: [cmd] [data_byte] [CRC] = 3 bytes total
    if (available < 3) return;

    uint8_t data = Serial1.read();
    emptySerialBuffer(); // consume CRC

    switch (lbp.byte) {
    case 0xE1: // Set LBP status (0 to clear errors)
        break;
    case 0xE2: // Set CRC check enable (always enabled, accept silently)
        break;
    case 0xF8: // Set low address byte
        address = (address & 0xFF00) | data;
        break;
    case 0xF9: // Set high address byte
        address = (address & 0x00FF) | ((uint16_t) data << 8);
        break;
    case 0xFA: // Add byte to current address
        address += data;
        break;
    case 0xFE: // Reset LBP processor if data == 0x5A
        if (data == 0x5A) {
            ESP.restart();
        }
        break;
    default:
        break;
    }

    // LBP spec: writes return CRC=0x00
    send(0, 1);
}

void handleRpc(uint8_t available) {
    if (lbp.byte == UnitNumberRPC && available >= 2) {
        emptySerialBuffer();
        txbuf[0] = unit.byte[0];
        txbuf[1] = unit.byte[1];
        txbuf[2] = unit.byte[2];
        txbuf[3] = unit.byte[3];
        send(4, 1);
    } else if (lbp.byte == DiscoveryRPC && available >= 2) {
        emptySerialBuffer();
        memcpy((void *) txbuf, ((uint8_t *) &discovery), sizeof(discovery));
        send(sizeof(discovery), 1);
    } else if (lbp.byte == ProcessDataRPC && available >= discovery.output + 2) {
        processDataInputs();
        processIncomingData();
        updateOutputPins();
    }
}

void handleRead(uint8_t available) {
    int size = 2 * lbp.as + 2;
    if (available >= size) {
        if (lbp.as) {
            const uint8_t first  = Serial1.read();
            const uint8_t second = Serial1.read();
            address              = first + (second << 8);
        }
        emptySerialBuffer();
        if ((address + (1 << lbp.ds)) <= sserial_slave_size) {
            memcpy((void *) txbuf, &sserial_slave[address], (1 << lbp.ds));
            send((1 << lbp.ds), 1);
        }
        if (lbp.ai) {
            address += (1 << lbp.ds);
        }
    }
}

void handleWrite(uint8_t available) {
    int size = 2 * lbp.as + (1 << lbp.ds) + 2;
    if (available >= size) {
        if (lbp.as) {
            const uint8_t first  = Serial1.read();
            const uint8_t second = Serial1.read();
            address              = first + (second << 8);
        }
        if ((address + (1 << lbp.ds)) <= sserial_slave_size) {
            for (int i = 0; i < (1 << lbp.ds); i++) {
                sserial_slave[address + i] = Serial1.read();
            }
        } else {
            // Consume data bytes even if out of bounds to keep protocol in sync
            for (int i = 0; i < (1 << lbp.ds); i++) {
                Serial1.read();
            }
        }
        // Consume host CRC byte
        Serial1.read();
        if (lbp.ai) {
            address += (1 << lbp.ds);
        }
        // LBP spec: writes return CRC=0x00 (CRC of empty response)
        send(0, 1);
    }
}
