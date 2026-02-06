#include "sserial_internal.h"
#include <string.h>

void handleLocalRead(uint8_t available) {
    if (available >= 2) {
        emptySerialBuffer();
        switch (lbp.byte) {
        case LBPCookieCMD:
            txbuf[0] = LBPCookie;
            break;
        case LBPStatusCMD:
            txbuf[0] = 0x00;
            break;
        case LBPCardName0Cmd ... LBPCardName3Cmd:
            txbuf[0] = name[lbp.byte - LBPCardName0Cmd];
            break;
        default:
            txbuf[0] = 0x00;
        }
        send(1, 1);
    }
}

void handleLocalWrite(uint8_t available) {
    emptySerialBuffer();
    if (lbp.byte == 0xFF || lbp.byte == 0xFC) {
        // Reset (0xFF) or Stop (0xFC) — no response needed
    } else if (available >= 3) {
        txbuf[0] = 0x00;
        send(1, 0);
    }
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
        if (lbp.ai) {
            address += (1 << lbp.ds);
        }
    }
}
