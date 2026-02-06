#include "sserial_internal.h"
#include "crc8.h"
#include <string.h>

void handleLocalRead() {
    if (!waitForBytes(1)) return;  // CRC
    uint8_t received_crc = Serial1.read();

    crc8_t crc = crc8_init();
    crc = crc8_update(crc, &lbp.byte, 1);
    if (crc8_finalize(crc) != received_crc) { crc_error_count++; return; }

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
        txbuf[0] = crc_error_count;
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

void handleLocalWrite() {
    if (lbp.byte == 0xFF) {
        // Reset LBP parser - no data byte follows, only CRC
        if (!waitForBytes(1)) return;
        uint8_t received_crc = Serial1.read();

        crc8_t crc = crc8_init();
        crc = crc8_update(crc, &lbp.byte, 1);
        if (crc8_finalize(crc) != received_crc) { crc_error_count++; return; }
        // No response per spec (parser is being reset)
        return;
    }

    // All other local writes: [data_byte] [CRC] = 2 bytes remaining
    if (!waitForBytes(2)) return;

    uint8_t data = Serial1.read();
    uint8_t received_crc = Serial1.read();

    crc8_t crc = crc8_init();
    crc = crc8_update(crc, &lbp.byte, 1);
    crc = crc8_update(crc, &data, 1);
    if (crc8_finalize(crc) != received_crc) { crc_error_count++; return; }

    switch (lbp.byte) {
    case 0xE1: // Set LBP status (0 to clear errors)
        crc_error_count = 0;
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

void handleRpc() {
    if (lbp.byte == UnitNumberRPC) {
        if (!waitForBytes(1)) return;  // CRC
        uint8_t received_crc = Serial1.read();

        crc8_t crc = crc8_init();
        crc = crc8_update(crc, &lbp.byte, 1);
        if (crc8_finalize(crc) != received_crc) { crc_error_count++; return; }

        txbuf[0] = unit.byte[0];
        txbuf[1] = unit.byte[1];
        txbuf[2] = unit.byte[2];
        txbuf[3] = unit.byte[3];
        send(4, 1);
    } else if (lbp.byte == DiscoveryRPC) {
        if (!waitForBytes(1)) return;  // CRC
        uint8_t received_crc = Serial1.read();

        crc8_t crc = crc8_init();
        crc = crc8_update(crc, &lbp.byte, 1);
        if (crc8_finalize(crc) != received_crc) { crc_error_count++; return; }

        memcpy((void *) txbuf, ((uint8_t *) &discovery), sizeof(discovery));
        send(sizeof(discovery), 1);
    } else if (lbp.byte == ProcessDataRPC) {
        if (!waitForBytes(discovery.output + 1)) return;  // output data + CRC
        processDataInputs();
        if (processIncomingData()) {
            updateOutputPins();
        }
    }
}

void handleRead() {
    int remaining = 2 * lbp.as + 1;  // optional address + CRC
    if (!waitForBytes(remaining)) return;

    crc8_t crc = crc8_init();
    crc = crc8_update(crc, &lbp.byte, 1);

    if (lbp.as) {
        uint8_t addr_bytes[2];
        addr_bytes[0] = Serial1.read();
        addr_bytes[1] = Serial1.read();
        crc = crc8_update(crc, addr_bytes, 2);
        address = addr_bytes[0] + (addr_bytes[1] << 8);
    }
    uint8_t received_crc = Serial1.read();
    if (crc8_finalize(crc) != received_crc) { crc_error_count++; return; }

    if ((address + (1 << lbp.ds)) <= sserial_slave_size) {
        memcpy((void *) txbuf, &sserial_slave[address], (1 << lbp.ds));
    } else {
        memset((void *) txbuf, 0, (1 << lbp.ds));
    }
    send((1 << lbp.ds), 1);
    if (lbp.ai) {
        address += (1 << lbp.ds);
    }
}

void handleWrite() {
    int remaining = 2 * lbp.as + (1 << lbp.ds) + 1;  // optional address + data + CRC
    if (!waitForBytes(remaining)) return;

    crc8_t crc = crc8_init();
    crc = crc8_update(crc, &lbp.byte, 1);

    if (lbp.as) {
        uint8_t addr_bytes[2];
        addr_bytes[0] = Serial1.read();
        addr_bytes[1] = Serial1.read();
        crc = crc8_update(crc, addr_bytes, 2);
        address = addr_bytes[0] + (addr_bytes[1] << 8);
    }

    uint8_t data_bytes[8];
    int data_len = (1 << lbp.ds);
    for (int i = 0; i < data_len; i++) {
        data_bytes[i] = Serial1.read();
    }
    crc = crc8_update(crc, data_bytes, data_len);

    uint8_t received_crc = Serial1.read();
    if (crc8_finalize(crc) != received_crc) { crc_error_count++; return; }

    if ((address + data_len) <= sserial_slave_size) {
        memcpy(&sserial_slave[address], data_bytes, data_len);
    }
    if (lbp.ai) {
        address += data_len;
    }
    // LBP spec: writes return CRC=0x00 (CRC of empty response)
    send(0, 1);
}
