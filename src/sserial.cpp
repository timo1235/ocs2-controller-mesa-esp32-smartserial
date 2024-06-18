#include "sserial.h"
#include "crc8.h"
#include <includes.h>
#include <string.h>

static volatile uint8_t rxbuf[128];
static volatile uint8_t txbuf[128];
static uint16_t         address;
static int              rxpos;
static uint32_t         timeout;
static lbp_t            lbp;
static const char       name[] = LBPCardName;
static unit_no_t        unit;
static uint32_t         max_waste_ticks;
static uint32_t         block_bytes;
static memory_t         memory;
static uint8_t         *heap_ptr;
static pd_table_t       pd_table;

uint8_t sserial_slave[] = {
    0x0A, 0x02, 0xE0, 0x00, 0xF4, 0x00, 0x00, 0x00,   // 0..7
    0x00, 0x00, 0x00, 0x00, 0xA0, 0x10, 0x01, 0x80,   // 8..15
    0x00, 0x00, 0xC8, 0xC2, 0x00, 0x00, 0xC8, 0x42,   // 16..23
    0x08, 0x00, 0x6E, 0x6F, 0x6E, 0x65, 0x00, 0x6F,   // 24..31
    0x75, 0x74, 0x00, 0x00, 0xA0, 0x08, 0x03, 0x00,   // 32..39
    0x00, 0x00, 0xFE, 0xC2, 0x00, 0x00, 0xFE, 0x42,   // 40..47
    0x23, 0x00, 0x6E, 0x6F, 0x6E, 0x65, 0x00, 0x6A,   // 48..55
    0x6F, 0x79, 0x5F, 0x78, 0x00, 0x00, 0x00, 0x00,   // 56..63
    0xA0, 0x08, 0x03, 0x00, 0x00, 0x00, 0xFE, 0xC2,   // 64..71
    0x00, 0x00, 0xFE, 0x42, 0x3D, 0x00, 0x6E, 0x6F,   // 72..79
    0x6E, 0x65, 0x00, 0x6A, 0x6F, 0x79, 0x5F, 0x79,   // 80..87
    0x00, 0x00, 0x00, 0x00, 0xA0, 0x08, 0x03, 0x00,   // 88..95
    0x00, 0x00, 0xFE, 0xC2, 0x00, 0x00, 0xFE, 0x42,   // 96..103
    0x59, 0x00, 0x6E, 0x6F, 0x6E, 0x65, 0x00, 0x6A,   // 104..111
    0x6F, 0x79, 0x5F, 0x7A, 0x00, 0x00, 0x00, 0x00,   // 112..119
    0xA0, 0x08, 0x02, 0x00, 0x00, 0x00, 0x00, 0x00,   // 120..127
    0x00, 0x00, 0x7F, 0x43, 0x75, 0x00, 0x6E, 0x6F,   // 128..135
    0x6E, 0x65, 0x00, 0x66, 0x65, 0x65, 0x64, 0x72,   // 136..143
    0x61, 0x74, 0x65, 0x00, 0x00, 0x00, 0x00, 0x00,   // 144..151
    0xA0, 0x08, 0x02, 0x00, 0x00, 0x00, 0x00, 0x00,   // 152..159
    0x00, 0x00, 0x7F, 0x43, 0x94, 0x00, 0x6E, 0x6F,   // 160..167
    0x6E, 0x65, 0x00, 0x72, 0x6F, 0x74, 0x61, 0x74,   // 168..175
    0x69, 0x6F, 0x6E, 0x00, 0x00, 0x00, 0x00, 0x00,   // 176..183
    0xA0, 0x20, 0x01, 0x00, 0x00, 0x00, 0xC8, 0xC2,   // 184..191
    0x00, 0x00, 0xC8, 0x42, 0xB4, 0x00, 0x6E, 0x6F,   // 192..199
    0x6E, 0x65, 0x00, 0x69, 0x6E, 0x00, 0xB0, 0x00,   // 200..207
    0x01, 0x00, 0x50, 0x6F, 0x73, 0x69, 0x74, 0x69,   // 208..215
    0x6F, 0x6E, 0x20, 0x6D, 0x6F, 0x64, 0x65, 0x00,   // 216..223
    0x0C, 0x00, 0x24, 0x00, 0x40, 0x00, 0x5C, 0x00,   // 224..231
    0x78, 0x00, 0x98, 0x00, 0xB8, 0x00, 0x00, 0x00,   // 232..239
    0x80, 0x1B, 0x00, 0x00, 0xCE, 0x00, 0x00, 0x00,   // 240..247

};

const discovery_rpc_t discovery = {
    .input  = 10,
    .output = 2,
    .ptocp  = 0x00E0,
    .gtocp  = 0x00F4,
};

typedef struct {
    uint32_t out_0 : 1;
    uint32_t out_1 : 1;
    uint32_t out_2 : 1;
    uint32_t out_3 : 1;
    uint32_t out_4 : 1;
    uint32_t out_5 : 1;
    uint32_t out_6 : 1;
    uint32_t out_7 : 1;
    uint32_t out_8 : 1;
    uint32_t out_9 : 1;
    uint32_t out_10 : 1;
    uint32_t out_11 : 1;
    uint32_t out_12 : 1;
    uint32_t out_13 : 1;
    uint32_t out_14 : 1;
    uint32_t out_15 : 1;
} sserial_out_process_data_t;

typedef struct {
    int8_t   joy_x;
    int8_t   joy_y;
    int8_t   joy_z;
    uint8_t  feedrate;
    uint8_t  rotation;
    uint32_t in_0 : 1;
    uint32_t in_1 : 1;
    uint32_t in_2 : 1;
    uint32_t in_3 : 1;
    uint32_t in_4 : 1;
    uint32_t in_5 : 1;
    uint32_t in_6 : 1;
    uint32_t in_7 : 1;
    uint32_t in_8 : 1;
    uint32_t in_9 : 1;
    uint32_t in_10 : 1;
    uint32_t in_11 : 1;
    uint32_t in_12 : 1;
    uint32_t in_13 : 1;
    uint32_t in_14 : 1;
    uint32_t in_15 : 1;
    uint32_t in_16 : 1;
    uint32_t in_17 : 1;
    uint32_t in_18 : 1;
    uint32_t in_19 : 1;
    uint32_t in_20 : 1;
    uint32_t in_21 : 1;
    uint32_t in_22 : 1;
    uint32_t in_23 : 1;
    uint32_t in_24 : 1;
    uint32_t in_25 : 1;
    uint32_t in_26 : 1;
    uint32_t in_27 : 1;
    uint32_t in_28 : 1;
    uint32_t in_29 : 1;
    uint32_t in_30 : 1;
    uint32_t in_31 : 1;
} sserial_in_process_data_t;
#define scale_address 300

static sserial_out_process_data_t data_out;
static sserial_in_process_data_t  data_in;

TaskHandle_t sserialTaskHandle;

static uint8_t crc_request(uint8_t len) {
    uint8_t crc = crc8_init();
    for (int i = rxpos; i < rxpos + len; i++) {
        crc = crc8_update(crc, (void *) &rxbuf[i % sizeof(rxbuf)], 1);
    }
    crc = crc8_finalize(crc);
    return crc == rxbuf[(rxpos + len) % sizeof(rxbuf)];
}

static uint8_t calculate_crc8(uint8_t *addr, uint8_t len) {
    uint8_t crc = crc8_init();
    crc         = crc8_update(crc, addr, len);
    return crc8_finalize(crc);
}

uint16_t add_pd(const char *name_string, const char *unit_string, uint8_t data_size_in_bits, uint8_t data_type, uint8_t data_dir,
                float param_min, float param_max) {
    process_data_descriptor_t pdr;
    pdr.record_type    = RECORD_TYPE_PROCESS_DATA_RECORD;
    pdr.data_size      = data_size_in_bits;
    pdr.data_type      = data_type;
    pdr.data_direction = data_dir;
    pdr.param_min      = param_min;
    pdr.param_max      = param_max;
    pdr.data_addr      = MEMPTR(*heap_ptr);

    heap_ptr += NUM_BYTES(data_size_in_bits);
    if ((uint32_t) heap_ptr % 4) {
        heap_ptr += 4 - (uint32_t) heap_ptr % 4;
    }

    memcpy(heap_ptr, &pdr, sizeof(process_data_descriptor_t));
    uint16_t pd_ptr = MEMPTR(*heap_ptr);

    heap_ptr = (uint8_t *) &(((process_data_descriptor_t *) heap_ptr)->names);

    strcpy((char *) heap_ptr, unit_string);
    heap_ptr += strlen(unit_string) + 1;

    strcpy((char *) heap_ptr, name_string);
    heap_ptr += strlen(name_string) + 1;

    return pd_ptr;
}

uint16_t add_mode(const char *name_string, uint8_t index, uint8_t type) {
    mode_descriptor_t mdr;
    mdr.record_type = RECORD_TYPE_MODE_DATA_RECORD;
    mdr.index       = index;
    mdr.type        = type;
    mdr.unused      = 0x00;

    memcpy(heap_ptr, &mdr, sizeof(mode_descriptor_t));
    uint16_t md_ptr = MEMPTR(*heap_ptr);

    heap_ptr = (uint8_t *) &(((mode_descriptor_t *) heap_ptr)->names);

    strcpy((char *) heap_ptr, name_string);
    heap_ptr += strlen(name_string) + 1;

    return md_ptr;
}

void metadata(pd_metadata_t *pdm, process_data_descriptor_t *ptr) {
    pdm->ptr    = ptr;
    pdm->range  = ptr->data_type == DATA_TYPE_SIGNED ? MAX(ABS(ptr->param_min), ABS(ptr->param_max)) * 2 : ptr->param_max;
    pdm->bitmax = (1 << ptr->data_size) - 1;
}

void print_pd(process_data_descriptor_t *pd) {
    int   strl = strlen(&pd->names);
    char *unit = &pd->names;
    char *name = &pd->names + strl + 1;
    switch (pd->data_type) {
    case DATA_TYPE_PAD:
        Serial.printf("uint32_t padding : %u;\n", pd->data_size);
        break;
    case DATA_TYPE_BITS:
        for (int i = 0; i < pd->data_size; i++) {
            Serial.printf("uint32_t %s_%i : 1;\n", name, i);
        }
        break;
    case DATA_TYPE_UNSIGNED:
        if (pd->data_size == 8) {
            Serial.printf("uint8_t %s;\n", name);
        } else {
            Serial.printf("warning: unsupported int size!\n");
        }
        break;
    case DATA_TYPE_SIGNED:
        if (pd->data_size == 8) {
            Serial.printf("int8_t %s;\n", name);
        } else {
            Serial.printf("warning: unsupported int size!\n");
        }
        break;
    case DATA_TYPE_FLOAT:
        if (pd->data_size != 32) {
            Serial.printf("warning: unsupported float size!");
        }
        Serial.printf("float %s;\n", name);
        break;
    case DATA_TYPE_BOOLEAN:
        Serial.printf("uint32_t %s : %u;\n", name, pd->data_size);
        break;
    default:
        Serial.printf("unsupported data type: 0x%02X\n", pd->data_type);
    }
}

static void send(uint8_t len, uint8_t docrc) {
    timeout = 0;
    if (docrc) {
        txbuf[len] = calculate_crc8((uint8_t *) txbuf, len);
    }
    Serial1.write(const_cast<uint8_t *>(txbuf), len + docrc);
}

void emptySerialBuffer() {
    while (Serial1.available()) {
        Serial1.read();
    }
}

void sserialLoop(void *pvParameters);

void sserial_init() {
    Serial.println("Init SSERIAL");

    heap_ptr = memory.heap;

    uint16_t input_bits  = 8;
    uint16_t output_bits = 0;

    uint16_t ptoc[32];
    uint16_t gtoc[32];

    uint16_t                  *ptocp = ptoc;
    uint16_t                  *gtocp = gtoc;
    process_data_descriptor_t *last_pd;

    ADD_PROCESS_VAR(("out", "none", 16, DATA_TYPE_BITS, DATA_DIRECTION_OUTPUT, -100, 100));
    metadata(&(pd_table.enable), last_pd);

    ADD_PROCESS_VAR(("joy_x", "none", 8, DATA_TYPE_SIGNED, DATA_DIRECTION_INPUT, -127, 127));
    metadata(&(pd_table.current), last_pd);
    ADD_PROCESS_VAR(("joy_y", "none", 8, DATA_TYPE_SIGNED, DATA_DIRECTION_INPUT, -127, 127));
    metadata(&(pd_table.current), last_pd);
    ADD_PROCESS_VAR(("joy_z", "none", 8, DATA_TYPE_SIGNED, DATA_DIRECTION_INPUT, -127, 127));
    metadata(&(pd_table.current), last_pd);
    ADD_PROCESS_VAR(("feedrate", "none", 8, DATA_TYPE_UNSIGNED, DATA_DIRECTION_INPUT, 0, 255));
    metadata(&(pd_table.current), last_pd);
    ADD_PROCESS_VAR(("rotation", "none", 8, DATA_TYPE_UNSIGNED, DATA_DIRECTION_INPUT, 0, 255));
    metadata(&(pd_table.current), last_pd);
    ADD_PROCESS_VAR(("in", "none", 32, DATA_TYPE_BITS, DATA_DIRECTION_INPUT, -100, 100));
    metadata(&(pd_table.input_pins), last_pd);

    ADD_MODE(("Position mode", 0, 1));

    if (input_bits % 8) ADD_PROCESS_VAR(("padding", "", 8 - (input_bits % 8), DATA_TYPE_PAD, DATA_DIRECTION_INPUT, 0, 0));
    if (output_bits % 8) ADD_PROCESS_VAR(("padding", "", 8 - (output_bits % 8), DATA_TYPE_PAD, DATA_DIRECTION_OUTPUT, 0, 0));

    memory.discovery.input  = input_bits >> 3;
    memory.discovery.output = output_bits >> 3;

    memory.discovery.ptocp = MEMPTR(*heap_ptr);

    for (uint8_t i = 0; i < ptocp - ptoc; i++) {
        *heap_ptr++ = ptoc[i] & 0x00FF;
        *heap_ptr++ = (ptoc[i] & 0xFF00) >> 8;
    }
    *heap_ptr++ = 0x00;
    *heap_ptr++ = 0x00;

    memory.discovery.gtocp = MEMPTR(*heap_ptr);

    for (uint8_t i = 0; i < gtocp - gtoc; i++) {
        *heap_ptr++ = gtoc[i] & 0x00FF;
        *heap_ptr++ = (gtoc[i] & 0xFF00) >> 8;
    }
    *heap_ptr++ = 0x00;
    *heap_ptr++ = 0x00;

    unit.unit = 0x04030201;

    // Insert print code here

    xTaskCreatePinnedToCore(sserialLoop, "SSerial Task", 10000, NULL, 10, NULL, SSERIAL_CPU);
}

uint32_t lastSerialCommunication = 0;

void sserialLoop(void *pvParameters) {
    for (;;) {
        auto available = Serial1.available();

        if (available >= 1) {
            sserial_timeoutFlag     = false;
            lastSerialCommunication = millis();
            lbp.byte                = static_cast<uint8_t>(Serial1.read());

            switch (lbp.ct) {
            case CT_LOCAL:
                if (lbp.wr == 0) {
                    handleLocalRead(available);
                } else {
                    handleLocalWrite(available);
                }
                break;
            case CT_RPC:
                handleRpc(available);
                break;
            case CT_RW:
                if (lbp.wr == 0) {
                    handleRead(available);
                } else {
                    handleWrite(available);
                }
                break;
            default:
                debug.addPrint(
                    "Unknown: Available: %lu, byte: 0x%02X lbp: ct: %d, wr: %d, ai: %d, as: %d, ds: %d, rid: %d, rpc: %d, dummy: %d",
                    available, lbp.byte, lbp.ct, lbp.wr, lbp.ai, lbp.as, lbp.ds, lbp.rid, lbp.rpc, lbp.dummy);
            }
            emptySerialBuffer();
        }
        checkForTimeout();
    }
}

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
        rxpos += 1;
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
    } else if (lbp.byte == ProcessDataRPC && available >= discovery.output + 2 - block_bytes) {
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
        memcpy((void *) txbuf, &sserial_slave[address], (1 << lbp.ds));
        send((1 << lbp.ds), 1);
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
        if ((address + (1 << lbp.ds)) < ARRAY_SIZE(sserial_slave)) {
            for (int i = 0; i < (1 << lbp.ds); i++) {
                sserial_slave[address + i] = Serial1.read();
            }
        }
        float tmp;
        memcpy(&tmp, &sserial_slave[scale_address], 4);
        if (lbp.ai) {
            address += (1 << lbp.ds);
        }
    }
}

void processDataInputs() {
    data_in.in_0  = ioRegister.mcp1Data.bits.alarmAll;
    data_in.in_1  = ioRegister.mcp0Data.bits.in1;
    data_in.in_2  = ioRegister.mcp0Data.bits.in2;
    data_in.in_3  = ioRegister.mcp0Data.bits.in3;
    data_in.in_4  = ioRegister.mcp0Data.bits.in4;
    data_in.in_5  = ioRegister.mcp0Data.bits.in5;
    data_in.in_6  = ioRegister.mcp0Data.bits.in6;
    data_in.in_7  = ioRegister.mcp0Data.bits.in7;
    data_in.in_8  = ioRegister.mcp0Data.bits.in8;
    data_in.in_9  = ioRegister.mcp0Data.bits.in9;
    data_in.in_10 = ioRegister.mcp0Data.bits.in10;
    data_in.in_11 = ioRegister.mcp0Data.bits.in11;
    data_in.in_12 = ioRegister.mcp0Data.bits.in12;
    data_in.in_13 = ioRegister.mcp0Data.bits.in13;
    data_in.in_14 = ioRegister.mcp0Data.bits.in14;
    data_in.in_15 = ioRegister.mcp0Data.bits.in15;
    data_in.in_16 = ioRegister.mcp0Data.bits.in16;
    data_in.in_17 = ioRegister.mcp1Data.bits.programmStart;
    data_in.in_18 = ioRegister.mcp1Data.bits.motorStart;
    data_in.in_19 = ioRegister.mcp1Data.bits.OK;
    data_in.in_20 = ioRegister.mcp1Data.bits.auswahlX;
    data_in.in_21 = ioRegister.mcp1Data.bits.auswahlY;
    data_in.in_22 = ioRegister.mcp1Data.bits.auswahlZ;
    data_in.in_23 = ioRegister.mcp1Data.bits.speed1;
    data_in.in_24 = ioRegister.mcp1Data.bits.speed2;

    data_in.joy_x = map(adcManager.getJoystickX(), 0, 4095, -127, 127);
    data_in.joy_y = map(adcManager.getJoystickY(), 0, 4095, -127, 127);
    data_in.joy_z = map(adcManager.getJoystickZ(), 0, 4095, -127, 127);
}

void processIncomingData() {
    for (int i = 0; i < discovery.output; i++) {
        ((uint8_t *) (&data_out))[i] = Serial1.read();
    }

    txbuf[0] = 0x00;
    for (int i = 0; i < (discovery.input - 1); i++) {
        txbuf[i + 1] = ((uint8_t *) (&data_in))[i];
    }
    txbuf[discovery.input] = calculate_crc8((uint8_t *) txbuf, discovery.input);
    send(discovery.input, 1);
}

void updateOutputPins() {
    ioRegister.setOutput(OutputPin::ENA, data_out.out_0);
    ioRegister.setOutput(OutputPin::OUT1, data_out.out_1);
    ioRegister.setOutput(OutputPin::OUT2, data_out.out_2);
    ioRegister.setOutput(OutputPin::OUT3, data_out.out_3);
    ioRegister.setOutput(OutputPin::OUT4, data_out.out_4);
    ioRegister.setOutput(OutputPin::OUT5, data_out.out_5);
    ioRegister.setOutput(OutputPin::OUT6, data_out.out_6);
    ioRegister.setOutput(OutputPin::OUT7, data_out.out_7);
    ioRegister.setOutput(OutputPin::OUT8, data_out.out_8);
    ioRegister.setOutput(OutputPin::SPINDEL_ON_OFF, data_out.out_9);
}

void checkForTimeout() {
    if (millis() - lastSerialCommunication > 5000 && !sserial_timeoutFlag) {
        debug.print("SSerial timeout");
        debug.printQueue();
        sserial_timeoutFlag = true;
    }
}

// Print code
// Serial.printf("gtoc:%u\n", memory.discovery.gtocp);
// Serial.printf("ptoc:%u\n", memory.discovery.ptocp);
// Serial.printf("%i\n", sizeof(memory_t));
// Serial.printf("%u\n", MEMPTR(*heap_ptr));
// int nl = 0;
// Serial.printf("uint8_t sserial_slave[] = {\n");
// for (int i = 0; i < MEMPTR(*heap_ptr); i++) {
//     // Serial.printf("%u %c\n",memory.bytes[i],memory.bytes[i]);
//     Serial.printf("0x%02X,", memory.bytes[i]);
//     nl++;
//     if (nl > 7) {
//         nl = 0;
//         Serial.printf("// %i..%i\n", i - 7, i);
//     }
// }
// Serial.printf("\n};\n\n");

// Serial.printf("const discovery_rpc_t discovery = {\n");
// Serial.printf("  .input = %u,\n", memory.discovery.input);
// Serial.printf("  .output = %u,\n", memory.discovery.output);
// Serial.printf("  .ptocp = 0x%04X,\n", memory.discovery.ptocp);
// Serial.printf("  .gtocp = 0x%04X,\n", memory.discovery.gtocp);
// Serial.printf("};\n\n");

// Serial.printf("typedef struct {\n");
// ptocp = (uint16_t *) (memory.bytes + memory.discovery.ptocp);
// gtocp = (uint16_t *) (memory.bytes + memory.discovery.gtocp);
// while (*ptocp != 0x0000) {
//     process_data_descriptor_t *pd = (process_data_descriptor_t *) (memory.bytes + *ptocp++);
//     // Serial.printf("0x%02X\n",pd->data_direction);
//     if ((pd->data_direction == DATA_DIRECTION_OUTPUT || pd->data_direction == DATA_DIRECTION_BI_DIRECTIONAL) &&
//         pd->record_type == RECORD_TYPE_PROCESS_DATA_RECORD) {
//         print_pd(pd);
//     }
//     // Serial.printf("pd has data at %x with value %x\n", pd->data_addr, MEMU16(pd->data_addr));
// }
// Serial.printf("} sserial_out_process_data_t; //size:%u bytes\n", memory.discovery.output);
// Serial.printf("\n");
// Serial.printf("typedef struct {\n");
// ptocp = (uint16_t *) (memory.bytes + memory.discovery.ptocp);
// while (*ptocp != 0x0000) {
//     process_data_descriptor_t *pd = (process_data_descriptor_t *) (memory.bytes + *ptocp++);
//     // Serial.printf("0x%02X\n",pd->data_direction);
//     if ((pd->data_direction == DATA_DIRECTION_INPUT || pd->data_direction == DATA_DIRECTION_BI_DIRECTIONAL) &&
//         pd->record_type == RECORD_TYPE_PROCESS_DATA_RECORD) {
//         print_pd(pd);
//     }
//     // Serial.printf("pd has data at %x with value %x\n", pd->data_addr, MEMU16(pd->data_addr));
// }
// Serial.printf("} sserial_in_process_data_t; //size:%u bytes\n", memory.discovery.input - 1);
// gtocp = (uint16_t *) (memory.bytes + memory.discovery.gtocp);
// while (*gtocp != 0x0000) {
//     process_data_descriptor_t *pd = (process_data_descriptor_t *) (memory.bytes + *gtocp++);
//     if (pd->record_type == RECORD_TYPE_PROCESS_DATA_RECORD) {
//         int   strl = strlen(&pd->names);
//         char *unit = &pd->names;
//         char *name = &pd->names + strl + 1;
//         Serial.printf("//global name:%s addr:0x%x size:%i dir:0x%x\n", name, pd->data_addr, pd->data_size, pd->data_direction);
//         Serial.printf("#define %s_address %i\n", name, pd->data_addr);
//     }
// }