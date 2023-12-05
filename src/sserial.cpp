#include "crc8.h"
#include "math.h"
#include <includes.h>
#include <string.h>

// TODO: move to ctx
struct sserial_ctx_t {
    uint32_t phase;
};

static volatile uint8_t rxbuf[128];   // rx dma buffer
static volatile uint8_t txbuf[128];   // tx dma buffer
static uint16_t         address;      // current address pointer
static int              rxpos;        // read pointer for rx ringbuffer
static uint32_t         timeout;
static lbp_t            lbp;
static const char       name[] = LBPCardName;
static unit_no_t        unit;
static uint32_t         max_waste_ticks;
static uint32_t         block_bytes;

//*****************************************************************************
uint8_t sserial_slave[] = {
    0x0D, 0x02, 0xFF, 0x00, 0x13, 0x01, 0x00, 0x00,   // 0..7
    0x00, 0x00, 0x00, 0x00, 0xA0, 0x10, 0x01, 0x80,   // 8..15
    0x00, 0x00, 0xC8, 0xC2, 0x00, 0x00, 0xC8, 0x42,   // 16..23
    0x08, 0x00, 0x6E, 0x6F, 0x6E, 0x65, 0x00, 0x6F,   // 24..31
    0x75, 0x74, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,   // 32..39
    0xA0, 0x0C, 0x02, 0x00, 0x00, 0x00, 0x00, 0x00,   // 40..47
    0x00, 0xF0, 0x7F, 0x45, 0x23, 0x00, 0x6E, 0x6F,   // 48..55
    0x6E, 0x65, 0x00, 0x6A, 0x6F, 0x79, 0x5F, 0x78,   // 56..63
    0x00, 0x00, 0x00, 0x00, 0xA0, 0x0C, 0x02, 0x00,   // 64..71
    0x00, 0x00, 0x00, 0x00, 0x00, 0xF0, 0x7F, 0x45,   // 72..79
    0x41, 0x00, 0x6E, 0x6F, 0x6E, 0x65, 0x00, 0x6A,   // 80..87
    0x6F, 0x79, 0x5F, 0x79, 0x00, 0x00, 0x00, 0x00,   // 88..95
    0xA0, 0x0C, 0x02, 0x00, 0x00, 0x00, 0x00, 0x00,   // 96..103
    0x00, 0xF0, 0x7F, 0x45, 0x5D, 0x00, 0x6E, 0x6F,   // 104..111
    0x6E, 0x65, 0x00, 0x6A, 0x6F, 0x79, 0x5F, 0x7A,   // 112..119
    0x00, 0x00, 0x00, 0x00, 0xA0, 0x0C, 0x02, 0x00,   // 120..127
    0x00, 0x00, 0x00, 0x00, 0x00, 0xF0, 0x7F, 0x45,   // 128..135
    0x79, 0x00, 0x6E, 0x6F, 0x6E, 0x65, 0x00, 0x66,   // 136..143
    0x65, 0x65, 0x64, 0x72, 0x61, 0x74, 0x65, 0x00,   // 144..151
    0x00, 0x00, 0x00, 0x00, 0xA0, 0x0C, 0x02, 0x00,   // 152..159
    0x00, 0x00, 0x00, 0x00, 0x00, 0xF0, 0x7F, 0x45,   // 160..167
    0x98, 0x00, 0x6E, 0x6F, 0x6E, 0x65, 0x00, 0x72,   // 168..175
    0x6F, 0x74, 0x61, 0x74, 0x69, 0x6F, 0x6E, 0x00,   // 176..183
    0x00, 0x00, 0x00, 0x00, 0xA0, 0x20, 0x01, 0x00,   // 184..191
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,   // 192..199
    0xB8, 0x00, 0x6E, 0x6F, 0x6E, 0x65, 0x00, 0x69,   // 200..207
    0x6E, 0x00, 0xB0, 0x00, 0x01, 0x00, 0x50, 0x6F,   // 208..215
    0x73, 0x69, 0x74, 0x69, 0x6F, 0x6E, 0x20, 0x6D,   // 216..223
    0x6F, 0x64, 0x65, 0x00, 0x00, 0x00, 0x00, 0x00,   // 224..231
    0xA0, 0x04, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,   // 232..239
    0x00, 0x00, 0x00, 0x00, 0xE4, 0x00, 0x00, 0x70,   // 240..247
    0x61, 0x64, 0x64, 0x69, 0x6E, 0x67, 0x00, 0x0C,   // 248..255
    0x00, 0x28, 0x00, 0x44, 0x00, 0x60, 0x00, 0x7C,   // 256..263
    0x00, 0x9C, 0x00, 0xBC, 0x00, 0xE8, 0x00, 0xA3,   // 264..271
    0x18, 0x00, 0x00, 0xD2, 0x00, 0x00, 0x00,
};

const discovery_rpc_t discovery = {
    .input  = 13,
    .output = 2,
    .ptocp  = 0x00FF,
    .gtocp  = 0x0113,
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
} sserial_out_process_data_t;   // size:2 bytes

typedef struct {
    uint16_t joy_x : 12;      // Custom size: you need to set/shift this bits manually before sending!!
    uint16_t joy_y : 12;      // Custom size: you need to set/shift this bits manually before sending!!
    uint16_t joy_z : 12;      // Custom size: you need to set/shift this bits manually before sending!!
    uint16_t feedrate : 12;   // Custom size: you need to set/shift this bits manually before sending!!
    uint16_t rotation : 12;   // Custom size: you need to set/shift this bits manually before sending!!
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
    uint32_t padding : 4;
} sserial_in_process_data_t;   // size:12 bytes
#define scale_address 300
//******************************************************************************

static sserial_out_process_data_t data_out;
static sserial_in_process_data_t  data_in;

TaskHandle_t sserialTaskHandle;

static uint8_t crc_reuest(uint8_t len) {
    uint8_t crc = crc8_init();
    for (int i = rxpos; i < rxpos + len; i++) {
        crc = crc8_update(crc, (void *) &(rxbuf[i % sizeof(rxbuf)]), 1);
    }
    crc = crc8_finalize(crc);
    return crc == rxbuf[(rxpos + len) % sizeof(rxbuf)];
}

static uint8_t crc8(uint8_t *addr, uint8_t len) {
    uint8_t crc = crc8_init();
    crc         = crc8_update(crc, addr, len);
    return crc8_finalize(crc);
}

memory_t   memory;
uint8_t   *heap_ptr;
pd_table_t pd_table;

uint16_t add_pd(const char *name_string, const char *unit_string, uint8_t data_size_in_bits, uint8_t data_type, uint8_t data_dir, float param_min, float param_max) {
    process_data_descriptor_t pdr;
    pdr.record_type    = RECORD_TYPE_PROCESS_DATA_RECORD;
    pdr.data_size      = data_size_in_bits;
    pdr.data_type      = data_type;
    pdr.data_direction = data_dir;
    pdr.param_min      = param_min;
    pdr.param_max      = param_max;
    pdr.data_addr      = MEMPTR(*heap_ptr);

    heap_ptr += NUM_BYTES(data_size_in_bits);
    // this aligns the heap pointer to 32bit.  Not doing this causes the floats in the pd to be misaligned, which crashes the arm.
    if ((uint32_t) heap_ptr % 4) {
        heap_ptr += 4 - (uint32_t) heap_ptr % 4;
    }

    memcpy(heap_ptr, &pdr, sizeof(process_data_descriptor_t));
    // note that we don't store the names in the struct anymore.  The fixed-length struct is copied into memory, and then the nmaes go in
    // directly behind it, so they'll read out properly

    uint16_t pd_ptr = MEMPTR(*heap_ptr);   // save off the ptr to return, before we modify the heap ptr

    heap_ptr = (uint8_t *) &(((process_data_descriptor_t *) heap_ptr)->names);

    // copy the strings in after the pd
    strcpy((char *) heap_ptr, unit_string);
    heap_ptr += strlen(unit_string) + 1;

    strcpy((char *) heap_ptr, name_string);
    heap_ptr += strlen(name_string) + 1;

    // moved this up to before the pd record
    /*
     // this aligns the heap pointer to 32bit.  Not doing this causes the floats in the pd to be misaligned, which crashes the arm.
     if((uint32_t)heap_ptr % 4){
     heap_ptr += 4 - (uint32_t)heap_ptr % 4;
     }
     */

    return pd_ptr;
}

uint16_t add_mode(const char *name_string, uint8_t index, uint8_t type) {
    mode_descriptor_t mdr;
    mdr.record_type = RECORD_TYPE_MODE_DATA_RECORD;
    mdr.index       = index;
    mdr.type        = type;   // hw = 0, sw = 1
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
        } else if (pd->data_size > 8 && pd->data_size <= 16) {
            Serial.printf("uint16_t %s : %i; // Custom size: you need to set/shift this bits manually before sending!!\n", name, pd->data_size);
        } else if (pd->data_size > 16 && pd->data_size <= 32) {
            Serial.printf("uint32_t %s : %i; // Custom size: you need to set/shift this bits manually before sending!!\n", name, pd->data_size);
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
        // TODO: check size
        if (pd->data_size != 32) {
            Serial.printf("warning: unsupported float size!");
        }
        Serial.printf("float %s;\n", name);
        break;
    case DATA_TYPE_BOOLEAN:
        Serial.printf("uint32_t %s : %u;\n", name, pd->data_size);
        break;
    case DATA_TYPE_NONVOL_UNSIGNED:
    case DATA_TYPE_NONVOL_SIGNED:
    case DATA_TYPE_STREAM:
    case DATA_TYPE_ENCODER:
    case DATA_TYPE_ENCODER_H:
    case DATA_TYPE_ENCODER_L:
    default:
        Serial.printf("unsupported data type: 0x%02X\n", pd->data_type);
    }
}

static void send(uint8_t len, uint8_t docrc) {
    timeout = 0;
    if (docrc) {
        txbuf[len] = crc8((uint8_t *) txbuf, len);
    }
    Serial1.write(const_cast<uint8_t *>(txbuf), len + docrc);
#ifdef USE_DEBUG
    // Construct a string with all the bytes to be sent
    String output = "Sending: ";
    for (int i = 0; i < len + docrc; i++) {
        output += String(txbuf[i], HEX);   // Convert each byte to a hex string and append
        output += " ";                     // Add space after each byte
    }

    // Now use addPrint with the constructed string
    debug.addPrint(output.c_str());   // Convert String to C-style string
#endif
}

// Reads the serial buffer until its empty
void emptySerialBuffer() {
    while (Serial1.available()) {
        Serial1.read();
    }
}

void sserialLoop(void *pvParameters);

void sserial_init() {
    Serial.println("Init SSERIAL");

    heap_ptr = memory.heap;

    uint16_t input_bits  = 8;   // this starts at 8 bits = 1 byte for the fault byte
    uint16_t output_bits = 0;

    // these are temp toc arrays that the macros will write pointers into.
    // the tocs get copied to main memory after everything else is
    // written in
    uint16_t ptoc[32];
    uint16_t gtoc[32];

    uint16_t                  *ptocp = ptoc;
    uint16_t                  *gtocp = gtoc;
    process_data_descriptor_t *last_pd;

    // ADD_PROCESS_VAR(("pos_cmd", "rad", 32, DATA_TYPE_FLOAT, DATA_DIRECTION_OUTPUT, -INFINITY, INFINITY));
    // metadata(&(pd_table.pos_cmd), last_pd);
    // ADD_PROCESS_VAR(("vel_cmd", "rad", 32, DATA_TYPE_FLOAT, DATA_DIRECTION_OUTPUT, -INFINITY, INFINITY));
    // metadata(&(pd_table.vel_cmd), last_pd);
    // ADD_PROCESS_VAR(("out", "none", 4, DATA_TYPE_BITS, DATA_DIRECTION_OUTPUT, 0, 1));
    // metadata(&(pd_table.output_pins), last_pd);
    // ADD_PROCESS_VAR(("enable", "none", 1, DATA_TYPE_BOOLEAN, DATA_DIRECTION_OUTPUT, 0, 1));
    // metadata(&(pd_table.enable), last_pd);
    ADD_PROCESS_VAR(("out", "none", 16, DATA_TYPE_BITS, DATA_DIRECTION_OUTPUT, -100, 100));
    metadata(&(pd_table.enable), last_pd);

    // ADD_PROCESS_VAR(("pos_fb", "rad", 32, DATA_TYPE_FLOAT, DATA_DIRECTION_INPUT, -INFINITY, INFINITY));
    // metadata(&(pd_table.pos_fb), last_pd);
    // ADD_PROCESS_VAR(("vel_fb", "rad", 32, DATA_TYPE_FLOAT, DATA_DIRECTION_INPUT, -INFINITY, INFINITY));
    // metadata(&(pd_table.vel_fb), last_pd);
    // ADD_PROCESS_VAR(("current", "A", 8, DATA_TYPE_SIGNED, DATA_DIRECTION_INPUT, -30, 30));
    // metadata(&(pd_table.current), last_pd);

    // Input calculcation
    // Available space is 12 bytes or 96 bits
    // 32 bit are used for general inputs
    // 12 bit for every analog input
    // 32 + 12 * 5 = 92
    // 92 / 8 = 11.5 Bytes
    ADD_PROCESS_VAR(("joy_x", "none", 12, DATA_TYPE_UNSIGNED, DATA_DIRECTION_INPUT, 0, 4095));
    metadata(&(pd_table.current), last_pd);
    ADD_PROCESS_VAR(("joy_y", "none", 12, DATA_TYPE_UNSIGNED, DATA_DIRECTION_INPUT, 0, 4095));
    metadata(&(pd_table.current), last_pd);
    ADD_PROCESS_VAR(("joy_z", "none", 12, DATA_TYPE_UNSIGNED, DATA_DIRECTION_INPUT, 0, 4095));
    metadata(&(pd_table.current), last_pd);
    ADD_PROCESS_VAR(("feedrate", "none", 12, DATA_TYPE_UNSIGNED, DATA_DIRECTION_INPUT, 0, 4095));
    metadata(&(pd_table.current), last_pd);
    ADD_PROCESS_VAR(("rotation", "none", 12, DATA_TYPE_UNSIGNED, DATA_DIRECTION_INPUT, 0, 4095));
    metadata(&(pd_table.current), last_pd);

    // Todo split the general inputs directly to the correct names
    ADD_PROCESS_VAR(("in", "none", 32, DATA_TYPE_BITS, DATA_DIRECTION_INPUT, 0, 0));
    metadata(&(pd_table.input_pins), last_pd);
    // ADD_PROCESS_VAR(("fault", "none", 1, DATA_TYPE_BOOLEAN, DATA_DIRECTION_INPUT, 0, 1));
    // metadata(&(pd_table.fault), last_pd);
    // ADD_PROCESS_VAR(("index_enable", "none", 1, DATA_TYPE_BOOLEAN, DATA_DIRECTION_BI_DIRECTIONAL, 0, 1));
    // metadata(&(pd_table.index_enable), last_pd);

    // ADD_GLOBAL_VAR(("scale", "none", 32, DATA_TYPE_FLOAT, DATA_DIRECTION_OUTPUT, -INFINITY, INFINITY));

    ADD_MODE(("Position mode", 0, 1));

    // automatically create padding pds based on the mod remainder of input/output bits
    if (input_bits % 8) ADD_PROCESS_VAR(("padding", "", 8 - (input_bits % 8), DATA_TYPE_PAD, DATA_DIRECTION_INPUT, 0, 0));
    if (output_bits % 8) ADD_PROCESS_VAR(("padding", "", 8 - (output_bits % 8), DATA_TYPE_PAD, DATA_DIRECTION_OUTPUT, 0, 0));

    // now that all the toc entries have been added, write out the tocs to memory and set up the toc pointers

    // calculate bytes from bits
    memory.discovery.input  = input_bits >> 3;
    memory.discovery.output = output_bits >> 3;

    memory.discovery.ptocp = MEMPTR(*heap_ptr);

    for (uint8_t i = 0; i < ptocp - ptoc; i++) {
        *heap_ptr++ = ptoc[i] & 0x00FF;
        *heap_ptr++ = (ptoc[i] & 0xFF00) >> 8;
    }
    // this is the ptoc end marker
    *heap_ptr++ = 0x00;
    *heap_ptr++ = 0x00;

    memory.discovery.gtocp = MEMPTR(*heap_ptr);

    for (uint8_t i = 0; i < gtocp - gtoc; i++) {
        *heap_ptr++ = gtoc[i] & 0x00FF;
        *heap_ptr++ = (gtoc[i] & 0xFF00) >> 8;
    }
    // this is the gtoc end marker
    *heap_ptr++ = 0x00;
    *heap_ptr++ = 0x00;

    Serial.printf("gtoc:%u\n", memory.discovery.gtocp);
    Serial.printf("ptoc:%u\n", memory.discovery.ptocp);
    Serial.printf("%i\n", sizeof(memory_t));
    Serial.printf("%u\n", MEMPTR(*heap_ptr));
    int nl = 0;
    Serial.printf("uint8_t sserial_slave[] = {\n");
    for (int i = 0; i < MEMPTR(*heap_ptr); i++) {
        // Serial.printf("%u %c\n",memory.bytes[i],memory.bytes[i]);
        Serial.printf("0x%02X,", memory.bytes[i]);
        nl++;
        if (nl > 7) {
            nl = 0;
            Serial.printf("// %i..%i\n", i - 7, i);
        }
    }
    Serial.printf("\n};\n\n");

    Serial.printf("const discovery_rpc_t discovery = {\n");
    Serial.printf("  .input = %u,\n", memory.discovery.input);
    Serial.printf("  .output = %u,\n", memory.discovery.output);
    Serial.printf("  .ptocp = 0x%04X,\n", memory.discovery.ptocp);
    Serial.printf("  .gtocp = 0x%04X,\n", memory.discovery.gtocp);
    Serial.printf("};\n\n");

    Serial.printf("typedef struct {\n");
    ptocp = (uint16_t *) (memory.bytes + memory.discovery.ptocp);
    gtocp = (uint16_t *) (memory.bytes + memory.discovery.gtocp);
    while (*ptocp != 0x0000) {
        process_data_descriptor_t *pd = (process_data_descriptor_t *) (memory.bytes + *ptocp++);
        // Serial.printf("0x%02X\n",pd->data_direction);
        if ((pd->data_direction == DATA_DIRECTION_OUTPUT || pd->data_direction == DATA_DIRECTION_BI_DIRECTIONAL) && pd->record_type == RECORD_TYPE_PROCESS_DATA_RECORD) {
            print_pd(pd);
        }
        // Serial.printf("pd has data at %x with value %x\n", pd->data_addr, MEMU16(pd->data_addr));
    }
    Serial.printf("} sserial_out_process_data_t; //size:%u bytes\n", memory.discovery.output);
    Serial.printf("\n");
    Serial.printf("typedef struct {\n");
    ptocp = (uint16_t *) (memory.bytes + memory.discovery.ptocp);
    while (*ptocp != 0x0000) {
        process_data_descriptor_t *pd = (process_data_descriptor_t *) (memory.bytes + *ptocp++);
        // Serial.printf("0x%02X\n",pd->data_direction);
        if ((pd->data_direction == DATA_DIRECTION_INPUT || pd->data_direction == DATA_DIRECTION_BI_DIRECTIONAL) && pd->record_type == RECORD_TYPE_PROCESS_DATA_RECORD) {
            print_pd(pd);
        }
        // Serial.printf("pd has data at %x with value %x\n", pd->data_addr, MEMU16(pd->data_addr));
    }
    Serial.printf("} sserial_in_process_data_t; //size:%u bytes\n", memory.discovery.input - 1);
    gtocp = (uint16_t *) (memory.bytes + memory.discovery.gtocp);
    while (*gtocp != 0x0000) {
        process_data_descriptor_t *pd = (process_data_descriptor_t *) (memory.bytes + *gtocp++);
        if (pd->record_type == RECORD_TYPE_PROCESS_DATA_RECORD) {
            int   strl = strlen(&pd->names);
            char *unit = &pd->names;
            char *name = &pd->names + strl + 1;
            Serial.printf("//global name:%s addr:0x%x size:%i dir:0x%x\n", name, pd->data_addr, pd->data_size, pd->data_direction);
            Serial.printf("#define %s_address %i\n", name, pd->data_addr);
        }
    }

    // generate unit number from 96bit unique chip ID
    unit.unit = 0x04030201;

    Serial.printf("SSerial init done\n");

    xTaskCreatePinnedToCore(sserialLoop,    /* Task function. */
                            "SSerial Task", /* name of task. */
                            10000,          /* Stack size of task */
                            NULL,           /* parameter of the task */
                            10,             /* priority of the task */
                            NULL,           /* Task handle to keep track of created task */
                            SSERIAL_CPU);
}

uint32_t lastSerialCommunication = 0;

void sserialLoop(void *pvParameters) {
    for (;;) {
        auto available = Serial1.available();

        if (available >= 1) {
            sserial_timeoutFlag     = false;
            lastSerialCommunication = millis();
            // VERB_PRINTF("Available: %lu", available);
            lbp.byte = static_cast<uint8_t>(Serial1.read());
            // VERB_PRINTF("Received: 0x%02X", static_cast<uint32_t>(lbp.byte));

#ifdef USE_DEBUG
            // print the available, byte and the whole lbt stuct
            debug.addPrint("Available: %lu, byte: 0x%02X lbp: ct: %d, wr: %d, ai: %d, as: %d, ds: %d, rid: %d, rpc: %d, dummy: %d", available, lbp.byte, lbp.ct, lbp.wr, lbp.ai,
                           lbp.as, lbp.ds, lbp.rid, lbp.rpc, lbp.dummy);
#endif
            if (lbp.ct == CT_LOCAL /*11b*/ && lbp.wr == 0) {   // local read, cmd+crc = 2b
                // timeout = 0;
                if (available >= 2) {
                    emptySerialBuffer();
                    switch (lbp.byte) {
                    case LBPCookieCMD:
                        txbuf[0] = LBPCookie;
                        break;
                    case LBPStatusCMD:   // TODO: return status
                        txbuf[0] = 0x00;
                        break;
                    case LBPCardName0Cmd ... LBPCardName3Cmd:
                        txbuf[0] = name[lbp.byte - LBPCardName0Cmd];
                        break;
                    default:   // TODO: handle unknown command condition
                        txbuf[0] = 0x00;
                    }
                    send(1, 1);
                }
            } else if (lbp.ct == CT_LOCAL && lbp.wr == 1) {   // local write, cmd+data+crc = 3b
                emptySerialBuffer();
                // timeout = 0;
                // 0xFF and 0xFC are not followed by crc
                if (lbp.byte == 0xFF) {
                    // reset parser
                    rxpos += 1;
                } else if (lbp.byte == 0xFC) {
                    // todo
                    rxpos += 1;
                } else if (available >= 3) {   // writes do not expect crc in reply
                    txbuf[0] = 0x00;
                    send(1, 0);
                }
            } else if (lbp.ct == CT_RPC) {   // RPC TODO: check for ct should not required for rpc
                // timeout = 0;
                if (lbp.byte == UnitNumberRPC && available >= 2) {   // unit number, cmd+crc = 2b
                    emptySerialBuffer();
                    txbuf[0] = unit.byte[0];
                    txbuf[1] = unit.byte[1];
                    txbuf[2] = unit.byte[2];
                    txbuf[3] = unit.byte[3];
                    send(4, 1);
                } else if (lbp.byte == DiscoveryRPC && available >= 2) {   // discovery, cmd+crc = 2b
                    emptySerialBuffer();
                    memcpy((void *) txbuf, ((uint8_t *) &discovery), sizeof(discovery));
                    send(sizeof(discovery), 1);
                } else if (lbp.byte == ProcessDataRPC && available >= discovery.output + 2 - block_bytes) {   // process data, requires cmd+output bytes+crc
                    // uint32_t t1         = hal_get_systick_value();
                    // uint32_t wait_ticks = 0;
                    // // wait with timeout until rest of process data is received
                    // do {
                    //     uint32_t t2 = hal_get_systick_value();
                    //     if (t1 < t2) {
                    //         t1 += hal_get_systick_reload();
                    //     }
                    //     wait_ticks = t1 - t2;
                    //     // next received packet will be written to bufferpos
                    //     bufferpos = sizeof(rxbuf) - DMA_GetCurrDataCounter(DMA2_Stream5);
                    //     // how many packets we have the the rx buffer for processing
                    //     available = (bufferpos - rxpos + sizeof(rxbuf)) % sizeof(rxbuf);
                    // } while (available < discovery.output + 2 && wait_ticks <= max_waste_ticks);
                    // TODO: fault handling on timeout...
                    // set input pins
                    // data_in.pos_fb  = PIN(pos_fb) + PIN(vel_fb) * PIN(pos_advance);
                    // data_in.vel_fb  = PIN(vel_fb);
                    // data_in.current = CLAMP(PIN(current) / (30.0f / 128.0f), -127, 127);
                    // data_in.in_0    = (PIN(in0) > 0) ? 1 : 0;
                    // data_in.in_1    = (PIN(in1) > 0) ? 1 : 0;
                    // data_in.in_2    = (PIN(in2) > 0) ? 1 : 0;
                    // data_in.in_3    = (PIN(in3) > 0) ? 1 : 0;
                    // data_in.fault   = (PIN(fault) > 0) ? 1 : 0;

                    // data_in.in_0 = 1;
                    // data_in.in_1 = 1;
                    // data_in.in_2 = millis();
                    // data_in.in_3 = millis();

                    // data_in.in_0  = inRegister.data.bits.alarmAll;
                    // data_in.in_1  = inRegister.data.bits.in1;
                    // data_in.in_2  = inRegister.data.bits.in2;
                    // data_in.in_3  = inRegister.data.bits.in3;
                    // data_in.in_4  = inRegister.data.bits.in4;
                    // data_in.in_5  = inRegister.data.bits.in5;
                    // data_in.in_6  = inRegister.data.bits.in6;
                    // data_in.in_7  = inRegister.data.bits.in7;
                    // data_in.in_8  = inRegister.data.bits.in8;
                    // data_in.in_9  = inRegister.data.bits.in9;
                    // data_in.in_10 = inRegister.data.bits.in10;
                    // data_in.in_11 = inRegister.data.bits.in11;
                    // data_in.in_12 = inRegister.data.bits.in12;
                    // data_in.in_13 = inRegister.data.bits.in13;
                    // data_in.in_14 = inRegister.data.bits.in14;
                    // data_in.in_15 = inRegister.data.bits.in15;
                    // data_in.in_16 = inRegister.data.bits.in16;
                    // data_in.in_17 = inRegister.data.bits.programmStart;
                    // data_in.in_18 = inRegister.data.bits.motorStart;
                    // data_in.in_19 = inRegister.data.bits.ok;
                    // data_in.in_20 = inRegister.data.bits.auswahlX;
                    // data_in.in_21 = inRegister.data.bits.auswahlY;
                    // data_in.in_22 = inRegister.data.bits.auswahlZ;
                    // data_in.in_23 = inRegister.data.bits.speed1;
                    // data_in.in_24 = inRegister.data.bits.speed2;
                    data_in.joy_x    = adcManager.getJoystickX();
                    data_in.joy_y    = adcManager.getJoystickY();
                    data_in.joy_z    = adcManager.getJoystickZ();
                    data_in.feedrate = adcManager.getFeedrate();
                    data_in.rotation = adcManager.getRotationSpeed();

                    // copy output pins from rx buffer
                    for (int i = 0; i < discovery.output; i++) {
                        ((uint8_t *) (&data_out))[i] = Serial1.read();
                    }

                    // // copy input pins to tx buffer
                    txbuf[0] = 0x00;   // fault byte

                    // // Kopiere die 12-Bit-Felder von joy_x, joy_y, joy_z, feedrate, rotation
                    // txbuf[1] = data_in.joy_x & 0xFF;
                    // txbuf[2] = ((data_in.joy_x >> 8) & 0x0F) | ((data_in.joy_y << 4) & 0xF0);
                    // txbuf[3] = (data_in.joy_y >> 4) & 0xFF;
                    // txbuf[4] = ((data_in.joy_y >> 12) & 0x0F) | ((data_in.joy_z << 4) & 0xF0);
                    // txbuf[5] = (data_in.joy_z >> 4) & 0xFF;
                    // txbuf[6] = ((data_in.joy_z >> 12) & 0x0F) | ((data_in.feedrate << 4) & 0xF0);
                    // txbuf[7] = (data_in.feedrate >> 4) & 0xFF;
                    // txbuf[8] = ((data_in.feedrate >> 12) & 0x0F) | ((data_in.rotation << 4) & 0xF0);
                    // txbuf[9] = (data_in.rotation >> 4) & 0xFF;

                    // Kopiere die 12-Bit-Felder von joy_x, joy_y, joy_z, feedrate, rotation
                    // Kopiere die 12-Bit-Felder von joy_x, joy_y, joy_z
                    txbuf[1] = data_in.joy_x & 0xFF;                                               // Die unteren 8 Bit von joy_x
                    txbuf[2] = ((data_in.joy_x >> 8) & 0x0F) | ((data_in.joy_y << 4) & 0xF0);      // Die oberen 4 Bit von joy_x und die unteren 4 Bit von joy_y
                    txbuf[3] = (data_in.joy_y >> 4) & 0xFF;                                        // Die nächsten 8 Bit von joy_y
                    txbuf[4] = data_in.joy_z & 0xFF;                                               // Die unteren 8 Bit von joy_z
                    txbuf[5] = ((data_in.joy_z >> 8) & 0x0F) | ((data_in.feedrate << 4) & 0xF0);   // Die oberen 4 Bit von joy_z und die unteren 4 Bit von feedrate
                    txbuf[6] = (data_in.feedrate >> 4) & 0xFF;                                     // Die nächsten 8 Bit von feedrate
                    txbuf[7] = data_in.rotation & 0xFF;                                            // Die unteren 8 Bit von rotation
                    // die oberen 4 bit von rotation und die 4 bits in_0, in_1, in_2, in_3
                    txbuf[8] = ((data_in.rotation >> 8) & 0x0F) | (data_in.in_0 << 4) | (data_in.in_1 << 5) | (data_in.in_2 << 6) | (data_in.in_3 << 7);

                    for (int i = 9; i < (discovery.input - 1); i++) {
                        txbuf[i + 1] = ((uint8_t *) (&data_in))[i];
                    }

                    // send
                    txbuf[discovery.input] = crc8((uint8_t *) txbuf, discovery.input);
                    send(discovery.input, 1);
                    // set output pins
                    // outRegister.set(RegisterOut::Output::ena, data_out.out_0);
                    // outRegister.set(RegisterOut::Output::out1, data_out.out_1);
                    // outRegister.set(RegisterOut::Output::out2, data_out.out_2);
                    // outRegister.set(RegisterOut::Output::out3, data_out.out_3);
                    // outRegister.set(RegisterOut::Output::out4, data_out.out_4);
                    // outRegister.set(RegisterOut::Output::out5, data_out.out_5);
                    // outRegister.set(RegisterOut::Output::out6, data_out.out_6);
                    // outRegister.set(RegisterOut::Output::out7, data_out.out_7);
                    // outRegister.set(RegisterOut::Output::out8, data_out.out_8);
                    // outRegister.set(RegisterOut::Output::spindelOnOff, data_out.out_9);
                    // outRegister.set(RegisterOut::Output::out10, data_out.out_10);
                    // outRegister.set(RegisterOut::Output::out11, data_out.out_11);
                    // outRegister.set(RegisterOut::Output::out12, data_out.out_12);
                    // outRegister.set(RegisterOut::Output::out13, data_out.out_13);
                    // outRegister.set(RegisterOut::Output::out14, data_out.out_14);
                    // outRegister.set(RegisterOut::Output::out15, data_out.out_15);
                }
            } else if (lbp.ct == CT_RW && lbp.wr == 0) {   // read
                // size = 1 + 2*lbp.as  + 1
                int size = 2 * lbp.as + 2;
                // timeout  = 0;
                if (available >= size) {
                    if (lbp.as) {   // address included in command = cmd+addr+addr+crc
                        const uint8_t frist  = Serial1.read();
                        const uint8_t second = Serial1.read();
                        address              = frist + (second << 8);
                        // uint8_t address = rxbuf[(rxpos + 1) % sizeof(rxbuf)] + (rxbuf[(rxpos + 2) % sizeof(rxbuf)] << 8);
                    }
                    // print address
                    emptySerialBuffer();
                    // TODO: causes timeouts...
                    // if((address + (1 << lbp.ds)) < ARRAY_SIZE(sserial_slave)) {  //check if address is valid
                    memcpy((void *) txbuf, &sserial_slave[address], (1 << lbp.ds));
                    send((1 << lbp.ds), 1);
                    //}
                    if (lbp.ai) {   // auto increment address by datasize
                        address += (1 << lbp.ds);
                    }
                }
            } else if (lbp.ct == CT_RW && lbp.wr == 1) {   // lbp (addr1 addr2) data0, data1,...
                // size = 1 + 2*ai +ds +crc
                int size = 2 * lbp.as + (1 << lbp.ds) + 2;
                // timeout  = 0;
                if (available >= size) {
                    if (lbp.as) {                                        // address included in command = cmd+addr+addr+crc
                        const uint8_t first   = Serial1.read();          // 10100110
                        const uint8_t second  = Serial1.read();          // 00000001
                        uint8_t       address = first + (second << 8);   // 0000000110100110 -> 422
                        // rxpos += 3;
                    } else {   // address not included in command = cmd+crc
                        // rxpos += 1;
                    }
                    // TODO: check size
                    if ((address + (1 << lbp.ds)) < ARRAY_SIZE(sserial_slave)) {   // check if address is valid
                        for (int i = 0; i < (1 << lbp.ds); i++) {
                            // sserial_slave[address + i] = rxbuf[(rxpos + i) % sizeof(rxbuf)];
                            sserial_slave[address + i] = Serial1.read();
                        }
                    }
                    // rxpos += (1 << lbp.ds) + 1;
                    // update globals
                    float tmp;
                    memcpy(&tmp, &sserial_slave[scale_address], 4);
                    // PIN(scale) = tmp;
                    if (lbp.ai) {   // auto increment address by datasize
                        address += (1 << lbp.ds);
                    }
                }
            } else {
                // TODO: handle unkown packet
            }
            emptySerialBuffer();
        }
        if (millis() - lastSerialCommunication > 5000 && !sserial_timeoutFlag) {
            debug.print("SSerial timeout");
            debug.printQueue();
            sserial_timeoutFlag = true;
        }
    }
    // taskYIELD();
}
