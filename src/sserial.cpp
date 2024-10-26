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

uint8_t sserial_slave[] = {
    0x0A, 0x02, 0xA3, 0x04, 0xF9, 0x04, 0x00, 0x00,   // 0..7
    0x00, 0x00, 0x00, 0x00, 0xA0, 0x01, 0x01, 0x80,   // 8..15
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x80, 0x3F,   // 16..23
    0x08, 0x00, 0x6E, 0x6F, 0x6E, 0x65, 0x00, 0x6F,   // 24..31
    0x75, 0x74, 0x31, 0x00, 0x00, 0x00, 0x00, 0x00,   // 32..39
    0xA0, 0x01, 0x01, 0x80, 0x00, 0x00, 0x00, 0x00,   // 40..47
    0x00, 0x00, 0x80, 0x3F, 0x24, 0x00, 0x6E, 0x6F,   // 48..55
    0x6E, 0x65, 0x00, 0x6F, 0x75, 0x74, 0x32, 0x00,   // 56..63
    0x00, 0x00, 0x00, 0x00, 0xA0, 0x01, 0x01, 0x80,   // 64..71
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x80, 0x3F,   // 72..79
    0x40, 0x00, 0x6E, 0x6F, 0x6E, 0x65, 0x00, 0x6F,   // 80..87
    0x75, 0x74, 0x33, 0x00, 0x00, 0x00, 0x00, 0x00,   // 88..95
    0xA0, 0x01, 0x01, 0x80, 0x00, 0x00, 0x00, 0x00,   // 96..103
    0x00, 0x00, 0x80, 0x3F, 0x5C, 0x00, 0x6E, 0x6F,   // 104..111
    0x6E, 0x65, 0x00, 0x6F, 0x75, 0x74, 0x34, 0x00,   // 112..119
    0x00, 0x00, 0x00, 0x00, 0xA0, 0x01, 0x01, 0x80,   // 120..127
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x80, 0x3F,   // 128..135
    0x78, 0x00, 0x6E, 0x6F, 0x6E, 0x65, 0x00, 0x6F,   // 136..143
    0x75, 0x74, 0x35, 0x00, 0x00, 0x00, 0x00, 0x00,   // 144..151
    0xA0, 0x01, 0x01, 0x80, 0x00, 0x00, 0x00, 0x00,   // 152..159
    0x00, 0x00, 0x80, 0x3F, 0x94, 0x00, 0x6E, 0x6F,   // 160..167
    0x6E, 0x65, 0x00, 0x6F, 0x75, 0x74, 0x36, 0x00,   // 168..175
    0x00, 0x00, 0x00, 0x00, 0xA0, 0x01, 0x01, 0x80,   // 176..183
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x80, 0x3F,   // 184..191
    0xB0, 0x00, 0x6E, 0x6F, 0x6E, 0x65, 0x00, 0x6F,   // 192..199
    0x75, 0x74, 0x37, 0x00, 0x00, 0x00, 0x00, 0x00,   // 200..207
    0xA0, 0x01, 0x01, 0x80, 0x00, 0x00, 0x00, 0x00,   // 208..215
    0x00, 0x00, 0x80, 0x3F, 0xCC, 0x00, 0x6E, 0x6F,   // 216..223
    0x6E, 0x65, 0x00, 0x6F, 0x75, 0x74, 0x38, 0x00,   // 224..231
    0x00, 0x00, 0x00, 0x00, 0xA0, 0x01, 0x01, 0x80,   // 232..239
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x80, 0x3F,   // 240..247
    0xE8, 0x00, 0x6E, 0x6F, 0x6E, 0x65, 0x00, 0x65,   // 248..255
    0x6E, 0x61, 0x00, 0x00, 0xA0, 0x01, 0x01, 0x80,   // 256..263
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x80, 0x3F,   // 264..271
    0x03, 0x01, 0x6E, 0x6F, 0x6E, 0x65, 0x00, 0x73,   // 272..279
    0x70, 0x69, 0x6E, 0x64, 0x65, 0x6C, 0x00, 0x00,   // 280..287
    0xA0, 0x08, 0x03, 0x00, 0x00, 0x00, 0xFE, 0xC2,   // 288..295
    0x00, 0x00, 0xFE, 0x42, 0x1F, 0x01, 0x6E, 0x6F,   // 296..303
    0x6E, 0x65, 0x00, 0x6A, 0x6F, 0x79, 0x5F, 0x78,   // 304..311
    0x00, 0x00, 0x00, 0x00, 0xA0, 0x08, 0x03, 0x00,   // 312..319
    0x00, 0x00, 0xFE, 0xC2, 0x00, 0x00, 0xFE, 0x42,   // 320..327
    0x39, 0x01, 0x6E, 0x6F, 0x6E, 0x65, 0x00, 0x6A,   // 328..335
    0x6F, 0x79, 0x5F, 0x79, 0x00, 0x00, 0x00, 0x00,   // 336..343
    0xA0, 0x08, 0x03, 0x00, 0x00, 0x00, 0xFE, 0xC2,   // 344..351
    0x00, 0x00, 0xFE, 0x42, 0x55, 0x01, 0x6E, 0x6F,   // 352..359
    0x6E, 0x65, 0x00, 0x6A, 0x6F, 0x79, 0x5F, 0x7A,   // 360..367
    0x00, 0x00, 0x00, 0x00, 0xA0, 0x08, 0x02, 0x00,   // 368..375
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x7F, 0x43,   // 376..383
    0x71, 0x01, 0x6E, 0x6F, 0x6E, 0x65, 0x00, 0x66,   // 384..391
    0x65, 0x65, 0x64, 0x72, 0x61, 0x74, 0x65, 0x00,   // 392..399
    0x00, 0x00, 0x00, 0x00, 0xA0, 0x08, 0x02, 0x00,   // 400..407
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x7F, 0x43,   // 408..415
    0x90, 0x01, 0x6E, 0x6F, 0x6E, 0x65, 0x00, 0x72,   // 416..423
    0x6F, 0x74, 0x61, 0x74, 0x69, 0x6F, 0x6E, 0x00,   // 424..431
    0x00, 0x00, 0x00, 0x00, 0xA0, 0x01, 0x01, 0x00,   // 432..439
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x80, 0x3F,   // 440..447
    0xB0, 0x01, 0x6E, 0x6F, 0x6E, 0x65, 0x00, 0x69,   // 448..455
    0x6E, 0x31, 0x00, 0x00, 0xA0, 0x01, 0x01, 0x00,   // 456..463
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x80, 0x3F,   // 464..471
    0xCB, 0x01, 0x6E, 0x6F, 0x6E, 0x65, 0x00, 0x69,   // 472..479
    0x6E, 0x32, 0x00, 0x00, 0xA0, 0x01, 0x01, 0x00,   // 480..487
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x80, 0x3F,   // 488..495
    0xE3, 0x01, 0x6E, 0x6F, 0x6E, 0x65, 0x00, 0x69,   // 496..503
    0x6E, 0x33, 0x00, 0x00, 0xA0, 0x01, 0x01, 0x00,   // 504..511
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x80, 0x3F,   // 512..519
    0xFB, 0x01, 0x6E, 0x6F, 0x6E, 0x65, 0x00, 0x69,   // 520..527
    0x6E, 0x34, 0x00, 0x00, 0xA0, 0x01, 0x01, 0x00,   // 528..535
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x80, 0x3F,   // 536..543
    0x13, 0x02, 0x6E, 0x6F, 0x6E, 0x65, 0x00, 0x69,   // 544..551
    0x6E, 0x35, 0x00, 0x00, 0xA0, 0x01, 0x01, 0x00,   // 552..559
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x80, 0x3F,   // 560..567
    0x2B, 0x02, 0x6E, 0x6F, 0x6E, 0x65, 0x00, 0x69,   // 568..575
    0x6E, 0x36, 0x00, 0x00, 0xA0, 0x01, 0x01, 0x00,   // 576..583
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x80, 0x3F,   // 584..591
    0x43, 0x02, 0x6E, 0x6F, 0x6E, 0x65, 0x00, 0x69,   // 592..599
    0x6E, 0x37, 0x00, 0x00, 0xA0, 0x01, 0x01, 0x00,   // 600..607
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x80, 0x3F,   // 608..615
    0x5B, 0x02, 0x6E, 0x6F, 0x6E, 0x65, 0x00, 0x69,   // 616..623
    0x6E, 0x38, 0x00, 0x00, 0xA0, 0x01, 0x01, 0x00,   // 624..631
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x80, 0x3F,   // 632..639
    0x73, 0x02, 0x6E, 0x6F, 0x6E, 0x65, 0x00, 0x69,   // 640..647
    0x6E, 0x39, 0x00, 0x00, 0xA0, 0x01, 0x01, 0x00,   // 648..655
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x80, 0x3F,   // 656..663
    0x8B, 0x02, 0x6E, 0x6F, 0x6E, 0x65, 0x00, 0x69,   // 664..671
    0x6E, 0x31, 0x30, 0x00, 0x00, 0x00, 0x00, 0x00,   // 672..679
    0xA0, 0x01, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00,   // 680..687
    0x00, 0x00, 0x80, 0x3F, 0xA4, 0x02, 0x6E, 0x6F,   // 688..695
    0x6E, 0x65, 0x00, 0x69, 0x6E, 0x31, 0x31, 0x00,   // 696..703
    0x00, 0x00, 0x00, 0x00, 0xA0, 0x01, 0x01, 0x00,   // 704..711
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x80, 0x3F,   // 712..719
    0xC0, 0x02, 0x6E, 0x6F, 0x6E, 0x65, 0x00, 0x69,   // 720..727
    0x6E, 0x31, 0x32, 0x00, 0x00, 0x00, 0x00, 0x00,   // 728..735
    0xA0, 0x01, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00,   // 736..743
    0x00, 0x00, 0x80, 0x3F, 0xDC, 0x02, 0x6E, 0x6F,   // 744..751
    0x6E, 0x65, 0x00, 0x69, 0x6E, 0x31, 0x33, 0x00,   // 752..759
    0x00, 0x00, 0x00, 0x00, 0xA0, 0x01, 0x01, 0x00,   // 760..767
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x80, 0x3F,   // 768..775
    0xF8, 0x02, 0x6E, 0x6F, 0x6E, 0x65, 0x00, 0x69,   // 776..783
    0x6E, 0x31, 0x34, 0x00, 0x00, 0x00, 0x00, 0x00,   // 784..791
    0xA0, 0x01, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00,   // 792..799
    0x00, 0x00, 0x80, 0x3F, 0x14, 0x03, 0x6E, 0x6F,   // 800..807
    0x6E, 0x65, 0x00, 0x69, 0x6E, 0x31, 0x35, 0x00,   // 808..815
    0x00, 0x00, 0x00, 0x00, 0xA0, 0x01, 0x01, 0x00,   // 816..823
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x80, 0x3F,   // 824..831
    0x30, 0x03, 0x6E, 0x6F, 0x6E, 0x65, 0x00, 0x69,   // 832..839
    0x6E, 0x31, 0x36, 0x00, 0x00, 0x00, 0x00, 0x00,   // 840..847
    0xA0, 0x01, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00,   // 848..855
    0x00, 0x00, 0x80, 0x3F, 0x4C, 0x03, 0x6E, 0x6F,   // 856..863
    0x6E, 0x65, 0x00, 0x61, 0x6C, 0x61, 0x72, 0x6D,   // 864..871
    0x00, 0x00, 0x00, 0x00, 0xA0, 0x01, 0x01, 0x00,   // 872..879
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x80, 0x3F,   // 880..887
    0x69, 0x03, 0x6E, 0x6F, 0x6E, 0x65, 0x00, 0x6F,   // 888..895
    0x6B, 0x00, 0x00, 0x00, 0xA0, 0x01, 0x01, 0x00,   // 896..903
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x80, 0x3F,   // 904..911
    0x82, 0x03, 0x6E, 0x6F, 0x6E, 0x65, 0x00, 0x6D,   // 912..919
    0x6F, 0x74, 0x6F, 0x72, 0x73, 0x74, 0x61, 0x72,   // 920..927
    0x74, 0x00, 0x00, 0x00, 0xA0, 0x01, 0x01, 0x00,   // 928..935
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x80, 0x3F,   // 936..943
    0xA2, 0x03, 0x6E, 0x6F, 0x6E, 0x65, 0x00, 0x70,   // 944..951
    0x72, 0x6F, 0x67, 0x72, 0x61, 0x6D, 0x6D, 0x73,   // 952..959
    0x74, 0x61, 0x72, 0x74, 0x00, 0x00, 0x00, 0x00,   // 960..967
    0xA0, 0x01, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00,   // 968..975
    0x00, 0x00, 0x80, 0x3F, 0xC5, 0x03, 0x6E, 0x6F,   // 976..983
    0x6E, 0x65, 0x00, 0x61, 0x75, 0x73, 0x77, 0x61,   // 984..991
    0x68, 0x6C, 0x78, 0x00, 0x00, 0x00, 0x00, 0x00,   // 992..999
    0xA0, 0x01, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00,   // 1000..1007
    0x00, 0x00, 0x80, 0x3F, 0xE4, 0x03, 0x6E, 0x6F,   // 1008..1015
    0x6E, 0x65, 0x00, 0x61, 0x75, 0x73, 0x77, 0x61,   // 1016..1023
    0x68, 0x6C, 0x79, 0x00, 0x00, 0x00, 0x00, 0x00,   // 1024..1031
    0xA0, 0x01, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00,   // 1032..1039
    0x00, 0x00, 0x80, 0x3F, 0x04, 0x04, 0x6E, 0x6F,   // 1040..1047
    0x6E, 0x65, 0x00, 0x61, 0x75, 0x73, 0x77, 0x61,   // 1048..1055
    0x68, 0x6C, 0x7A, 0x00, 0x00, 0x00, 0x00, 0x00,   // 1056..1063
    0xA0, 0x01, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00,   // 1064..1071
    0x00, 0x00, 0x80, 0x3F, 0x24, 0x04, 0x6E, 0x6F,   // 1072..1079
    0x6E, 0x65, 0x00, 0x73, 0x70, 0x65, 0x65, 0x64,   // 1080..1087
    0x31, 0x00, 0x00, 0x00, 0xA0, 0x01, 0x01, 0x00,   // 1088..1095
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x80, 0x3F,   // 1096..1103
    0x42, 0x04, 0x6E, 0x6F, 0x6E, 0x65, 0x00, 0x73,   // 1104..1111
    0x70, 0x65, 0x65, 0x64, 0x32, 0x00, 0xB0, 0x00,   // 1112..1119
    0x01, 0x00, 0x50, 0x6F, 0x73, 0x69, 0x74, 0x69,   // 1120..1127
    0x6F, 0x6E, 0x20, 0x6D, 0x6F, 0x64, 0x65, 0x00,   // 1128..1135
    0x00, 0x00, 0x00, 0x00, 0xA0, 0x07, 0x00, 0x00,   // 1136..1143
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,   // 1144..1151
    0x70, 0x04, 0x00, 0x70, 0x61, 0x64, 0x64, 0x69,   // 1152..1159
    0x6E, 0x67, 0x00, 0x00, 0xA0, 0x06, 0x00, 0x80,   // 1160..1167
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,   // 1168..1175
    0x8B, 0x04, 0x00, 0x70, 0x61, 0x64, 0x64, 0x69,   // 1176..1183
    0x6E, 0x67, 0x00, 0x0C, 0x00, 0x28, 0x00, 0x44,   // 1184..1191
    0x00, 0x60, 0x00, 0x7C, 0x00, 0x98, 0x00, 0xB4,   // 1192..1199
    0x00, 0xD0, 0x00, 0xEC, 0x00, 0x04, 0x01, 0x20,   // 1200..1207
    0x01, 0x3C, 0x01, 0x58, 0x01, 0x74, 0x01, 0x94,   // 1208..1215
    0x01, 0xB4, 0x01, 0xCC, 0x01, 0xE4, 0x01, 0xFC,   // 1216..1223
    0x01, 0x14, 0x02, 0x2C, 0x02, 0x44, 0x02, 0x5C,   // 1224..1231
    0x02, 0x74, 0x02, 0x8C, 0x02, 0xA8, 0x02, 0xC4,   // 1232..1239
    0x02, 0xE0, 0x02, 0xFC, 0x02, 0x18, 0x03, 0x34,   // 1240..1247
    0x03, 0x50, 0x03, 0x6C, 0x03, 0x84, 0x03, 0xA4,   // 1248..1255
    0x03, 0xC8, 0x03, 0xE8, 0x03, 0x08, 0x04, 0x28,   // 1256..1263
    0x04, 0x44, 0x04, 0x74, 0x04, 0x8C, 0x04, 0x00,   // 1264..1271
    0x00, 0x5E, 0x04, 0x00, 0x00,
};

const discovery_rpc_t discovery = {
    .input  = 10,
    .output = 2,
    .ptocp  = 0x04A3,
    .gtocp  = 0x04F9,
};

typedef struct {
    uint32_t out1 : 1;
    uint32_t out2 : 1;
    uint32_t out3 : 1;
    uint32_t out4 : 1;
    uint32_t out5 : 1;
    uint32_t out6 : 1;
    uint32_t out7 : 1;
    uint32_t out8 : 1;
    uint32_t ena : 1;
    uint32_t spindel : 1;
    uint32_t padding : 6;
} sserial_out_process_data_t;   // size:2 bytes

typedef struct {
    int8_t   joy_x;
    int8_t   joy_y;
    int8_t   joy_z;
    uint8_t  feedrate;
    uint8_t  rotation;
    uint32_t in1 : 1;
    uint32_t in2 : 1;
    uint32_t in3 : 1;
    uint32_t in4 : 1;
    uint32_t in5 : 1;
    uint32_t in6 : 1;
    uint32_t in7 : 1;
    uint32_t in8 : 1;
    uint32_t in9 : 1;
    uint32_t in10 : 1;
    uint32_t in11 : 1;
    uint32_t in12 : 1;
    uint32_t in13 : 1;
    uint32_t in14 : 1;
    uint32_t in15 : 1;
    uint32_t in16 : 1;
    uint32_t alarm : 1;
    uint32_t ok : 1;
    uint32_t motorstart : 1;
    uint32_t programmstart : 1;
    uint32_t auswahlx : 1;
    uint32_t auswahly : 1;
    uint32_t auswahlz : 1;
    uint32_t speed1 : 1;
    uint32_t speed2 : 1;
    uint32_t padding : 7;
} sserial_in_process_data_t;   // size:9 bytes

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

void print_pd(process_data_descriptor_t *pd) {
    int   strl = strlen(&pd->names);
    char *unit = &pd->names;
    char *name = &pd->names + strl + 1;
    switch (pd->data_type) {
    case DATA_TYPE_PAD:
        Serial.printf("uint32_t padding : %u;\n", pd->data_size);
        break;
    case DATA_TYPE_BITS:
        if (pd->data_size == 1) {
            Serial.printf("uint32_t %s : 1;\n", name);
        } else {
            for (int i = 0; i < pd->data_size; i++) {
                Serial.printf("uint32_t %s_%i : 1;\n", name, i);
            }
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

    uint16_t ptoc[64];
    uint16_t gtoc[64];

    uint16_t                  *ptocp = ptoc;
    uint16_t                  *gtocp = gtoc;
    process_data_descriptor_t *last_pd;

    // ADD_PROCESS_VAR(("out", "none", 16, DATA_TYPE_BITS, DATA_DIRECTION_OUTPUT, -100, 100));
    ADD_PROCESS_VAR(("out1", "none", 1, DATA_TYPE_BITS, DATA_DIRECTION_OUTPUT, 0, 1));
    ADD_PROCESS_VAR(("out2", "none", 1, DATA_TYPE_BITS, DATA_DIRECTION_OUTPUT, 0, 1));
    ADD_PROCESS_VAR(("out3", "none", 1, DATA_TYPE_BITS, DATA_DIRECTION_OUTPUT, 0, 1));
    ADD_PROCESS_VAR(("out4", "none", 1, DATA_TYPE_BITS, DATA_DIRECTION_OUTPUT, 0, 1));
    ADD_PROCESS_VAR(("out5", "none", 1, DATA_TYPE_BITS, DATA_DIRECTION_OUTPUT, 0, 1));
    ADD_PROCESS_VAR(("out6", "none", 1, DATA_TYPE_BITS, DATA_DIRECTION_OUTPUT, 0, 1));
    ADD_PROCESS_VAR(("out7", "none", 1, DATA_TYPE_BITS, DATA_DIRECTION_OUTPUT, 0, 1));
    ADD_PROCESS_VAR(("out8", "none", 1, DATA_TYPE_BITS, DATA_DIRECTION_OUTPUT, 0, 1));
    ADD_PROCESS_VAR(("ena", "none", 1, DATA_TYPE_BITS, DATA_DIRECTION_OUTPUT, 0, 1));
    ADD_PROCESS_VAR(("spindel", "none", 1, DATA_TYPE_BITS, DATA_DIRECTION_OUTPUT, 0, 1));

    ADD_PROCESS_VAR(("joy_x", "none", 8, DATA_TYPE_SIGNED, DATA_DIRECTION_INPUT, -127, 127));
    ADD_PROCESS_VAR(("joy_y", "none", 8, DATA_TYPE_SIGNED, DATA_DIRECTION_INPUT, -127, 127));
    ADD_PROCESS_VAR(("joy_z", "none", 8, DATA_TYPE_SIGNED, DATA_DIRECTION_INPUT, -127, 127));
    ADD_PROCESS_VAR(("feedrate", "none", 8, DATA_TYPE_UNSIGNED, DATA_DIRECTION_INPUT, 0, 255));
    ADD_PROCESS_VAR(("rotation", "none", 8, DATA_TYPE_UNSIGNED, DATA_DIRECTION_INPUT, 0, 255));
    ADD_PROCESS_VAR(("in1", "none", 1, DATA_TYPE_BITS, DATA_DIRECTION_INPUT, 0, 1));
    ADD_PROCESS_VAR(("in2", "none", 1, DATA_TYPE_BITS, DATA_DIRECTION_INPUT, 0, 1));
    ADD_PROCESS_VAR(("in3", "none", 1, DATA_TYPE_BITS, DATA_DIRECTION_INPUT, 0, 1));
    ADD_PROCESS_VAR(("in4", "none", 1, DATA_TYPE_BITS, DATA_DIRECTION_INPUT, 0, 1));
    ADD_PROCESS_VAR(("in5", "none", 1, DATA_TYPE_BITS, DATA_DIRECTION_INPUT, 0, 1));
    ADD_PROCESS_VAR(("in6", "none", 1, DATA_TYPE_BITS, DATA_DIRECTION_INPUT, 0, 1));
    ADD_PROCESS_VAR(("in7", "none", 1, DATA_TYPE_BITS, DATA_DIRECTION_INPUT, 0, 1));
    ADD_PROCESS_VAR(("in8", "none", 1, DATA_TYPE_BITS, DATA_DIRECTION_INPUT, 0, 1));
    ADD_PROCESS_VAR(("in9", "none", 1, DATA_TYPE_BITS, DATA_DIRECTION_INPUT, 0, 1));
    ADD_PROCESS_VAR(("in10", "none", 1, DATA_TYPE_BITS, DATA_DIRECTION_INPUT, 0, 1));
    ADD_PROCESS_VAR(("in11", "none", 1, DATA_TYPE_BITS, DATA_DIRECTION_INPUT, 0, 1));
    ADD_PROCESS_VAR(("in12", "none", 1, DATA_TYPE_BITS, DATA_DIRECTION_INPUT, 0, 1));
    ADD_PROCESS_VAR(("in13", "none", 1, DATA_TYPE_BITS, DATA_DIRECTION_INPUT, 0, 1));
    ADD_PROCESS_VAR(("in14", "none", 1, DATA_TYPE_BITS, DATA_DIRECTION_INPUT, 0, 1));
    ADD_PROCESS_VAR(("in15", "none", 1, DATA_TYPE_BITS, DATA_DIRECTION_INPUT, 0, 1));
    ADD_PROCESS_VAR(("in16", "none", 1, DATA_TYPE_BITS, DATA_DIRECTION_INPUT, 0, 1));
    ADD_PROCESS_VAR(("alarm", "none", 1, DATA_TYPE_BITS, DATA_DIRECTION_INPUT, 0, 1));
    ADD_PROCESS_VAR(("ok", "none", 1, DATA_TYPE_BITS, DATA_DIRECTION_INPUT, 0, 1));
    ADD_PROCESS_VAR(("motorstart", "none", 1, DATA_TYPE_BITS, DATA_DIRECTION_INPUT, 0, 1));
    ADD_PROCESS_VAR(("programmstart", "none", 1, DATA_TYPE_BITS, DATA_DIRECTION_INPUT, 0, 1));
    ADD_PROCESS_VAR(("auswahlx", "none", 1, DATA_TYPE_BITS, DATA_DIRECTION_INPUT, 0, 1));
    ADD_PROCESS_VAR(("auswahly", "none", 1, DATA_TYPE_BITS, DATA_DIRECTION_INPUT, 0, 1));
    ADD_PROCESS_VAR(("auswahlz", "none", 1, DATA_TYPE_BITS, DATA_DIRECTION_INPUT, 0, 1));
    ADD_PROCESS_VAR(("speed1", "none", 1, DATA_TYPE_BITS, DATA_DIRECTION_INPUT, 0, 1));
    ADD_PROCESS_VAR(("speed2", "none", 1, DATA_TYPE_BITS, DATA_DIRECTION_INPUT, 0, 1));

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
        if ((pd->data_direction == DATA_DIRECTION_OUTPUT || pd->data_direction == DATA_DIRECTION_BI_DIRECTIONAL) &&
            pd->record_type == RECORD_TYPE_PROCESS_DATA_RECORD) {
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
        if ((pd->data_direction == DATA_DIRECTION_INPUT || pd->data_direction == DATA_DIRECTION_BI_DIRECTIONAL) &&
            pd->record_type == RECORD_TYPE_PROCESS_DATA_RECORD) {
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
    data_in.in1           = ioRegister.mcp0Data.bits.in1;
    data_in.in2           = ioRegister.mcp0Data.bits.in2;
    data_in.in3           = ioRegister.mcp0Data.bits.in3;
    data_in.in4           = ioRegister.mcp0Data.bits.in4;
    data_in.in5           = ioRegister.mcp0Data.bits.in5;
    data_in.in6           = ioRegister.mcp0Data.bits.in6;
    data_in.in7           = ioRegister.mcp0Data.bits.in7;
    data_in.in8           = ioRegister.mcp0Data.bits.in8;
    data_in.in9           = ioRegister.mcp0Data.bits.in9;
    data_in.in10          = ioRegister.mcp0Data.bits.in10;
    data_in.in11          = ioRegister.mcp0Data.bits.in11;
    data_in.in12          = ioRegister.mcp0Data.bits.in12;
    data_in.in13          = ioRegister.mcp0Data.bits.in13;
    data_in.in14          = ioRegister.mcp0Data.bits.in14;
    data_in.in15          = ioRegister.mcp0Data.bits.in15;
    data_in.in16          = ioRegister.mcp0Data.bits.in16;
    data_in.ok            = ioRegister.mcp1Data.bits.OK;
    data_in.motorstart    = ioRegister.mcp1Data.bits.motorStart;
    data_in.programmstart = ioRegister.mcp1Data.bits.programmStart;
    data_in.auswahlx      = ioRegister.mcp1Data.bits.auswahlX;
    data_in.auswahly      = ioRegister.mcp1Data.bits.auswahlY;
    data_in.auswahlz      = ioRegister.mcp1Data.bits.auswahlZ;
    data_in.speed1        = ioRegister.mcp1Data.bits.speed1;
    data_in.speed2        = ioRegister.mcp1Data.bits.speed2;
    data_in.alarm         = ioRegister.mcp1Data.bits.alarmAll;

    data_in.joy_x    = map(adcManager.getJoystickX(), 0, 4095, -127, 127);
    data_in.joy_y    = map(adcManager.getJoystickY(), 0, 4095, -127, 127);
    data_in.joy_z    = map(adcManager.getJoystickZ(), 0, 4095, -127, 127);
    data_in.feedrate = map(adcManager.getFeedrate(), 0, 4095, 0, 255);
    data_in.rotation = map(adcManager.getRotationSpeed(), 0, 4095, 0, 255);
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

    // ioRegister.setOutput(OutputPin::OUT2, data_out.out_2);
    // ioRegister.setOutput(OutputPin::OUT3, data_out.out_3);
    // ioRegister.setOutput(OutputPin::OUT4, data_out.out_4);
    // ioRegister.setOutput(OutputPin::OUT5, data_out.out_5);
    // ioRegister.setOutput(OutputPin::OUT6, data_out.out_6);
    // ioRegister.setOutput(OutputPin::OUT7, data_out.out_7);
    // ioRegister.setOutput(OutputPin::OUT8, data_out.out_8);
    // ioRegister.setOutput(OutputPin::SPINDEL_ON_OFF, data_out.out_9);
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