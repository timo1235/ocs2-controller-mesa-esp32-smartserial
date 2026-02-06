# Teststrategie OCS2-Controller

Konkrete Teststrategie fuer das OCS2 SmartSerial-Projekt. Definiert Frameworks, Architektur-Entscheidungen und Umsetzungsschritte.

## Entscheidungen

| Thema | Entscheidung |
|---|---|
| Native Tests (Host-PC) | **GoogleTest + GMock** |
| Embedded Tests (ESP32) | **Unity** (PlatformIO-Standard) |
| Mocking-Strategie | **HAL-Interfaces** mit Dependency Injection + GMock |
| Statische Analyse | **Cppcheck** (bereits konfiguriert, erweitern) |
| CI/CD | **GitHub Actions** (Native Tests + Build + Analyse) |
| HIL-Tests | Nicht geplant |

---

## 1. Testbare Projektstruktur

### 1.1 Verzeichnislayout

```
ocs2-controller-mesa-esp32-smartserial/
├── .github/workflows/ci.yml
├── lib/
│   └── hal/                          # HAL-Interfaces (neu)
│       ├── ISerialPort.h
│       ├── IExpanderDriver.h
│       └── IADCReader.h
├── src/
│   ├── crc8.cpp / crc8.h            # Nativ testbar (kein Arduino)
│   ├── sserial.h                     # LBP-Typen/Defines nativ testbar
│   ├── sserial_internal.h            # Prozessdaten-Structs nativ testbar
│   ├── sserial.cpp                   # Braucht Serial-HAL fuer Tests
│   ├── sserial_handlers.cpp          # Braucht Serial-HAL fuer Tests
│   ├── sserial_io.cpp                # Braucht IORegister/ADC-HAL
│   ├── ioRegister.cpp / ioRegister.h # Braucht MCP23S17-HAL
│   ├── ADCManager.cpp / ADCManager.h # Braucht analogRead-HAL
│   └── main.cpp
├── test/
│   ├── test_native/                  # Laufen auf Host-PC (schnell, CI)
│   │   ├── stubs/
│   │   │   ├── Arduino.h             # Minimaler Arduino-Stub
│   │   │   └── freertos_stubs.h      # portMUX etc. als No-Op
│   │   ├── test_crc8/
│   │   │   └── test_crc8.cpp
│   │   ├── test_lbp_types/
│   │   │   └── test_lbp_types.cpp
│   │   ├── test_bitfields/
│   │   │   └── test_bitfields.cpp
│   │   └── test_protocol/
│   │       └── test_protocol.cpp
│   └── test_embedded/                # Laufen auf ESP32 (Board noetig)
│       └── test_io_integration/
│           └── test_io_integration.cpp
└── platformio.ini
```

### 1.2 platformio.ini Konfiguration

```ini
[platformio]
default_envs = esp32doit-devkit-v1

[env]
platform = espressif32
framework = arduino
monitor_speed = 115200
build_flags =
    -Wall
    -Wextra
    -Wno-unused-parameter
lib_deps =
    adafruit/Adafruit ADS1X15@~2.4.2
    adafruit/Adafruit MCP23017 Arduino Library@^2.3.2
    robtillaart/MCP23S17@^0.5.2
check_tool = cppcheck
check_skip_packages = yes
check_flags =
    cppcheck: --enable=warning,performance,portability,style
              --suppress=missingIncludeSystem
              --inline-suppr
check_src_filters = +<src/>

; === Produktion ===
[env:esp32doit-devkit-v1]
board = esp32doit-devkit-v1
test_filter = test_embedded/*

; === Native Tests ===
[env:native]
platform = native
test_framework = googletest
build_flags =
    -std=gnu++17
    -DNATIVE_BUILD
test_filter = test_native/*
; Arduino-Hardware-Libraries nicht einbinden
lib_ignore =
    Adafruit ADS1X15
    Adafruit MCP23017 Arduino Library
    MCP23S17
; Stub-Verzeichnis als Include-Pfad fuer Arduino.h etc.
lib_extra_dirs = test/test_native/stubs
; Nur hardwareunabhaengige Source-Files kompilieren
build_src_filter =
    +<crc8.cpp>
```

**Wichtig:** `build_src_filter` muss erweitert werden, sobald weitere Source-Files nativ kompilierbar sind (nach HAL-Refactoring).

---

## 2. Native Tests (GoogleTest + GMock)

### 2.1 Was nativ getestet wird

| Testbereich | Source-Files | Abhaengigkeiten |
|---|---|---|
| CRC8-Berechnung | `crc8.cpp`, `crc8.h` | Keine (reines C) |
| LBP-Typen | `sserial.h` | Keine (nur Typen/Defines) |
| Prozessdaten-Structs | `sserial_internal.h` | Keine (Bitfelder, `sizeof`) |
| Bitfeld-Unions | `ioRegister.h` | Keine (`Mcp0Bits`, `Mcp1Bits`) |
| Protokoll-Handler | `sserial_handlers.cpp` | `ISerialPort` (HAL, Phase 2) |
| Prozessdaten-Mapping | `sserial_io.cpp` | `IExpanderDriver`, `IADCReader` (HAL, Phase 2) |

### 2.2 Stubs fuer native Kompilierung

Minimale Header, damit Typen aus Arduino/FreeRTOS auf dem Host-PC verfuegbar sind:

**`test/test_native/stubs/Arduino.h`:**
```cpp
#pragma once
#include <stdint.h>
#include <stdlib.h>
#include <string.h>

#define HIGH 1
#define LOW  0

typedef uint8_t byte;
inline unsigned long millis() { return 0; }
inline unsigned long micros() { return 0; }
```

**`test/test_native/stubs/freertos_stubs.h`:**
```cpp
#pragma once
#include <stdint.h>

typedef struct {} portMUX_TYPE;
#define portMUX_INITIALIZER_UNLOCKED {}
#define portENTER_CRITICAL(mux)
#define portEXIT_CRITICAL(mux)
```

### 2.3 Konkrete Testfaelle - Phase 1 (ohne HAL-Refactoring)

**CRC8:**
```cpp
// test/test_native/test_crc8/test_crc8.cpp
#include <gtest/gtest.h>
#include "crc8.h"

TEST(CRC8, InitReturnsZero) {
    EXPECT_EQ(crc8_init(), 0x00);
}

TEST(CRC8, FinalizeIsIdentity) {
    // XorOut = 0x00, also aendert finalize nichts
    EXPECT_EQ(crc8_finalize(0x42), 0x42);
}

TEST(CRC8, KnownLBPCookieResponse) {
    // LBPCookie-Antwort: Daten = {0x5A}, CRC muss berechenbar sein
    uint8_t data[] = {0x5A};
    crc8_t crc = crc8_init();
    crc = crc8_update(crc, data, 1);
    crc = crc8_finalize(crc);
    // CRC ist deterministisch - Wert als Referenz festhalten
    EXPECT_NE(crc, 0x00);  // Nicht Null fuer nicht-leere Daten
}

TEST(CRC8, EmptyDataReturnsSeed) {
    crc8_t crc = crc8_init();
    crc = crc8_update(crc, nullptr, 0);
    EXPECT_EQ(crc8_finalize(crc), 0x00);
}

TEST(CRC8, SingleByteMatchesTable) {
    // Fuer Poly=0x31 ReflectIn=True: CRC(0x00) = Tabelleneintrag 0
    uint8_t data[] = {0x00};
    crc8_t crc = crc8_init();
    crc = crc8_update(crc, data, 1);
    EXPECT_EQ(crc8_finalize(crc), 0x00);
}

TEST(CRC8, MultiByteConsistency) {
    // Gleiches Ergebnis bei schrittweiser vs. einmaliger Berechnung
    uint8_t data[] = {0xDF, 0x5A, 0x01, 0x02};
    crc8_t crc_full = crc8_init();
    crc_full = crc8_update(crc_full, data, 4);

    crc8_t crc_step = crc8_init();
    crc_step = crc8_update(crc_step, data, 2);
    crc_step = crc8_update(crc_step, data + 2, 2);

    EXPECT_EQ(crc8_finalize(crc_full), crc8_finalize(crc_step));
}
```

**LBP-Typen und Bitfelder:**
```cpp
// test/test_native/test_lbp_types/test_lbp_types.cpp
#include <gtest/gtest.h>
#include "sserial.h"

TEST(LBPTypes, CommandTypeParsing) {
    lbp_t cmd;
    // CT_LOCAL = 3 = 0b11 in bits [7:6]
    cmd.byte = 0xDF;  // LBPCookieCMD
    EXPECT_EQ(cmd.ct, CT_LOCAL);
    EXPECT_EQ(cmd.wr, 0);  // Read
}

TEST(LBPTypes, RWCommandFields) {
    lbp_t cmd;
    // Byte 0x45 = 0b01_0_0_0_1_01
    // ct=01(RW), wr=0(read), ai=0, as=0, rid=0, ds=01(2 bytes)
    cmd.byte = 0x45;
    EXPECT_EQ(cmd.ct, CT_RW);
    EXPECT_EQ(cmd.wr, 0);
    EXPECT_EQ(cmd.ds, 1);  // 2 bytes
    EXPECT_EQ(cmd.as, 0);
    EXPECT_EQ(cmd.ai, 0);
}

TEST(LBPTypes, RPCCommandByte) {
    lbp_t cmd;
    cmd.byte = DiscoveryRPC;
    EXPECT_EQ(cmd.ct, CT_RPC);

    cmd.byte = UnitNumberRPC;
    EXPECT_EQ(cmd.ct, CT_RPC);

    cmd.byte = ProcessDataRPC;
    EXPECT_EQ(cmd.ct, CT_RPC);
}

TEST(LBPTypes, WriteCommandBit) {
    lbp_t cmd;
    cmd.byte = 0x65;  // RW write: ct=01, wr=1
    EXPECT_EQ(cmd.ct, CT_RW);
    EXPECT_EQ(cmd.wr, 1);
}

TEST(LBPTypes, DataSizeDecoding) {
    lbp_t cmd;
    cmd.byte = 0x44;  // ds=00 -> 1 byte
    EXPECT_EQ(1 << cmd.ds, 1);

    cmd.byte = 0x45;  // ds=01 -> 2 bytes
    EXPECT_EQ(1 << cmd.ds, 2);

    cmd.byte = 0x46;  // ds=10 -> 4 bytes
    EXPECT_EQ(1 << cmd.ds, 4);
}

TEST(LBPTypes, DiscoveryRPCSize) {
    EXPECT_EQ(sizeof(discovery_rpc_t), 6);
}

TEST(LBPTypes, UnitNumberSize) {
    EXPECT_EQ(sizeof(unit_no_t), 4);
}
```

**Prozessdaten-Structs:**
```cpp
// test/test_native/test_bitfields/test_bitfields.cpp
#include <gtest/gtest.h>
#include <cstring>

// Nur die Struct-Definitionen einbinden (nicht sserial_internal.h,
// da der externe Abhaengigkeiten hat). Stattdessen direkt:
#include "sserial.h"

// Structs hier dupliziert oder ueber einen separaten Header verfuegbar machen.
// Fuer Phase 1 direkt testen:
#pragma pack(push, 1)
typedef struct {
    uint16_t out1 : 1;
    uint16_t out2 : 1;
    uint16_t out3 : 1;
    uint16_t out4 : 1;
    uint16_t out5 : 1;
    uint16_t out6 : 1;
    uint16_t out7 : 1;
    uint16_t out8 : 1;
    uint16_t ena : 1;
    uint16_t spindel : 1;
    uint16_t padding : 6;
} test_out_t;
static_assert(sizeof(test_out_t) == 2, "Output struct must be 2 bytes");

typedef struct {
    int8_t  joy_x;
    int8_t  joy_y;
    int8_t  joy_z;
    uint8_t feedrate;
    uint8_t rotation;
    uint8_t in1 : 1;  uint8_t in2 : 1;  uint8_t in3 : 1;  uint8_t in4 : 1;
    uint8_t in5 : 1;  uint8_t in6 : 1;  uint8_t in7 : 1;  uint8_t in8 : 1;
    uint8_t in9 : 1;  uint8_t in10 : 1; uint8_t in11 : 1; uint8_t in12 : 1;
    uint8_t in13 : 1; uint8_t in14 : 1; uint8_t in15 : 1; uint8_t in16 : 1;
    uint8_t alarm : 1;
    uint8_t ok : 1;
    uint8_t motorstart : 1;
    uint8_t programmstart : 1;
    uint8_t auswahlx : 1;
    uint8_t auswahly : 1;
    uint8_t auswahlz : 1;
    uint8_t speed1 : 1;
    uint8_t speed2 : 1;
    uint8_t padding : 7;
} test_in_t;
static_assert(sizeof(test_in_t) == 9, "Input struct must be 9 bytes");
#pragma pack(pop)

TEST(Bitfields, OutputStructSize) {
    EXPECT_EQ(sizeof(test_out_t), 2);
}

TEST(Bitfields, InputStructSize) {
    EXPECT_EQ(sizeof(test_in_t), 9);
}

TEST(Bitfields, OutputBitOrder) {
    test_out_t out;
    memset(&out, 0, sizeof(out));
    out.out1 = 1;
    uint16_t raw;
    memcpy(&raw, &out, 2);
    EXPECT_EQ(raw & 0x01, 1);  // out1 ist Bit 0
}

TEST(Bitfields, InputJoystickRange) {
    test_in_t in;
    memset(&in, 0, sizeof(in));
    in.joy_x = -128;
    EXPECT_EQ(in.joy_x, -128);
    in.joy_x = 127;
    EXPECT_EQ(in.joy_x, 127);
}

TEST(Bitfields, OutputDeserialization) {
    // Simuliert empfangene 2 Bytes von Mesa: out1=1, ena=1
    uint8_t raw[] = {0x01, 0x01};  // Bit 0 (out1) + Bit 8 (ena)
    test_out_t out;
    memcpy(&out, raw, 2);
    EXPECT_EQ(out.out1, 1);
    EXPECT_EQ(out.out2, 0);
    EXPECT_EQ(out.ena, 1);
    EXPECT_EQ(out.spindel, 0);
}
```

---

## 3. HAL-Interfaces (Phase 2)

Um die Protokoll-Handler und I/O-Logik nativ testen zu koennen, muessen Hardware-Abhaengigkeiten hinter Interfaces versteckt werden.

### 3.1 Benoetigte Interfaces

**`lib/hal/ISerialPort.h`** - Abstrahiert `Serial1` (UART zum Mesa-Board):
```cpp
#pragma once
#include <stdint.h>
#include <stddef.h>

class ISerialPort {
public:
    virtual ~ISerialPort() = default;
    virtual int    available() = 0;
    virtual int    read() = 0;
    virtual size_t write(const uint8_t* buf, size_t len) = 0;
};
```

**Betroffene Dateien:** `sserial.cpp` (`send()`, `emptySerialBuffer()`, `waitForBytes()`), `sserial_handlers.cpp` (alle Handler), `sserial_io.cpp` (`processIncomingData()`).

**`lib/hal/IExpanderDriver.h`** - Abstrahiert MCP23S17:
```cpp
#pragma once
#include <stdint.h>

class IExpanderDriver {
public:
    virtual ~IExpanderDriver() = default;
    virtual uint16_t readAll() = 0;
    virtual void     writeAll(uint16_t value) = 0;
};
```

**Betroffene Dateien:** `ioRegister.cpp` (MCP0, MCP1, MCP2 Zugriffe).

**`lib/hal/IADCReader.h`** - Abstrahiert `analogRead()`:
```cpp
#pragma once
#include <stdint.h>

class IADCReader {
public:
    virtual ~IADCReader() = default;
    virtual int16_t readChannel(uint8_t channel) = 0;
};
```

**Betroffene Datei:** `ADCManager.cpp` (`analogRead()` Aufrufe).

### 3.2 Refactoring-Schritte

1. HAL-Header unter `lib/hal/` anlegen
2. `IORegister` Konstruktor erhaelt `IExpanderDriver&` statt direkt `MCP23S17` zu instanziieren
3. `ADCManager` Konstruktor erhaelt `IADCReader&` statt direkt `analogRead()` aufzurufen
4. SmartSerial-Funktionen erhalten `ISerialPort&` (entweder per Konstruktor oder als Modul-Init)
5. Produktions-Implementierungen: `ESP32SerialPort`, `MCP23S17Driver`, `ESP32ADCReader`
6. In `main.cpp` die konkreten Implementierungen erstellen und injizieren

### 3.3 Worauf achten

- **Keine virtuelle Dispatch-Kosten im Hot-Path:** Die `ISerialPort`-Aufrufe in den Protokoll-Handlern sind zeitkritisch. Optionen:
  - Template-basierte Injection (kein vtable-Overhead, aber komplexer)
  - `#ifdef NATIVE_BUILD` fuer Interface vs. direkte Aufrufe
  - Pragmatisch: vtable-Overhead ist ~5ns pro Call, bei 2.5 MBaud (4µs/Byte) vernachlaessigbar
- **FreeRTOS-Stubs:** Tasks und Spinlocks werden in nativen Tests durch No-Ops ersetzt. Thread-Safety wird nur auf dem ESP32 getestet.
- **`build_src_filter` erweitern:** Nach Refactoring koennen `sserial_handlers.cpp`, `sserial_io.cpp` etc. in die native Umgebung aufgenommen werden.

---

## 4. Statische Analyse

### 4.1 Cppcheck (aktuelle Konfiguration erweitern)

Die bestehende Konfiguration in `platformio.ini` um `style`-Checks erweitern:

```ini
check_flags =
    cppcheck: --enable=warning,performance,portability,style
              --suppress=missingIncludeSystem
              --suppress=unusedFunction
              --inline-suppr
```

**Ausfuehrung:**
```bash
# Vollstaendiger Check
pio check --skip-packages

# Nur High-Severity (fuer CI)
pio check --skip-packages --fail-on-defect=high
```

**Inline-Unterdrueckung** wo noetig:
```cpp
// cppcheck-suppress unusedFunction
crc8_t crc8_reflect(crc8_t data, size_t data_len) { ... }
```

---

## 5. CI/CD Pipeline (GitHub Actions)

### 5.1 Workflow

```yaml
# .github/workflows/ci.yml
name: CI

on:
  push:
    branches: [main]
  pull_request:
    branches: [main]

jobs:
  native-tests:
    runs-on: ubuntu-latest
    steps:
      - uses: actions/checkout@v4

      - uses: actions/cache@v4
        with:
          path: |
            ~/.cache/pip
            ~/.platformio/.cache
            ~/.platformio/packages
          key: ${{ runner.os }}-pio-${{ hashFiles('platformio.ini') }}
          restore-keys: ${{ runner.os }}-pio-

      - uses: actions/setup-python@v5
        with:
          python-version: '3.11'

      - name: Install PlatformIO
        run: pip install --upgrade platformio

      - name: Run native tests
        run: pio test -e native --verbose

  build:
    runs-on: ubuntu-latest
    steps:
      - uses: actions/checkout@v4

      - uses: actions/cache@v4
        with:
          path: |
            ~/.cache/pip
            ~/.platformio/.cache
            ~/.platformio/packages
          key: ${{ runner.os }}-pio-${{ hashFiles('platformio.ini') }}
          restore-keys: ${{ runner.os }}-pio-

      - uses: actions/setup-python@v5
        with:
          python-version: '3.11'

      - name: Install PlatformIO
        run: pip install --upgrade platformio

      - name: Build firmware
        run: pio run -e esp32doit-devkit-v1

  static-analysis:
    runs-on: ubuntu-latest
    steps:
      - uses: actions/checkout@v4

      - uses: actions/cache@v4
        with:
          path: |
            ~/.cache/pip
            ~/.platformio/.cache
            ~/.platformio/packages
          key: ${{ runner.os }}-pio-${{ hashFiles('platformio.ini') }}
          restore-keys: ${{ runner.os }}-pio-

      - uses: actions/setup-python@v5
        with:
          python-version: '3.11'

      - name: Install PlatformIO
        run: pip install --upgrade platformio

      - name: Run cppcheck
        run: pio check --skip-packages --fail-on-defect=high
```

### 5.2 Branch Protection

Nach Einrichtung der CI-Pipeline in den GitHub Repository Settings aktivieren:
- **Require status checks before merging:** `native-tests`, `build`, `static-analysis`
- **Require branches to be up to date before merging**

---

## 6. Umsetzungsplan

### Phase 1: Sofort (kein Refactoring noetig)

1. `[env:native]` in `platformio.ini` einrichten
2. Stubs erstellen (`Arduino.h`, `freertos_stubs.h`)
3. Tests schreiben:
   - `test_crc8` - CRC8-Berechnung gegen bekannte Werte
   - `test_lbp_types` - LBP Command Parsing, Bitfelder, Struct-Sizes
   - `test_bitfields` - Prozessdaten De-/Serialisierung, IO-Bitfeld-Unions
4. `pio test -e native` lokal verifizieren
5. GitHub Actions CI-Workflow einrichten

### Phase 2: HAL-Interfaces + erweiterte Tests

1. `ISerialPort`, `IExpanderDriver`, `IADCReader` Interfaces anlegen
2. `IORegister` refactoren (DI statt globaler MCP-Objekte)
3. `ADCManager` refactoren (DI statt `analogRead`)
4. SmartSerial-Modul refactoren (DI statt `Serial1`)
5. Mock-Klassen mit GMock erstellen
6. Tests schreiben:
   - `test_protocol` - Handler-Logik mit gemocktem Serial
   - `test_ioregister` - Output-Bit-Mapping mit gemocktem Expander
   - `test_process_data` - Analog-Konvertierung mit gemocktem ADC
7. `build_src_filter` in `[env:native]` erweitern

### Phase 3: CI-Erweiterungen

1. Code-Coverage mit `lcov`/`gcov` + Codecov
2. Cppcheck `style`-Checks aktivieren
3. Branch Protection Rules einrichten
