# OCS2 Controller Mesa ESP32 SmartSerial - Detaillierter Verbesserungsplan

## Inhaltsverzeichnis

1. [Projekt-Analyse](#1-projekt-analyse)
2. [Protokoll-Referenz (Mesa SmartSerial / LBP)](#2-protokoll-referenz)
3. [Kritische Bugs](#3-kritische-bugs)
4. [Architektur-Verbesserungen](#4-architektur-verbesserungen)
5. [Modul-Refactoring](#5-modul-refactoring)
6. [Build-System und Tooling](#6-build-system-und-tooling)
7. [Priorisierte Umsetzungsreihenfolge](#7-priorisierte-umsetzungsreihenfolge)

---

## 1. Projekt-Analyse

### Was dieses Projekt tut

ESP32-basierter CNC-Controller (Pendant), der ueber Mesa SmartSerial (LBP-Protokoll, 2.5 Mbit/s RS-422) mit einer Mesa FPGA-Karte (z.B. 7i92, 7i96) kommuniziert. Er meldet sich als Remote-I/O-Geraet bei LinuxCNC an und uebertraegt:

- **Eingaben an LinuxCNC (10 Bytes):** Joystick X/Y/Z, Feedrate, Rotation, 16 Digital-Eingaenge, Statusbits (Alarm, OK, Motorstart, etc.)
- **Ausgaben von LinuxCNC (2 Bytes):** 8 Digital-Ausgaenge, Enable, Spindel On/Off

### Architektur-Uebersicht

```
                    Mesa FPGA Karte (7i92/7i96)
                           |
                    RS-422 @ 2.5 Mbit/s
                           |
              +------------+------------+
              |    ESP32 (Dual Core)    |
              |                         |
              |  Core 1: sserialLoop()  |  <-- Prio 10, LBP State Machine
              |                         |
              |  Core 0: 4 Tasks        |
              |   - inputUpdateTask     |  <-- MCP23S17 SPI lesen (1ms)
              |   - outputUpdateTask    |  <-- MCP23S17 SPI schreiben (1ms)
              |   - readInputsTask      |  <-- ADC lesen (5ms)
              |   - debugSerialTask     |  <-- Serial Debug Output
              +----+-------+-------+----+
                   |       |       |
              MCP23S17x3  ADC    Serial1
              (SPI)      (GPIO)  (UART 2.5M)
```

### Quellcode-Metriken

| Datei | Zeilen | Aufgabe |
|-------|--------|---------|
| `sserial.cpp` | 785 | SmartSerial Protokoll + State Machine |
| `sserial.h` | 154 | Protokoll-Definitionen, Datenstrukturen |
| `ioRegister.cpp` | 97 | MCP23S17 SPI I/O Expander |
| `ioRegister.h` | 74 | I/O Register Definitionen |
| `ADCManager.cpp` | 46 | Analog-Eingaenge (Joystick, Potis) |
| `ADCManager.h` | 34 | ADC Interface |
| `debugHelper.cpp` | 91 | FreeRTOS Queue-basiertes Debug |
| `debugHelper.h` | 43 | Debug Interface |
| `main.cpp` | 35 | Setup + leerer Loop |
| `pinmap.h` | 97 | GPIO Pin-Zuordnung |
| `includes.h` | 35 | Globale Includes + Externals |
| **Gesamt** | **~1.490** | |

---

## 2. Protokoll-Referenz

### Quellen

Die folgenden offiziellen Mesa-Handbuecher dokumentieren das SmartSerial/LBP-Protokoll:

- [7i76 Manual](https://www.mesanet.com/pdf/parallel/7i76man.pdf) - PTOC/GTOC Seiten 29-31, LBP ab Seite 33
- [7i76E Manual](https://www.mesanet.com/pdf/parallel/7i76eman.pdf) - LBP Befehle ab Seite 54
- [7i69 Manual](https://www.mesanet.com/pdf/parallel/7i69man.pdf) - Vollstaendige LBP Spec, Seiten 38-43
- [LinuxCNC hostmot2 sserial.c](https://github.com/LinuxCNC/linuxcnc/blob/master/src/hal/drivers/mesa-hostmot2/sserial.c)
- [STMBL sserial.c](https://github.com/rene-dev/stmbl/blob/master/src/comps/sserial.c) - Referenz-Slave-Implementierung

### Referenz-Implementierungen

| Projekt | MCU | Besonderheiten |
|---------|-----|----------------|
| [fdarling/mesa-smartserial-device-template](https://github.com/fdarling/mesa-smartserial-device-template) | Teensy 3.2 | Saubere Referenzimplementierung |
| [rene-dev/stmbl](https://github.com/rene-dev/stmbl) | STM32F4 | DMA-basiert, 20kHz, Servo-Antrieb |
| [multigcs/SmartSerial-ESP32RelayX8Board](https://github.com/multigcs/SmartSerial-ESP32RelayX8Board) | ESP32 | 8-Kanal Relay, PlatformIO |

### Protokoll-Zusammenfassung

```
LBP Command Byte:
  Bit 7:6 = ct  (CommandType: 01=RW, 10=RPC, 11=Local)
  Bit 5   = wr  (Write=1, Read=0)
  Bit 4   = rid (RPCIncludesData)
  Bit 3   = ai  (AutoIncrement)
  Bit 2   = as  (AddressSize: 0=current, 1=2-byte address folgt)
  Bit 1:0 = ds  (DataSize: 00=1B, 01=2B, 10=4B, 11=8B)

Discovery Sequence:
  1. Mesa sendet Cookie (0xDF) -> Slave antwortet 0x5A
  2. Mesa sendet UnitNumberRPC (0xBC) -> Slave antwortet 4-byte ID
  3. Mesa sendet DiscoveryRPC (0xBB) -> Slave antwortet 6-byte Discovery
  4. Mesa liest Deskriptor-Tabelle via RW-Befehle
  5. Mesa beginnt zyklischen ProcessDataRPC (0xBD) Austausch

CRC-8: Polynom 0x31 (Dallas/Maxim), ReflectIn/Out, XorIn/Out=0x00
Baudrate: 2.500.000 (2.5 Mbit/s), 8N1
```

---

## 3. Kritische Bugs

### BUG-01: Fehlende Bounds-Checks in handleRead() [KRITISCH]

**Datei:** `sserial.cpp:618`
```cpp
memcpy((void *) txbuf, &sserial_slave[address], (1 << lbp.ds));
```

**Problem:** `address` kommt direkt vom seriellen Bus (Zeile 614-615), wird aber nicht gegen `sizeof(sserial_slave)` geprueft. Ein ungueltiger Wert liest ausserhalb des Arrays.

**Vergleich:** `handleWrite()` (Zeile 634) hat diesen Check:
```cpp
if ((address + (1 << lbp.ds)) < ARRAY_SIZE(sserial_slave))
```
Aber `handleRead()` hat ihn nicht!

**Fix:**
```cpp
void handleRead(uint8_t available) {
    int size = 2 * lbp.as + 2;
    if (available >= size) {
        if (lbp.as) {
            const uint8_t first  = Serial1.read();
            const uint8_t second = Serial1.read();
            address              = first + (second << 8);
        }
        emptySerialBuffer();
        // Bounds check hinzufuegen:
        if ((address + (1 << lbp.ds)) <= ARRAY_SIZE(sserial_slave)) {
            memcpy((void *) txbuf, &sserial_slave[address], (1 << lbp.ds));
            send((1 << lbp.ds), 1);
        }
        if (lbp.ai) {
            address += (1 << lbp.ds);
        }
    }
}
```

---

### BUG-02: Uninitialisierte Variable `block_bytes` [KRITISCH]

**Datei:** `sserial.cpp:15`
```cpp
static uint32_t block_bytes;  // nicht initialisiert!
```

**Verwendung (Zeile 602):**
```cpp
} else if (lbp.byte == ProcessDataRPC && available >= discovery.output + 2 - block_bytes) {
```

`block_bytes` ist eine nicht-initialisierte statische Variable. In C++ werden globale/statische Variablen zwar auf 0 gesetzt, aber die Intention ist unklar. Wenn `block_bytes` tatsaechlich immer 0 sein soll, sollte es eine Konstante sein. Wenn nicht, fehlt die Zuweisung.

**Fix:**
```cpp
static uint32_t block_bytes = 0;
```
Oder besser: Den Ausdruck vereinfachen zu:
```cpp
} else if (lbp.byte == ProcessDataRPC && available >= discovery.output + 2) {
```

---

### BUG-03: strcpy() ohne Bounds-Check [KRITISCH]

**Datei:** `sserial.cpp:280-283, 301`
```cpp
strcpy((char *) heap_ptr, unit_string);  // Zeile 280
strcpy((char *) heap_ptr, name_string);  // Zeile 283
strcpy((char *) heap_ptr, name_string);  // Zeile 301
```

**Problem:** `heap_ptr` zeigt in einen 2048-Byte Buffer (`memory.heap`). Es wird nie geprueft, ob genug Platz vorhanden ist.

**Fix:**
```cpp
// Am Anfang von add_pd():
size_t remaining = (memory.bytes + SSERIAL_MEM_SIZE) - heap_ptr;
size_t needed = sizeof(process_data_descriptor_t) + strlen(unit_string) + 1
              + strlen(name_string) + 1 + NUM_BYTES(data_size_in_bits) + 4;
if (needed > remaining) {
    Serial.println("ERROR: sserial memory exhausted");
    return 0;
}
// Dann strncpy verwenden:
strncpy((char *) heap_ptr, unit_string, remaining);
```

---

### BUG-04: Serial1.read() ohne Verfuegbarkeitspruefung [HOCH]

**Datei:** `sserial.cpp:683`
```cpp
((uint8_t *) (&data_out))[i] = Serial1.read();
```

**Problem:** `Serial1.read()` gibt -1 zurueck wenn keine Daten verfuegbar sind. Dieser Wert wird auf `uint8_t` gecastet und ergibt 0xFF.

In `handleRpc()` (Zeile 602) wird zwar `available >= discovery.output + 2` geprueft, aber zwischen der Pruefung und dem tatsaechlichen Lesen koennten Bytes bereits durch `emptySerialBuffer()` in anderen Handlern konsumiert worden sein.

**Fix:** Jeden `Serial1.read()` Aufruf gegen `-1` pruefen:
```cpp
void processIncomingData() {
    for (int i = 0; i < discovery.output; i++) {
        int byte = Serial1.read();
        if (byte < 0) {
            // Fehler: Nicht genug Daten
            txbuf[0] = 0x01; // Fault status
            send(1, 1);
            return;
        }
        ((uint8_t *)(&data_out))[i] = (uint8_t)byte;
    }
    // ... rest
}
```

---

### BUG-05: Doppelte CRC-Berechnung in processIncomingData() [NIEDRIG]

**Datei:** `sserial.cpp:690-691`
```cpp
txbuf[discovery.input] = calculate_crc8((uint8_t *) txbuf, discovery.input); // CRC berechnen
send(discovery.input, 1);  // send() berechnet CRC NOCHMAL weil docrc=1
```

`send()` mit `docrc=1` ueberschreibt die CRC bei `txbuf[len]`. Die Berechnung in Zeile 690 ist redundant.

**Fix:** Entweder Zeile 690 entfernen ODER `send(discovery.input, 1)` zu `send(discovery.input + 1, 0)` aendern.

---

### BUG-06: Tote Variablen [NIEDRIG]

| Variable | Datei:Zeile | Problem |
|----------|-------------|---------|
| `timeout` | sserial.cpp:10 | Wird in `send()` auf 0 gesetzt, aber nie gelesen |
| `max_waste_ticks` | sserial.cpp:14 | Nie zugewiesen, nie gelesen |
| `rxbuf[128]` | sserial.cpp:6 | Wird in `crc_request()` verwendet, aber nie beschrieben (rxpos bleibt 0) |
| `rxpos` | sserial.cpp:9 | Nur in `handleLocalWrite()` inkrementiert, nie zurueckgesetzt |
| `last_pd` | sserial.cpp:381 | Zugewiesen per Macro, nie gelesen |
| `foo` | sserial.h:135 | Platzhalter in memory_t Union |

---

### BUG-07: Pin-Konflikt in pinmap.h [HOCH]

**Datei:** `pinmap.h:68-69 vs. 74-75`
```cpp
#define MCP0_INTA 33          // Zeile 68 - auch in ioRegister.h:8 definiert!
#define REGISTER_OUT_CLK 33   // Zeile 75 - GLEICHER PIN!
```

Pin 33 wird sowohl fuer MCP0 Interrupt A als auch fuer den Shift Register Clock verwendet. Wenn beide Funktionen gleichzeitig aktiv sind, gibt es Konflikte.

**Fix:** `MCP0_INTA` ist in `ioRegister.h` nochmal definiert (Zeile 8). Die Definition sollte nur in `pinmap.h` stehen. Der Shift Register (`REGISTER_OUT_*`) scheint nicht verwendet zu werden (nur fuer CM_S2) - Dead Code entfernen oder Pin-Zuordnung korrigieren.

---

### BUG-08: Bitfield/Union Groessen-Mismatch in ioRegister.h [HOCH]

**Datei:** `ioRegister.h:20-40`
```cpp
union {
    uint16_t inputData;      // 2 Bytes
    struct {
        uint32_t in12 : 1;  // uint32_t Bitfield -> 4 Bytes!
        uint32_t in4 : 1;
        // ... 14 weitere Bits
    } bits;
} mcp0Data;
```

**Problem:** Die Union hat `uint16_t inputData` (2 Bytes) und einen `uint32_t`-Bitfield-Struct (4 Bytes). Die Groessen stimmen nicht ueberein. Wenn `inputData` mit `MCP0.read16()` geschrieben wird, werden nur die unteren 2 Bytes gefuellt - aber der Compiler koennte den `bits`-Struct als 4 Bytes layouten.

**Fix:** Bitfields auf `uint16_t` aendern:
```cpp
union {
    uint16_t inputData;
    struct {
        uint16_t in12 : 1;
        uint16_t in4 : 1;
        // ...
    } bits;
} mcp0Data;
```

---

## 4. Architektur-Verbesserungen

### ARCH-01: Thread-Safety fuer geteilte Daten

**Problem:** Mehrere Datenstrukturen werden zwischen Core 0 und Core 1 ohne Synchronisation geteilt:

| Daten | Schreiber (Core) | Leser (Core) | Schutz |
|-------|-------------------|--------------|--------|
| `mcp0Data.inputData` | inputUpdateTask (0) | processDataInputs (1) | Keiner |
| `mcp1Data.inputData` | inputUpdateTask (0) | processDataInputs (1) | Keiner |
| `localOutputStatus` | sserialLoop->setOutput (1) | outputUpdateTask (0) | Keiner |
| `joystickX/Y/Z` | readInputsTask (0) | processDataInputs (1) | Nur `volatile` |
| `data_in` (struct) | processDataInputs (1) | sserialLoop (1) | Gleicher Core, OK |

**Loesung: Snapshot-Pattern**

Statt Mutexe (zu langsam fuer 2.5 Mbit/s) einen atomaren Snapshot verwenden:

```cpp
// In ioRegister.h:
struct IOSnapshot {
    uint16_t mcp0;
    uint16_t mcp1;
};
volatile IOSnapshot _snapshot;  // Atomic auf 32-bit ESP32

// In inputUpdateTask:
IOSnapshot s;
s.mcp0 = MCP0.read16();
s.mcp1 = MCP1.read16();
_snapshot = s;  // Atomarer 32-bit Write

// In processDataInputs:
IOSnapshot local = _snapshot;  // Atomarer 32-bit Read
// Alle Bits von 'local' lesen - konsistenter Snapshot
```

Fuer ADC-Werte (5x int16_t = 10 Bytes, nicht atomar):
```cpp
// Variante A: Spinlock (schnell, ~1us)
portMUX_TYPE adcMux = portMUX_INITIALIZER_UNLOCKED;

// In readInputsTask:
portENTER_CRITICAL(&adcMux);
adcManager->joystickX = x;
adcManager->joystickY = y;
adcManager->joystickZ = z;
adcManager->feedrate  = f;
adcManager->rotationSpeed = r;
portEXIT_CRITICAL(&adcMux);

// In processDataInputs:
int16_t x, y, z, f, r;
portENTER_CRITICAL(&adcMux);
x = adcManager.joystickX;
y = adcManager.joystickY;
z = adcManager.joystickZ;
f = adcManager.feedrate;
r = adcManager.rotationSpeed;
portEXIT_CRITICAL(&adcMux);
```

---

### ARCH-02: Fehler-Wiederherstellung (Error Recovery)

**Aktuell:** Timeout wird erkannt (5s), Flag gesetzt, aber nichts weiter passiert. Die Kommunikation ist danach tot.

**Loesung:**
```cpp
void checkForTimeout() {
    if (millis() - lastSerialCommunication > 5000) {
        if (!sserial_timeoutFlag) {
            debug.print("SSerial timeout - resetting");
            sserial_timeoutFlag = true;
        }
        // Recovery: Buffer leeren, auf neuen Cookie-Request warten
        emptySerialBuffer();
        // Outputs in sicheren Zustand bringen
        safeState();
    }
}

void safeState() {
    // Alle Ausgaenge auf LOW bei Kommunikationsverlust
    ioRegister.setOutput(OutputPin::ENA, LOW);
    ioRegister.setOutput(OutputPin::SPINDEL_ON_OFF, LOW);
    for (int i = 0; i < 8; i++) {
        ioRegister.setOutput(static_cast<OutputPin>(i), LOW);
    }
}
```

---

### ARCH-03: Watchdog Timer

**Problem:** Kein Watchdog. Wenn ein Task haengt, bleibt das System haengen - gefaehrlich bei CNC-Maschinen.

**Loesung:**
```cpp
#include "esp_task_wdt.h"

void setup() {
    // ...
    esp_task_wdt_init(3, true);  // 3 Sekunden Timeout, Panic bei Ausloesung
}

void sserialLoop(void *pvParameters) {
    esp_task_wdt_add(NULL);
    for (;;) {
        esp_task_wdt_reset();
        // ... bestehende Logik ...
    }
}
```

---

### ARCH-04: Deskriptor-Tabelle dynamisch statt hardcoded

**Problem:** `sserial_slave[]` (Zeile 19-180) ist ein 1275-Byte hardcoded Array, das identisch mit der Ausgabe von `sserial_init()` sein muss. Wenn sich die Konfiguration aendert, muss man erst `sserial_init()` ausfuehren, den Output kopieren und den Array ersetzen.

**Loesung:** Die `memory`-Struktur direkt verwenden:

```cpp
// Statt:
memcpy((void *) txbuf, &sserial_slave[address], (1 << lbp.ds));

// Direkt:
if (address + (1 << lbp.ds) <= SSERIAL_MEM_SIZE) {
    memcpy((void *) txbuf, &memory.bytes[address], (1 << lbp.ds));
}
```

Dann kann `sserial_slave[]` komplett entfernt werden (spart ~1.3 KB RAM).

---

### ARCH-05: emptySerialBuffer() Verhalten korrigieren

**Problem (Zeile 554):**
```cpp
emptySerialBuffer();  // Nach JEDEM Befehl alle verbleibenden Bytes verwerfen
```

Bei 2.5 Mbit/s koennen zwischen Command-Processing und `emptySerialBuffer()` bereits Bytes des naechsten Befehls eingetroffen sein. Die Mesa FPGA wartet zwar typischerweise auf eine Antwort, aber bei manchen Discovery-Operationen werden Befehle schnell hintereinander gesendet.

**Loesung:** `emptySerialBuffer()` nur dort aufrufen, wo es wirklich noetig ist (Fehlerfall, Timeout), nicht nach jedem normalen Befehl.

```cpp
void sserialLoop(void *pvParameters) {
    for (;;) {
        if (Serial1.available() >= 1) {
            sserial_timeoutFlag     = false;
            lastSerialCommunication = millis();
            lbp.byte                = Serial1.read();

            switch (lbp.ct) {
            case CT_LOCAL:
                lbp.wr ? handleLocalWrite() : handleLocalRead();
                break;
            case CT_RPC:
                handleRpc();
                break;
            case CT_RW:
                lbp.wr ? handleWrite() : handleRead();
                break;
            default:
                emptySerialBuffer();  // Nur bei unbekanntem Befehl
                break;
            }
            // KEIN emptySerialBuffer() hier!
        }
        checkForTimeout();
    }
}
```

Jeder Handler liest nur die Bytes, die er braucht. Das Timing muss dann aber stimmen - jeder Handler muss auf genuegend Bytes warten.

---

## 5. Modul-Refactoring

### MOD-01: sserial.cpp aufteilen

Die Datei hat 785 Zeilen und mischt Protokoll-Logik, I/O-Mapping und Debugging. Vorschlag:

```
src/
  sserial/
    sserial.cpp          -- State Machine + Task (Loop, Dispatch)
    sserial.h            -- Protokoll-Definitionen (unveraendert)
    sserial_descriptors.cpp  -- add_pd(), add_mode(), print_pd(), sserial_init()
    sserial_handlers.cpp     -- handleLocalRead/Write, handleRpc, handleRead/Write
    sserial_io.cpp           -- processDataInputs(), processIncomingData(), updateOutputPins()
  io/
    ioRegister.cpp
    ioRegister.h
    ADCManager.cpp
    ADCManager.h
  util/
    debugHelper.cpp
    debugHelper.h
    crc8.cpp
    crc8.h
  config/
    pinmap.h
    includes.h
  main.cpp
```

---

### MOD-02: IORegister vereinfachen

**Aktuelle Probleme:**
1. `MCP0_INTA` doppelt definiert (ioRegister.h:8 und pinmap.h:68)
2. `outputStatus` (static) und `localOutputStatus` (instance) - zwei Kopien ohne Sync
3. `Serial.println("MCP0 Interrupt")` im Update-Task - blockiert und hat nichts im Produktivcode verloren
4. OutputPin enum mit PCB-spezifischen Bit-Positionen ist fragil

**Verbesserung:**
```cpp
class IORegister {
public:
    struct InputState {
        uint16_t mcp0;  // Rohdaten MCP0
        uint16_t mcp1;  // Rohdaten MCP1
    };

    void init();
    InputState getInputSnapshot() const;  // Atomarer Snapshot
    void setOutputs(uint16_t outputMask);  // Alle Outputs auf einmal

private:
    volatile InputState _inputState;
    volatile uint16_t   _pendingOutput;
    uint16_t            _currentOutput;
    portMUX_TYPE        _inputMux;
    // ...
};
```

---

### MOD-03: ADCManager - Initialisierung und Filterung

**Aktuelle Probleme:**
1. Konstruktor initialisiert keine Member-Variablen
2. Keine Filterung - Rauschen wird direkt an LinuxCNC weitergegeben
3. `readJoystickX()` etc. (Zeile 36-40) werden nie aufgerufen - Dead Code

**Verbesserung:**
```cpp
ADCManager::ADCManager()
    : joystickX(2048)  // Mitte als Default
    , joystickY(2048)
    , joystickZ(2048)
    , feedrate(0)
    , rotationSpeed(0) {}

// Einfacher gleitender Durchschnitt (4 Samples):
void ADCManager::readInputsTask(void *pvParameters) {
    ADCManager *self = static_cast<ADCManager *>(pvParameters);
    int16_t jx[4] = {2048,2048,2048,2048};
    int idx = 0;
    for (;;) {
        jx[idx] = analogRead(JOYSTICK_X);
        self->joystickX = (jx[0] + jx[1] + jx[2] + jx[3]) / 4;
        // ... analog fuer Y, Z, feedrate, rotation
        idx = (idx + 1) & 3;
        vTaskDelay(pdMS_TO_TICKS(5));
    }
}
```

Dead Code entfernen: `readJoystickX()`, `readJoystickY()`, `readJoystickZ()`, `readFeedrate()`, `readRotationSpeed()`.

---

### MOD-04: Debug-System korrigieren

**Aktuelle Probleme:**
1. `debugSerialTask` verwendet Semaphore als Mutex, blockiert mit `portMAX_DELAY` (Zeile 64)
2. `_printQueue` wird nie im Task verarbeitet - nur manuell via `printQueue()`
3. 100 Eintraege * 256 Bytes * 2 Queues = **51.2 KB RAM** - sehr viel fuer ESP32
4. Semaphore-Logik ist fehlerhaft: Task blockiert, haelt das Semaphore, andere koennen nicht schreiben

**Verbesserung:** Auf eine einzige Queue reduzieren, Semaphore korrekt verwenden:

```cpp
class Debug {
public:
    void init();
    void printf(const char *format, ...);  // Thread-safe, non-blocking

private:
    static void task(void *pvParameters);
    QueueHandle_t _queue;  // Eine Queue reicht
    // Kein Semaphore noetig - FreeRTOS Queues sind thread-safe!
};

void Debug::printf(const char *format, ...) {
    char buf[128];  // 128 statt 256 reicht
    va_list args;
    va_start(args, format);
    vsnprintf(buf, sizeof(buf), format, args);
    va_end(args);
    xQueueSend(_queue, buf, 0);  // Non-blocking, drop wenn voll
}

void Debug::task(void *pvParameters) {
    Debug *self = static_cast<Debug *>(pvParameters);
    char buf[128];
    for (;;) {
        if (xQueueReceive(self->_queue, buf, portMAX_DELAY) == pdTRUE) {
            Serial.println(buf);
        }
    }
}
```

RAM-Einsparung: 51.2 KB -> 6.4 KB (50 * 128).

---

### MOD-05: Auskommentierter Code entfernen

| Datei | Zeilen | Was |
|-------|--------|-----|
| `sserial.cpp` | 724-785 | 60 Zeilen auskommentierter Print-Code (bereits in 454-516 aktiv) |
| `sserial.cpp` | 706-713 | Alte Output-Mappings |
| `ADCManager.cpp` | 26-30 | Debug-Prints |
| `ioRegister.cpp` | 61-62, 89-92 | Alte Interrupt/Test-Konfiguration |
| `main.cpp` | 30-33 | Test-Code |
| `pinmap.h` | 31-56 | Alte Pin-Definitionen |
| `ioRegister.h` | 5 | Alte OutputPin Definition |
| `includes.h` | 13-24 | Tote Debug-Macros |
| `sserial.cpp` | 383 | Auskommentiertes ADD_PROCESS_VAR |

---

## 6. Build-System und Tooling

### BUILD-01: Compiler-Warnungen aktivieren

**platformio.ini:**
```ini
[env:esp32doit-devkit-v1]
platform = espressif32
board = esp32doit-devkit-v1
framework = arduino
monitor_speed = 115200
build_flags =
    -Wall
    -Wextra
    -Wno-unused-parameter
build_type = debug
lib_deps =
    adafruit/Adafruit ADS1X15@~2.4.2
    adafruit/Adafruit MCP23017 Arduino Library@^2.3.2
    robtillaart/MCP23S17@^0.5.2
```

---

### BUILD-02: CI/CD Pipeline mit GitHub Actions

```yaml
# .github/workflows/build.yml
name: PlatformIO Build
on: [push, pull_request]
jobs:
  build:
    runs-on: ubuntu-latest
    steps:
      - uses: actions/checkout@v4
      - uses: actions/cache@v4
        with:
          path: |
            ~/.cache/pip
            ~/.platformio
          key: ${{ runner.os }}-pio
      - uses: actions/setup-python@v5
        with:
          python-version: '3.12'
      - run: pip install --upgrade platformio
      - run: pio run
```

---

### BUILD-03: Static Analysis

```ini
# platformio.ini
check_tool = clangtidy
check_flags =
    clangtidy: --checks=-*,bugprone-*,cert-*,clang-analyzer-*,performance-*
```

---

## 7. Umsetzungsstatus

### Phase 1: Kritische Sicherheit - ERLEDIGT

| # | Aufgabe | Status |
|---|---------|--------|
| 1.1 | Bounds-Check in `handleRead()` und `handleWrite()` | Erledigt |
| 1.2 | `block_bytes` entfernt, Bedingung vereinfacht | Erledigt |
| 1.3 | `strcpy` durch `heap_copy_string()` mit Bounds-Check ersetzt | Erledigt |
| 1.4 | Bitfield `uint32_t` -> `uint16_t` + `static_assert` | Erledigt |
| 1.5 | ADCManager Member initialisiert | Erledigt |
| 1.6 | Pin-Konflikte bereinigt, tote 74HC595-Definitionen entfernt | Erledigt |
| 1.7 | Serial1.read() Return-Wert geprueft, Fault-Status bei Fehler | Erledigt |
| 1.8 | `-Wall -Wextra -Wno-unused-parameter` aktiviert | Erledigt |

### Phase 2: Stabilitaet - ERLEDIGT

| # | Aufgabe | Status |
|---|---------|--------|
| 2.1 | Thread-Safety: portMUX Spinlocks + InputSnapshot fuer I/O | Erledigt |
| 2.2 | Thread-Safety: portMUX Spinlock + ADCSnapshot fuer ADC | Erledigt |
| 2.3 | Error Recovery bei Timeout (Buffer leeren, auf Discovery warten) | Erledigt |
| 2.4 | Safe-State bei Kommunikationsverlust (alle Outputs LOW) | Erledigt |
| 2.5 | Watchdog Timer (5s, esp_task_wdt) | Erledigt |
| 2.6 | emptySerialBuffer() nur bei unbekannten Befehlen | Erledigt |
| 2.7 | Deskriptor-Tabelle dynamisch (aufgeschoben - braeuchte grossen Umbau) | Offen |
| 2.8 | Redundante CRC-Berechnung entfernt | Erledigt |

### Phase 3: Codequalitaet - ERLEDIGT

| # | Aufgabe | Status |
|---|---------|--------|
| 3.1 | Dead Code entfernt (~185 Zeilen) | Erledigt |
| 3.2 | Tote Variablen entfernt (rxbuf, rxpos, timeout, etc.) | Erledigt |
| 3.3 | Magic Numbers als Konstanten (SSERIAL_TIMEOUT_MS, etc.) | Erledigt |
| 3.4 | Debug-System: 2 Queues/51KB -> 1 Queue/2.5KB | Erledigt |
| 3.5 | 8 ungenutzte Macros entfernt (MEMU8/16/32, ABS, MAX, etc.) | Erledigt |
| 3.6 | Doppelte Pin-Definitionen entfernt | Erledigt (Phase 1.6) |
| 3.7 | Debug Serial.println aus Produktiv-Code entfernt | Erledigt (Phase 2.1) |

### Phase 4: Architektur - ERLEDIGT

| # | Aufgabe | Status |
|---|---------|--------|
| 4.1 | sserial.cpp aufgeteilt in 4 Dateien | Erledigt |
| 4.2 | Header-Abhaengigkeiten aufgeraeumt | Erledigt |

**Neue Dateistruktur nach Phase 4:**
```
src/
  sserial.cpp              -- State Machine, Init, Descriptors (535 Zeilen)
  sserial.h                -- Oeffentliche Protokoll-Definitionen (160 Zeilen)
  sserial_internal.h       -- Interne geteilte State-Deklarationen (79 Zeilen)
  sserial_handlers.cpp     -- LBP Befehls-Handler (94 Zeilen)
  sserial_io.cpp           -- I/O Mapping, Safe State (87 Zeilen)
  ioRegister.cpp/h         -- MCP23S17 SPI I/O mit Spinlocks
  ADCManager.cpp/h         -- ADC mit Spinlock-Snapshot
  debugHelper.cpp/h        -- Vereinfachtes Queue-basiertes Debug
  main.cpp                 -- Setup
  pinmap.h                 -- GPIO Pin-Zuordnung
  includes.h               -- Gemeinsame Includes + Externals
  crc8.cpp/h               -- CRC-8 Berechnung
```

### Noch offen (optional/zukuenftig)

| # | Aufgabe | Hinweis |
|---|---------|---------|
| 2.7 | Deskriptor-Tabelle dynamisch statt hardcoded | Spart ~1.3KB RAM, braucht aber Aenderung der Lese-Logik |
| - | CI/CD Pipeline (GitHub Actions) | Empfohlen fuer automatisierte Builds |
| - | Static Analysis (clangtidy) | Empfohlen fuer weitere Bug-Erkennung |
| - | Konfigurierbare I/O (NVS-basiert) | Optional: I/O-Layout per NVS statt Hardcode |
| - | Unit Tests fuer Protokoll-Parser | Optional: Testbare Protokoll-Logik |

### Referenz-Links

- [Mesa 7i76 Manual PDF](https://www.mesanet.com/pdf/parallel/7i76man.pdf)
- [Mesa 7i76E Manual PDF](https://www.mesanet.com/pdf/parallel/7i76eman.pdf)
- [Mesa 7i69 Manual PDF](https://www.mesanet.com/pdf/parallel/7i69man.pdf)
- [LinuxCNC sserial.c](https://github.com/LinuxCNC/linuxcnc/blob/master/src/hal/drivers/mesa-hostmot2/sserial.c)
- [fdarling/mesa-smartserial-device-template](https://github.com/fdarling/mesa-smartserial-device-template)
- [rene-dev/stmbl sserial.c](https://github.com/rene-dev/stmbl/blob/master/src/comps/sserial.c)
- [multigcs/SmartSerial-ESP32RelayX8Board](https://github.com/multigcs/SmartSerial-ESP32RelayX8Board)
