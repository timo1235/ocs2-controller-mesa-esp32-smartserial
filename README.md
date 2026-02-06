# OCS2 Controller - Mesa ESP32 SmartSerial

ESP32-Firmware, die als SmartSerial-Slave (LBP-Protokoll) mit einer Mesa FPGA-Karte (z.B. 7i92, 7i76E) kommuniziert. Damit wird das [OCS2 CNC Controller Board](https://github.com/timo1235/ocs2-controller) an LinuxCNC angebunden.

## Funktionsweise

Der ESP32 meldet sich gegenüber der Mesa-Karte als SmartSerial-Device an und tauscht zyklisch Prozessdaten aus:

**Inputs (ESP32 → Mesa → LinuxCNC):**
- 16 digitale Eingänge (in1–in16) über MCP23S17 I/O-Expander (SPI)
- Joystick X/Y/Z, Feedrate, Rotation Speed über ADS1115 ADCs (I2C)
- Steuersignale: Alarm, OK, Motorstart, Programmstart, Auswahl X/Y/Z, Speed 1/2

**Outputs (LinuxCNC → Mesa → ESP32):**
- 8 digitale Ausgänge (out1–out8) über MCP23S17 I/O-Expander
- Enable (ENA) und Spindel On/Off

## Hardware

- **MCU:** ESP32 (DevKit V1 oder OCS2-Board mit ESP32-S2)
- **I/O-Expander:** 2× MCP23S17 (SPI) für digitale Ein-/Ausgänge
- **ADC:** 2× ADS1115 (I2C) für Analog-Eingänge (Joystick, Feedrate, Rotation)
- **UART:** SmartSerial-Kommunikation mit Mesa-Karte @ 2.5 MBaud

### Pin-Belegung (DevKit V1 / PCB 1.2)

| Funktion | Pin |
|---|---|
| SmartSerial RX/TX | GPIO 16 / 17 |
| SPI (MISO/MOSI/SCK) | GPIO 12 / 13 / 14 |
| MCP23S17 CS | GPIO 15 |
| I2C ADC (SDA/SCL) | GPIO 21 / 22 |
| Analog (Feedrate, Rotation, Joy X/Y/Z) | GPIO 32, 35, 34, 39, 36 |

## Architektur

Das Projekt nutzt Dual-Core-FreeRTOS:

- **Core 0:** I/O-Tasks – liest MCP23S17-Inputs (Interrupt-gesteuert), schreibt Outputs, liest ADC-Werte
- **Core 1:** SmartSerial-Task – LBP-Protokoll-Verarbeitung, UART-Kommunikation mit Mesa-Karte

Thread-Safety wird über `portMUX`-Spinlocks und atomare Snapshots sichergestellt.

### Dateien

| Datei | Beschreibung |
|---|---|
| `sserial.cpp` | SmartSerial-Initialisierung, Hauptloop, Descriptor-Tabellen |
| `sserial_handlers.cpp` | LBP-Protokoll-Handler (Local Read/Write, RPC, Memory Read/Write) |
| `sserial_io.cpp` | Prozessdaten-Mapping (Pins ↔ SmartSerial-Daten), Safe-State |
| `sserial_internal.h` | Interne Typen und shared state |
| `ioRegister.cpp/h` | MCP23S17 I/O-Expander-Treiber mit Interrupt-Support |
| `ADCManager.cpp/h` | ADS1115 ADC-Auslese-Task |
| `pinmap.h` | Pin-Definitionen für verschiedene Board-Varianten |

## Build

Das Projekt verwendet [PlatformIO](https://platformio.org/).

```bash
# Build
pio run

# Upload
pio run -t upload

# Serial Monitor
pio device monitor
```

## LinuxCNC-Konfiguration

In der LinuxCNC HAL-Konfiguration wird das SmartSerial-Device als `ocs2` erkannt. Die Pins sind dann unter `hm2_7i92.0.ocs2.0.0.*` (o.ä.) verfügbar:

```hal
# Beispiel: Digitalen Ausgang setzen
net spindle-on spindle.0.on => hm2_7i92.0.ocs2.0.0.spindel

# Beispiel: Joystick-Wert lesen
net joy-x <= hm2_7i92.0.ocs2.0.0.joy_x
```

## Lizenz

Siehe [LICENSE](LICENSE) Datei.
