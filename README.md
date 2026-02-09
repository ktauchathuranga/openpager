# OpenPager

OpenPager is a high-precision POCSAG (pager) transceiver library for Arduino-compatible microcontrollers (ESP8266, ESP32, etc.) and the TI CC1101 radio transceiver. It provides a robust implementation of the POCSAG protocol for both **transmitting** and **receiving** messages.

[![PlatformIO Registry](https://badges.registry.platformio.org/packages/ktauchathuranga/library/OpenPager.svg)](https://registry.platformio.org/libraries/ktauchathuranga/OpenPager)
[![arduino-library-badge](https://www.ardu-badge.com/badge/OpenPager.svg?)](https://www.ardu-badge.com/OpenPager)

## Features

- **RX & TX**: Full duplex support - transmit and receive POCSAG messages
- **Auto-Baud**: Automatically detects and receives 512, 1200, and 2400 baud messages simultaneously
- **Message Modes**: Alphanumeric (7-bit ASCII), Numeric (4-bit BCD), and Tone Only
- **Dual Radio**: Optional second CC1101 for true simultaneous TX/RX (RX never stops during TX)
- **ESP32 RMT**: Hardware-timed TX/RX via ESP32 RMT peripheral — no polling required
- **TX Power Control**: Adjustable transmit power from -30 dBm to +10 dBm
- **Receiver Features**:
  - Parallel decoding of multiple baud rates
  - Edge-based bit recovery with software clock synchronization
  - Automatic polarity detection (handles inverted signals)
  - BCH error detection (SECDED)
  - Async callback or polling API
  - RSSI monitoring
- **Timing Precision**: Sub-microsecond timing for reliable encoding/decoding
- **Hardware**: Optimized for ESP8266/ESP32 with CC1101 module

## Hardware Requirements

- **Microcontroller**: ESP8266, ESP32, or Arduino-compatible boards
- **Radio Module**: TI CC1101 (433MHz or 868/915MHz)
- **Connection**: SPI interface

### Wiring (ESP8266 Default)

| CC1101 Pin | ESP8266 Pin | Description |
|------------|-------------|-------------|
| VCC        | 3.3V        | Power       |
| GND        | GND         | Ground      |
| SI (MOSI)  | D7 (GPIO13) | SPI MOSI    |
| SO (MISO)  | D6 (GPIO12) | SPI MISO    |
| SCK        | D5 (GPIO14) | SPI Clock   |
| CSN        | D8 (GPIO15) | Chip Select |
| GDO0       | D1 (GPIO5)  | Data I/O    |

## Installation

1. Copy `openpager` folder to Arduino `libraries` directory
2. Restart Arduino IDE

## Quick Start

### Transmit Example
```cpp
#include <OpenPager.h>

OpenPager pager(15, 5);  // CSN, GDO0

void setup() {
    pager.begin(433.920, 1200);
    pager.setTxPower(OPENPAGER_TX_POWER_MAX);  // Optional: set TX power
}

void loop() {
    pager.transmit(1234567, 3, "Hello World", 1200, true);
    delay(5000);
}
```

### Receive Example
```cpp
#include <OpenPager.h>

OpenPager pager(15, 5);

void onMessage(OpenPagerMessage msg) {
    Serial.printf("[RIC: %lu] %s\n", msg.ric, msg.text);
}

void setup() {
    Serial.begin(115200);
    pager.begin(433.920, 0);       // Initialize radio (0 = Auto-Baud mode)
    pager.setCallback(onMessage);
    pager.startReceive(0);         // Start listening for 512, 1200, and 2400 baud
}

void loop() {
    pager.loop();  // On ESP32: timing not critical (RMT). On others: call ASAP.
}
```

### Dual Radio Example (Simultaneous TX/RX)
```cpp
#include <OpenPager.h>

// Two CC1101 modules: RX (CSN=15, GDO0=5) + TX (CSN=4, GDO0=2)
OpenPager pager(15, 5, 4, 2);

void onMessage(OpenPagerMessage msg) {
    Serial.printf("[RIC: %lu] %s\n", msg.ric, msg.text);
}

void setup() {
    Serial.begin(115200);
    pager.begin(433.920, 1200);
    pager.setCallback(onMessage);
    pager.startReceive(0);  // RX never stops, even during TX
}

void loop() {
    pager.loop();

    static uint32_t lastTx = 0;
    if (millis() - lastTx > 10000) {
        lastTx = millis();
        // TX happens on dedicated radio — RX keeps running!
        pager.transmit(1234567, 3, "Hello from dual radio", 1200, true);
    }
}
```

## API Reference

### Constructor
```cpp
OpenPager(uint8_t csn_pin, uint8_t gdo0_pin)                                  // Single radio
OpenPager(uint8_t csn_rx, uint8_t gdo0_rx, uint8_t csn_tx, uint8_t gdo0_tx)   // Dual radio
```

### TX Methods
| Method | Description |
|--------|-------------|
| `begin(freq, baud)` | Initialize radio |
| `transmit(ric, func, msg, baud, alpha)` | Send message |
| `setFreq(freq)` | Update frequency |
| `setInvert(bool)` | Invert FSK polarity |
| `setTxPower(power)` | Set TX power (see power constants below) |

### TX Power Constants
| Constant | Value | Approx. Power |
|----------|-------|---------------|
| `OPENPAGER_TX_POWER_MIN` | `0x12` | -30 dBm |
| `OPENPAGER_TX_POWER_LOW` | `0x0E` | -20 dBm |
| `OPENPAGER_TX_POWER_MEDIUM` | `0x27` | -10 dBm |
| `OPENPAGER_TX_POWER_HIGH` | `0x50` | 0 dBm |
| `OPENPAGER_TX_POWER_MAX` | `0xC0` | +10 dBm (default) |

### RX Methods
| Method | Description |
|--------|-------------|
| `startReceive(baud)` | Start receiving (0 = Auto-Baud for all rates) |
| `stopReceive()` | Stop receiving |
| `available()` | Check for messages |
| `getMessage()` | Get received message |
| `setCallback(fn)` | Set async callback |
| `getRSSI()` | Get signal strength |
| `loop()` | Process RX data (call in `loop()`) |

### Utility Methods
| Method | Description |
|--------|-------------|
| `isDualRadio()` | Returns `true` if using dual CC1101 mode |

### OpenPagerMessage Structure
```cpp
struct OpenPagerMessage {
    uint32_t ric;      // Receiver ID
    uint8_t func;      // Function (0-3)
    char text[256];    // Message text
    uint8_t textLen;   // Text length
    bool isNumeric;    // Numeric or alpha
    bool valid;        // BCH valid
    uint16_t baudRate; // Detected baud rate
};
```

## CLI Build & Upload

```bash
# Transmit example
arduino-cli compile -u -p /dev/ttyUSB0 --fqbn esp8266:esp8266:generic \
    --library . examples/SimpleTransmit/SimpleTransmit.ino

# Receive example
arduino-cli compile -u -p /dev/ttyUSB0 --fqbn esp8266:esp8266:generic \
    --library . examples/SimpleReceive/SimpleReceive.ino
```

## Technical Notes

### RX Architecture
- CC1101 operates in async mode, outputting raw demodulated data on GDO0
- **Parallel Decoders**: Spawns 3 independent `OpenPagerDecoder` instances (512, 1200, 2400)
- **Auto-Baud**: All decoders process signal in parallel; valid sync locks the appropriate decoder
- Software PLL maintains bit synchronization
- 32-bit sync word detection with automatic polarity handling
- BCH(31,21) validation on each codeword

### ESP32 RMT Hardware Acceleration
- On ESP32, the RMT peripheral captures edges on GDO0 in hardware — no CPU polling needed
- `loop()` only needs to check if a capture completed and process buffered edge data
- TX uses RMT for hardware-timed bit output — perfect timing, no busy-wait
- On non-ESP32 boards, the library falls back to `digitalRead()` polling (call `loop()` as fast as possible)

### Dual Radio Mode
- Uses two CC1101 modules on the same SPI bus (different CSN pins)
- RX radio is never interrupted during TX — true simultaneous operation
- Single-radio mode remains fully backward compatible

### Limitations
- Single radio mode: Cannot TX and RX simultaneously (half-duplex)
- 2400 baud RX works best on ESP32 (ESP8266 has ~3-5µs interrupt jitter)
- Message buffer holds 4 messages; older messages dropped if full
