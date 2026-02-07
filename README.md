# OpenPager

OpenPager is a high-precision POCSAG (pager) transceiver library for Arduino-compatible microcontrollers (ESP8266, ESP32, etc.) and the TI CC1101 radio transceiver. It provides a robust implementation of the POCSAG protocol for both **transmitting** and **receiving** messages.

## Features

- **TX & RX**: Full duplex support - transmit and receive POCSAG messages
- **Baud Rates**: 512, 1200, and 2400 baud
- **Message Modes**: Alphanumeric (7-bit ASCII) and Numeric (4-bit BCD)
- **Receiver Features**:
  - Edge-based bit recovery with software clock synchronization
  - Automatic polarity detection (handles inverted signals)
  - BCH error detection
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

void onMessage(PocsagMessage msg) {
    Serial.printf("[RIC: %lu] %s\n", msg.ric, msg.text);
}

void setup() {
    Serial.begin(115200);
    pager.begin(433.920, 1200);
    pager.setCallback(onMessage);
    pager.startReceive(1200);
}

void loop() {
    // Messages arrive via callback
}
```

## API Reference

### Constructor
`OpenPager(uint8_t csn_pin, uint8_t gdo0_pin)`

### TX Methods
| Method | Description |
|--------|-------------|
| `begin(freq, baud)` | Initialize radio |
| `transmit(ric, func, msg, baud, alpha)` | Send message |
| `setFreq(freq)` | Update frequency |
| `setInvert(bool)` | Invert FSK polarity |

### RX Methods
| Method | Description |
|--------|-------------|
| `startReceive(baud)` | Start receiving |
| `stopReceive()` | Stop receiving |
| `available()` | Check for messages |
| `getMessage()` | Get received message |
| `setCallback(fn)` | Set async callback |
| `getRSSI()` | Get signal strength |

### PocsagMessage Structure
```cpp
struct PocsagMessage {
    uint32_t ric;      // Receiver ID
    uint8_t func;      // Function (0-3)
    char text[256];    // Message text
    uint8_t textLen;   // Text length
    bool isNumeric;    // Numeric or alpha
    bool valid;        // BCH valid
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
- GPIO interrupt measures pulse widths for bit timing recovery
- Software PLL maintains bit synchronization
- 32-bit sync word detection with automatic polarity handling
- BCH(31,21) validation on each codeword

### Limitations
- Cannot TX and RX simultaneously (half-duplex)
- 2400 baud RX works best on ESP32 (ESP8266 has ~3-5µs interrupt jitter)
- Message buffer holds 4 messages; older messages dropped if full
