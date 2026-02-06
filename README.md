# OpenPager

OpenPager is a high-precision POCSAG (pager) transmitter library for Arduino-compatible microcontrollers (ESP8266, ESP32, etc.) and the TI CC1101 radio transceiver. It provides a robust implementation of the POCSAG protocol, supporting multiple baud rates and encoding modes with a focus on timing accuracy to ensure reliable decoding by standard pager receivers and software-defined radio (SDR) decoders.

## Features

- Supported Baud Rates: 512, 1200, and 2400 baud.
- Message Modes: Supports both Alphanumeric (7-bit ASCII) and Numeric (4-bit BCD) encoding.
- Timing Precision: Implements a sub-microsecond timing system using absolute target tracking to eliminate cumulative bit-drift, which is critical for long messages at high baud rates.
- Hardware Compatibility: Optimized for ESP8266, ESP32, and other Arduino-compatible boards.
- Automatic Calibration: Automatically reconfigures CC1101 registers when baud rates or frequencies are changed.
- Configurable Polarity: Supports FSK inversion to match different receiver requirements.

## Hardware Requirements

- Microcontroller: ESP8266 (e.g., NodeMCU, Wemos D1 Mini), ESP32, or other Arduino-compatible boards.
- Radio Module: TI CC1101 (commonly found on 433MHz or 868/915MHz modules).
- Connection: SPI interface.

### Wiring Diagram

Default wiring for ESP8266:

| CC1101 Pin | ESP8266 Pin | Description |
|------------|-------------|-------------|
| VCC        | 3.3V        | Power       |
| GND        | GND         | Ground      |
| SI (MOSI)  | D7 (GPIO13) | SPI MOSI    |
| SO (MISO)  | D6 (GPIO12) | SPI MISO    |
| SCK (CLK)  | D5 (GPIO14) | SPI Clock   |
| CSN (CS)   | D8 (GPIO15) | SPI Chip Select (Configurable) |
| GDO0       | D1 (GPIO5)  | Asynchronous Data Input (Configurable) |

## Installation

1. Download or clone this repository.
2. Move the `openpager` folder into your Arduino `libraries` folder (usually located at `Documents/Arduino/libraries`).
3. Restart the Arduino IDE.

## Quick Start (CLI)

To compile, upload, and open the serial monitor in a single command using `arduino-cli`, run the following from the root of the library directory:

```bash
arduino-cli compile -u -p /dev/ttyUSB0 --fqbn esp8266:esp8266:generic --library . examples/SimpleTransmit/SimpleTransmit.ino && arduino-cli monitor -p /dev/ttyUSB0 -c baudrate=115200
```

## API Reference

### Constructor
`OpenPager(uint8_t csn_pin, uint8_t gdo0_pin)`
- `csn_pin`: The GPIO pin used for SPI Chip Select.
- `gdo0_pin`: The GPIO pin connected to the CC1101 GDO0 pin for asynchronous data transmission.

### Methods

`void begin(float freq_mhz = 433.920, uint16_t baud = 1200)`
- Initializes the SPI interface and configures the CC1101 with the specified frequency and baud rate.

`void transmit(uint32_t ric, uint8_t func, String msg, uint16_t baud = 1200, bool alpha = true)`
- Transmits a POCSAG message.
- `ric`: Receiver Identification Code (Capcode).
- `func`: Function bits (typically 0-3). Conventions: 3 for Alphanumeric, 0 for Numeric.
- `msg`: The message string to transmit.
- `baud`: The transmission speed (512, 1200, or 2400).
- `alpha`: Set to true for Alphanumeric mode, false for Numeric mode.

`void setInvert(bool invert)`
- Inverts the FSK polarity. Some receivers require inverted signals (High Freq = 1 instead of 0).

`void setFreq(float freq_mhz)`
- Updates the carrier frequency and re-calibrates the CC1101.

## Example Usage

```cpp
#include <OpenPager.h>

// Initialize OpenPager with default pins (D8 for CSN, D1 for GDO0)
OpenPager pager(15, 5);

void setup() {
    Serial.begin(115200);
    // Start at 433.92 MHz and 1200 baud
    pager.begin(433.920, 1200);
}

void loop() {
    // Send an Alphanumeric message
    pager.transmit(1234567, 3, "Hello World", 1200, true);
    delay(5000);

    // Send a Numeric message
    pager.transmit(1234567, 0, "123-4567", 1200, false);
    delay(5000);
}
```

## Technical Background

### Timing Precision
In asynchronous serial mode, the CC1101 modulator samples the input pin at 8 times the configured internal data rate. Any jitter or drift in the bit-banging process can lead to decoding failures, especially as message length increases. OpenPager calculates the absolute target time for every bit relative to the start of the transmission using high-precision timing, ensuring zero cumulative error over the entire batch sequence.

### BCH Error Correction
The library implements the BCH(31,21) error correction algorithm required by the POCSAG standard. It calculates the 10 parity bits and the final even parity bit for every address and message codeword.

### Frame Sequencing
POCSAG messages are organized into batches. Each batch begins with a Sync word followed by 8 frames (2 slots each). The library automatically calculates the correct frame for a given RIC and manages the transmission of multiple batches if a message exceeds the available slots in the current batch.
