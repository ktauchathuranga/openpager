#ifndef OPENPAGER_H
#define OPENPAGER_H

#include <Arduino.h>
#include <SPI.h>
#include "OpenPagerDecoder.h"

#ifdef ESP32
#include "esp32-hal-rmt.h"
#include "esp_sleep.h"
#endif

#ifdef ESP8266
extern "C" {
#include "user_interface.h"
}
#endif

// Receiver buffer size (max messages in queue)
#define OPENPAGER_RX_BUFFER_SIZE 4

// Max number of CAP code (RIC) filters
#define OPENPAGER_MAX_CAP_FILTERS 16

// RMT configuration (TX only, ESP32)
#ifdef ESP32
#define OPENPAGER_RMT_TICK_FREQ        1000000  // 1 MHz = 1 µs resolution
#endif

// Hardware timer RX sampling (ESP32 and ESP8266)
// Samples GDO0 at ~19.2 kHz (8× 2400 baud) for non-blocking reception.
// On ESP8266 this uses Timer1 — incompatible with Servo and analogWrite.
#if defined(ESP32) || defined(ESP8266)
#ifdef ESP32
#define OPENPAGER_TIMER_BUF_SIZE       4096     // Timer sample ring buffer (power of 2)
#else
#define OPENPAGER_TIMER_BUF_SIZE       2048     // Smaller buffer for ESP8266 (RAM constrained)
#endif
#define OPENPAGER_TIMER_INTERVAL_US    52       // ~19.2 kHz (8x 2400 baud)
#endif

// CC1101 TX power levels (PATABLE values for 433 MHz)
// Approximate output power in dBm (varies slightly by board/antenna)
#define OPENPAGER_TX_POWER_MIN     0x12  // -30 dBm
#define OPENPAGER_TX_POWER_LOW     0x0E  // -20 dBm
#define OPENPAGER_TX_POWER_MEDIUM  0x27  // -10 dBm
#define OPENPAGER_TX_POWER_HIGH    0x50  //   0 dBm
#define OPENPAGER_TX_POWER_MAX     0xC0  // +10 dBm (default)

class OpenPager {
public:
    // Single radio (TX and RX share one CC1101)
    OpenPager(uint8_t csn_pin, uint8_t gdo0_pin);
    
    // Dual radio (dedicated RX and TX CC1101 modules — RX never stops during TX)
    OpenPager(uint8_t csn_rx, uint8_t gdo0_rx, uint8_t csn_tx, uint8_t gdo0_tx);
    
    // TX API
    void begin(float freq_mhz = 433.920, uint16_t baud = 1200);
    void transmit(uint32_t ric, uint8_t func, String msg, uint16_t baud = 1200, bool alpha = true);
    void setInvert(bool invert);
    void setFreq(float freq_mhz);
    void setTxPower(uint8_t power);  // Use OPENPAGER_TX_POWER_* constants or raw PATABLE byte

    // RX API
    void startReceive(uint16_t baud = 1200); // 0 = Auto
    void stopReceive();
    bool available();
    OpenPagerMessage getMessage();
    void setCallback(OpenPagerCallback cb);
    int16_t getRSSI();
    
    // CAP Code (RIC) Filtering
    // When filters are set, only messages matching a registered RIC are delivered.
    // With no filters, all messages pass through (default).
    void addCapFilter(uint32_t ric);          // Add a RIC to the allow list
    void removeCapFilter(uint32_t ric);       // Remove a RIC from the allow list
    void clearCapFilter();                    // Clear all filters (accept all)
    uint8_t getCapFilterCount() const { return _filter_count; }
    
    // Helpers (proxied to internal standard logic or removed if unused outside)
    // kept for compatibility if needed, otherwise removed
    // bchCheck/Repair were public, moving to private implementation in PocsagDecoder
    // If user code uses them, we break it. 
    // SimpleReceive.ino does NOT use them.
    
    // Must call this in loop() for RX to work
    void loop();
    
    // Debug counters (sum of all decoders or main)
    volatile uint32_t debugSampleCount;
    volatile uint32_t debugPreambleCount;
    volatile uint32_t debugSyncCount;
    volatile uint32_t debugBatchCount;
    
    // Debug callback (optional)
    void setDebugCallback(OpenPagerDebugCallback cb); // Deprecated/Not implemented in multi-decoder yet?

    // Query dual-radio mode
    bool isDualRadio() const { return _dual_mode; }

#ifdef ESP32
    // Deep Sleep API (ESP32 only)
    // Configure GDO2 pin for carrier-sense wakeup from deep sleep.
    // The CC1101 stays in RX mode while ESP32 sleeps.
    // When a signal is detected, GDO2 goes HIGH and wakes the ESP32.
    void setWakePin(uint8_t gdo2_pin);
    
    // Enter deep sleep. CC1101 remains in RX mode.
    // ESP32 wakes when carrier sense triggers on GDO2.
    // After waking, call begin() and startReceive() again.
    void sleep();
    
    // Auto-sleep: enter deep sleep after `timeout_ms` of no received messages.
    // Call this in loop(). Returns true if going to sleep.
    bool sleepAfterTimeout(uint32_t timeout_ms = 30000);
    
    // Check if this boot was a wake from deep sleep (carrier sense)
    static bool wokeFromSleep();
#endif

private:
    uint8_t _csn, _gdo0;
    float _freq;
    uint16_t _last_baud;
    bool _invert;
    
    // Dual-radio support
    uint8_t _csn_tx, _gdo0_tx;
    bool _dual_mode;
    void swapToTxRadio();   // Temporarily point _csn/_gdo0 to TX radio
    void restoreRxRadio();  // Restore _csn/_gdo0 to RX radio
    uint8_t _saved_csn, _saved_gdo0;  // Saved during swap
    
    // TX power
    uint8_t _tx_power;

#ifdef ESP32
    // Deep sleep
    uint8_t _gdo2;
    bool _wake_enabled;
    uint32_t _last_msg_time;
#endif
    
    // TX timing
    uint32_t _bit_start_us;
    double _bit_period_us;
    uint32_t _bits_sent;

    // RX state
    bool _rx_active;
    uint16_t _rx_config_baud; // Configured hardware baud (usually 2400 for auto)
    
    // Decoders
    OpenPagerDecoder* _decoders[3];
    uint8_t _decoder_count;

    // Message ring buffer
    OpenPagerMessage _rx_buffer[OPENPAGER_RX_BUFFER_SIZE];
    uint8_t _rx_head, _rx_tail;
    OpenPagerCallback _rx_callback;
    OpenPagerDebugCallback _debug_callback; // Not used in decoders yet
    
    // CAP code (RIC) filter list
    uint32_t _cap_filters[OPENPAGER_MAX_CAP_FILTERS];
    uint8_t _filter_count;
    bool passesCapFilter(uint32_t ric) const;

    // Internal callback wrapper
    static void staticCallback(OpenPagerMessage msg); // Trampoline

    // CC1101 Driver
    void writeReg(uint8_t reg, uint8_t value);
    uint8_t readReg(uint8_t reg);
    void sendCmd(uint8_t cmd);
    void initCC1101TX(uint16_t baud);
    void initCC1101RX(uint16_t baud);
    void calcDataRate(uint16_t baud, uint8_t *mdm4, uint8_t *mdm3);

    // POCSAG Encoding (TX) - Kept here for TX support
    uint32_t bchEncode(uint32_t data);
    uint32_t createAddressCW(uint32_t ric, uint8_t func);
    void encodeAlpha(String msg, uint32_t *codewords, uint16_t *count);
    void encodeNumeric(String msg, uint32_t *codewords, uint16_t *count);
    uint8_t reverseN(uint8_t x, uint8_t n);
    uint8_t charToBcd(char c);

    // TX
    void sendBit(bool bit);
    void sendWord(uint32_t word);
    
    // Build full POCSAG bitstream into a bool array (shared by RMT TX and legacy TX)
    void buildPocsagBitstream(uint32_t ric, uint8_t func, String msg, uint16_t baud, bool alpha,
                              bool** outBits, size_t* outCount);

#if defined(ESP32) || defined(ESP8266)
    // Hardware timer RX (samples GDO0 at fixed rate, feeds process())
    bool _timer_rx_active;
    void processTimerSamples();
#endif

#ifdef ESP32
    // RMT TX
    void transmitRmt(uint32_t ric, uint8_t func, String msg, uint16_t baud, bool alpha);
#endif
};

#endif
