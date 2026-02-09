#ifndef OPENPAGER_H
#define OPENPAGER_H

#include <Arduino.h>
#include <SPI.h>
#include "OpenPagerDecoder.h"

#ifdef ESP32
#include "esp32-hal-rmt.h"
#endif

// Receiver buffer size (max messages in queue)
#define OPENPAGER_RX_BUFFER_SIZE 4

// RMT configuration
#ifdef ESP32
#define OPENPAGER_RMT_RX_BUF_SYMBOLS 512
#define OPENPAGER_RMT_TICK_FREQ      1000000  // 1 MHz = 1 µs resolution
#endif

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

    // RX API
    void startReceive(uint16_t baud = 1200); // 0 = Auto
    void stopReceive();
    bool available();
    OpenPagerMessage getMessage();
    void setCallback(OpenPagerCallback cb);
    int16_t getRSSI();
    
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

#ifdef ESP32
    // RMT RX
    rmt_data_t* _rmt_rx_buf;
    size_t _rmt_rx_sym_count;
    bool _rmt_rx_reading;
    void processRmtEdges();
    
    // RMT TX
    void transmitRmt(uint32_t ric, uint8_t func, String msg, uint16_t baud, bool alpha);
#endif
};

#endif
