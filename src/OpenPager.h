#ifndef OPENPAGER_H
#define OPENPAGER_H

#include <Arduino.h>
#include <SPI.h>
#include "OpenPagerDecoder.h"

// Receiver buffer size (max messages in queue)
#define POCSAG_RX_BUFFER_SIZE 4

class OpenPager {
public:
    OpenPager(uint8_t csn_pin, uint8_t gdo0_pin);
    
    // TX API
    void begin(float freq_mhz = 433.920, uint16_t baud = 1200);
    void transmit(uint32_t ric, uint8_t func, String msg, uint16_t baud = 1200, bool alpha = true);
    void setInvert(bool invert);
    void setFreq(float freq_mhz);

    // RX API
    void startReceive(uint16_t baud = 1200); // 0 = Auto
    void stopReceive();
    bool available();
    PocsagMessage getMessage();
    void setCallback(PocsagCallback cb);
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
    void setDebugCallback(PocsagDebugCallback cb); // Deprecated/Not implemented in multi-decoder yet?

private:
    uint8_t _csn, _gdo0;
    float _freq;
    uint16_t _last_baud;
    bool _invert;
    
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
    PocsagMessage _rx_buffer[POCSAG_RX_BUFFER_SIZE];
    uint8_t _rx_head, _rx_tail;
    PocsagCallback _rx_callback;
    PocsagDebugCallback _debug_callback; // Not used in decoders yet

    // Internal callback wrapper
    static void staticCallback(PocsagMessage msg); // Trampoline

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
};

#endif
