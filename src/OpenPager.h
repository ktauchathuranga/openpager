#ifndef OPENPAGER_H
#define OPENPAGER_H

#include <Arduino.h>
#include <SPI.h>

// POCSAG Constants
#define POCSAG_SYNC_CODE      0x7CD215D8
#define POCSAG_SYNC_INVERTED  0x832DEA27
#define POCSAG_IDLE_CODE      0x7A89C197

// Receiver buffer size (max messages in queue)
#define POCSAG_RX_BUFFER_SIZE 4

enum PocsagMode {
    POCSAG_NUMERIC = 0,
    POCSAG_ALPHA = 1
};

// Received message structure
struct PocsagMessage {
    uint32_t ric;        // Receiver ID Code (capcode)
    uint8_t func;        // Function (0-3)
    char text[256];      // Message text
    uint8_t textLen;     // Text length
    bool isNumeric;      // Numeric or alphanumeric
    bool valid;          // Valid message flag
};

// Callback type for async message reception
typedef void (*PocsagCallback)(PocsagMessage msg);

// Debug callback type - called after each batch with raw codewords
typedef void (*PocsagDebugCallback)(uint32_t* codewords, uint8_t count);

class OpenPager {
public:
    OpenPager(uint8_t csn_pin, uint8_t gdo0_pin);
    
    // TX API
    void begin(float freq_mhz = 433.920, uint16_t baud = 1200);
    void transmit(uint32_t ric, uint8_t func, String msg, uint16_t baud = 1200, bool alpha = true);
    void setInvert(bool invert);
    void setFreq(float freq_mhz);

    // RX API
    void startReceive(uint16_t baud = 1200);
    void stopReceive();
    bool available();
    PocsagMessage getMessage();
    void setCallback(PocsagCallback cb);
    int16_t getRSSI();
    bool bchCheck(uint32_t codeword);
    uint32_t bchRepair(uint32_t codeword);
    
    // Must call this in loop() for RX to work
    void loop();
    
    // Debug counters (public for monitoring)
    volatile uint32_t debugSampleCount;
    volatile uint32_t debugPreambleCount;
    volatile uint32_t debugSyncCount;
    volatile uint32_t debugBatchCount;
    
    // Debug callback (optional) - set this to receive raw codewords
    void setDebugCallback(PocsagDebugCallback cb);
    PocsagDebugCallback _debug_callback;

private:
    uint8_t _csn, _gdo0;
    float _freq;
    uint16_t _last_baud;
    bool _invert;
    
    // TX timing
    uint32_t _bit_start_us;
    double _bit_period_us;
    uint32_t _bits_sent;

    // RX state machine
    volatile bool _rx_active;
    uint16_t _rx_baud;
    uint32_t _rx_sample_interval_us;
    uint32_t _rx_last_sample_us;
    
    // Bit-level state
    uint32_t _rx_shift_reg;
    uint8_t _rx_bit_count;
    
    // Sync hunting
    bool _rx_synced;
    bool _rx_inverted;
    uint8_t _rx_preamble_ones;
    
    // Batch collection
    uint32_t _rx_codewords[17];  // 1 sync + 16 data
    uint8_t _rx_cw_index;
    
    // Current message being assembled
    uint32_t _rx_current_ric;
    uint8_t _rx_current_func;
    uint32_t _rx_msg_cws[40];  // Support longer messages (up to 40 codewords)
    uint8_t _rx_msg_cw_count;
    bool _rx_in_message;
    uint32_t _rx_last_msg_activity_us;  // For timeout detection

    // Message ring buffer
    PocsagMessage _rx_buffer[POCSAG_RX_BUFFER_SIZE];
    uint8_t _rx_head, _rx_tail;
    PocsagCallback _rx_callback;

    // CC1101 Driver
    void writeReg(uint8_t reg, uint8_t value);
    uint8_t readReg(uint8_t reg);
    void sendCmd(uint8_t cmd);
    void initCC1101TX(uint16_t baud);
    void initCC1101RX(uint16_t baud);
    void calcDataRate(uint16_t baud, uint8_t *mdm4, uint8_t *mdm3);

    // POCSAG Encoding (TX)
    uint32_t bchEncode(uint32_t data);
    uint32_t createAddressCW(uint32_t ric, uint8_t func);
    void encodeAlpha(String msg, uint32_t *codewords, uint16_t *count);
    void encodeNumeric(String msg, uint32_t *codewords, uint16_t *count);
    uint8_t reverseN(uint8_t x, uint8_t n);
    uint8_t charToBcd(char c);

    // POCSAG Decoding (RX)
    void processSample(bool bit);
    bool checkSyncWord();
    void processBatch();
    void finalizeMessage();
    void decodeAlphaMessage();
    void decodeNumericMessage();
    void pushMessage(PocsagMessage& msg);
    char bcdToChar(uint8_t bcd);

    // TX
    void sendBit(bool bit);
    void sendWord(uint32_t word);
};

#endif
