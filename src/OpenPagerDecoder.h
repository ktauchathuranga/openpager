#ifndef OPENPAGER_DECODER_H
#define OPENPAGER_DECODER_H

#include <Arduino.h>

// POCSAG Constants
#define POCSAG_SYNC_CODE      0x7CD215D8
#define POCSAG_SYNC_INVERTED  0x832DEA27
#define POCSAG_IDLE_CODE      0x7A89C197

enum OpenPagerMode {
    OPENPAGER_NUMERIC = 0,
    OPENPAGER_ALPHA = 1
};

// Received message structure
struct OpenPagerMessage {
    uint32_t ric;        // Receiver ID Code (capcode)
    uint8_t func;        // Function (0-3)
    char text[256];      // Message text
    uint8_t textLen;     // Text length
    bool isNumeric;      // Numeric or alphanumeric
    bool valid;          // Valid message flag
    uint16_t baudRate;   // Baud rate
};

// Callback type for async message reception
typedef void (*OpenPagerCallback)(OpenPagerMessage msg);

// Debug callback type
typedef void (*OpenPagerDebugCallback)(uint32_t* codewords, uint8_t count);

class OpenPagerDecoder {
public:
    OpenPagerDecoder(uint16_t baud, OpenPagerCallback callback);
    
    void process(uint32_t now, bool bit);
    void processEdgeData(const uint32_t* durations_us, const bool* levels, size_t count);
    void reset();
    
    // Config
    void setInvert(bool invert);
    
    // Debug
    volatile uint32_t debugSampleCount;
    volatile uint32_t debugSyncCount;
    volatile uint32_t debugBatchCount;

private:
    uint16_t _baud;
    OpenPagerCallback _callback;
    bool _invert;
    
    // Timing
    double _bit_period_us;
    uint32_t _next_sample_time;
    bool _last_bit;
    
    // Bit State
    uint32_t _shift_reg;
    uint8_t _bit_count;
    
    // Sync State
    bool _synced;
    bool _inverted; // Auto-detected polarity
    
    // Batch State
    uint32_t _codewords[17];
    uint8_t _cw_index;
    
    // Message Assembly
    uint32_t _current_ric;
    uint8_t _current_func;
    uint32_t _msg_cws[64];
    uint8_t _msg_cw_count;
    bool _in_message;
    uint32_t _last_msg_activity_us;
    
    // Helpers
    void processSample(bool bit);
    bool checkSyncWord();
    void processBatch();
    void finalizeMessage();
    
    bool bchCheck(uint32_t cw);
    uint32_t bchRepair(uint32_t cw);
    uint32_t bchEncode(uint32_t data);
    
    void decodeAlphaMessage(OpenPagerMessage& msg);
    void decodeNumericMessage(OpenPagerMessage& msg);
    char bcdToChar(uint8_t bcd);
};

#endif
