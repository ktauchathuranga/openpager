#ifndef OPENPAGER_H
#define OPENPAGER_H

#include <Arduino.h>
#include <SPI.h>

#define POCSAG_SYNC_CODE  0x7CD215D8
#define POCSAG_IDLE_CODE  0x7A89C197

enum PocsagMode {
    POCSAG_NUMERIC = 0,
    POCSAG_ALPHA = 1
};

class OpenPager {
public:
    OpenPager(uint8_t csn_pin, uint8_t gdo0_pin);
    
    void begin(float freq_mhz = 433.920, uint16_t baud = 1200);
    void transmit(uint32_t ric, uint8_t func, String msg, uint16_t baud = 1200, bool alpha = true);
    void setInvert(bool invert);
    void setFreq(float freq_mhz);

private:
    uint8_t _csn, _gdo0;
    float _freq;
    uint16_t _last_baud;
    bool _invert;
    
    // Internal timing
    uint32_t _bit_start_us;
    double _bit_period_us;
    uint32_t _bits_sent;

    // CC1101 Driver
    void writeReg(uint8_t reg, uint8_t value);
    void sendCmd(uint8_t cmd);
    void initCC1101(uint16_t baud);
    void calcDataRate(uint16_t baud, uint8_t *mdm4, uint8_t *mdm3);

    // POCSAG Encoding
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
