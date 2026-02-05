#include "OpenPager.h"

OpenPager::OpenPager(uint8_t csn_pin, uint8_t gdo0_pin) : 
    _csn(csn_pin), _gdo0(gdo0_pin), _freq(433.920), _last_baud(0), _invert(false) {
}

void OpenPager::begin(float freq_mhz, uint16_t baud) {
    _freq = freq_mhz;
    pinMode(_csn, OUTPUT);
    pinMode(_gdo0, OUTPUT);
    digitalWrite(_csn, HIGH);
    digitalWrite(_gdo0, LOW);
    
    SPI.begin();
    initCC1101(baud);
    _last_baud = baud;
}

void OpenPager::setInvert(bool invert) {
    _invert = invert;
}

void OpenPager::setFreq(float freq_mhz) {
    _freq = freq_mhz;
    initCC1101(_last_baud);
}

void OpenPager::transmit(uint32_t ric, uint8_t func, String msg, uint16_t baud, bool alpha) {
    if (baud != _last_baud) {
        initCC1101(baud);
        _last_baud = baud;
    }

    uint32_t addr_cw = createAddressCW(ric, func);
    uint32_t msg_cw[128];
    uint16_t msg_count = 0;
    
    if (alpha) encodeAlpha(msg, msg_cw, &msg_count);
    else encodeNumeric(msg, msg_cw, &msg_count);

    uint8_t frame = ric & 0x07;
    uint16_t start_slot = frame * 2;
    uint16_t total_cw = 1 + msg_count;
    uint16_t batches = (start_slot + total_cw + 15) / 16;

    // Start TX
    sendCmd(0x35); // STX
    
    _bit_period_us = 1000000.0 / (double)baud;
    _bits_sent = 0;
    _bit_start_us = micros();

    // Preamble
    for (int i = 0; i < 20; i++) sendWord(0xAAAAAAAA);

    uint16_t msg_idx = 0;
    bool addr_sent = false;
    for (uint16_t b = 0; b < batches; b++) {
        sendWord(POCSAG_SYNC_CODE);
        for (uint8_t f = 0; f < 8; f++) {
            for (uint8_t s = 0; s < 2; s++) {
                if (!addr_sent && f == frame) {
                    sendWord(addr_cw);
                    addr_sent = true;
                } else if (addr_sent && msg_idx < msg_count) {
                    sendWord(msg_cw[msg_idx++]);
                } else {
                    sendWord(POCSAG_IDLE_CODE);
                }
            }
        }
        if (msg_idx >= msg_count) break;
    }

    for (int i = 0; i < 4; i++) sendWord(POCSAG_IDLE_CODE);
    
    digitalWrite(_gdo0, LOW);
    sendCmd(0x36); // SIDLE
}

// CC1101 Internals
void OpenPager::writeReg(uint8_t reg, uint8_t value) {
    digitalWrite(_csn, LOW);
    SPI.transfer(reg);
    SPI.transfer(value);
    digitalWrite(_csn, HIGH);
}

void OpenPager::sendCmd(uint8_t cmd) {
    digitalWrite(_csn, LOW);
    SPI.transfer(cmd);
    digitalWrite(_csn, HIGH);
}

void OpenPager::initCC1101(uint16_t baud) {
    sendCmd(0x30); // SRES
    delay(10);
    
    uint32_t freq_reg = (uint32_t)((_freq * 65536.0) / 26.0);
    uint8_t mdm4, mdm3;
    calcDataRate(baud, &mdm4, &mdm3);

    writeReg(0x02, 0x0D); // IOCFG0: Async Serial
    writeReg(0x08, 0x32); // PKTCTRL0: Async, Infinite
    writeReg(0x0D, (freq_reg >> 16) & 0xFF);
    writeReg(0x0E, (freq_reg >> 8) & 0xFF);
    writeReg(0x0F, freq_reg & 0xFF);
    writeReg(0x10, mdm4);
    writeReg(0x11, mdm3);
    writeReg(0x12, 0x00); // 2-FSK
    writeReg(0x13, 0x22); // Deviation
    writeReg(0x15, 0x40); // Standard deviation settings
    writeReg(0x18, 0x18); // FSCTRL0
    writeReg(0x19, 0x16); // FOCCFG
    writeReg(0x1B, 0x03); // AGCCTRL2
    writeReg(0x21, 0x56); // FREND1
    writeReg(0x22, 0x10); // FREND0
    writeReg(0x23, 0xE9); // FSCAL3
    writeReg(0x24, 0x2A); // FSCAL2
    writeReg(0x25, 0x00); // FSCAL1
    writeReg(0x26, 0x1F); // FSCAL0
    writeReg(0x3E, 0xC0); // PATABLE: Max Power
    
    sendCmd(0x33); // SCAL
    delay(5);
}

void OpenPager::calcDataRate(uint16_t baud, uint8_t *mdm4, uint8_t *mdm3) {
    if (baud == 512) { *mdm4 = 0xF4; *mdm3 = 0x4B; }
    else if (baud == 2400) { *mdm4 = 0xF6; *mdm3 = 0x83; }
    else { *mdm4 = 0xF5; *mdm3 = 0x83; } // Default 1200
}

// POCSAG Internals
uint32_t OpenPager::bchEncode(uint32_t data) {
    uint32_t cwE = data & 0xFFFFF800;
    for (int i = 0; i < 21; i++) {
        if (cwE & 0x80000000) cwE ^= 0xED200000;
        cwE <<= 1;
    }
    uint32_t cw = (data & 0xFFFFF800) | (cwE >> 21);
    uint32_t p = 0;
    for (int i = 0; i < 32; i++) if ((cw >> i) & 1) p++;
    if (p % 2) cw |= 1;
    return cw;
}

uint32_t OpenPager::createAddressCW(uint32_t ric, uint8_t func) {
    uint32_t raw = ((ric >> 3) << 13) | ((func & 0x03) << 11);
    return bchEncode(raw);
}

uint8_t OpenPager::reverseN(uint8_t x, uint8_t n) {
    uint8_t r = 0;
    for (int i = 0; i < 8; i++) { r = (r << 1) | ((x >> i) & 1); }
    return r >> (8 - n);
}

void OpenPager::encodeAlpha(String msg, uint32_t *codewords, uint16_t *count) {
    uint64_t buf = 0; uint8_t bits = 0; uint16_t idx = 0;
    for (uint16_t i = 0; i < msg.length(); i++) {
        buf = (buf << 7) | reverseN(msg[i], 7);
        bits += 7;
        while (bits >= 20) {
            bits -= 20;
            codewords[idx++] = bchEncode((1UL << 31) | (((buf >> bits) & 0xFFFFF) << 11));
        }
    }
    buf = (buf << 7) | reverseN(0x04, 7); bits += 7;
    if (bits > 0) {
        while (bits < 20) { buf <<= 1; bits++; }
        codewords[idx++] = bchEncode((1UL << 31) | (((buf >> (bits-20)) & 0xFFFFF) << 11));
    }
    *count = idx;
}

void OpenPager::encodeNumeric(String msg, uint32_t *codewords, uint16_t *count) {
    uint32_t cur = 0; uint8_t digits = 0; uint16_t idx = 0;
    for (uint16_t i = 0; i < msg.length(); i++) {
        cur = (cur << 4) | reverseN(charToBcd(msg[i]), 4);
        if (++digits == 5) {
            codewords[idx++] = bchEncode((1UL << 31) | (cur << 11));
            cur = 0; digits = 0;
        }
    }
    if (digits > 0) {
        while (digits < 5) { cur = (cur << 4) | reverseN(0x0C, 4); digits++; }
        codewords[idx++] = bchEncode((1UL << 31) | (cur << 11));
    }
    *count = idx;
}

uint8_t OpenPager::charToBcd(char c) {
    if (c >= '0' && c <= '9') return c - '0';
    if (c == '*' || c == 'U' || c == 'u') return 0x0B;
    if (c == ' ') return 0x0C;
    if (c == '-') return 0x0D;
    if (c == ')' || c == ']') return 0x0E;
    if (c == '(' || c == '[') return 0x0F;
    return 0x0C;
}

void OpenPager::sendBit(bool bit) {
    if (_invert) bit = !bit;
    digitalWrite(_gdo0, bit ? LOW : HIGH);
    _bits_sent++;
    uint32_t target = _bit_start_us + (uint32_t)(_bits_sent * _bit_period_us);
    while ((int32_t)(target - micros()) > 0);
}

void OpenPager::sendWord(uint32_t word) {
    noInterrupts();
    for (int i = 31; i >= 0; i--) sendBit((word >> i) & 1);
    interrupts();
    ESP.wdtFeed();
}
