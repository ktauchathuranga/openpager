#include "OpenPager.h"

OpenPager::OpenPager(uint8_t csn_pin, uint8_t gdo0_pin) : 
    _csn(csn_pin), _gdo0(gdo0_pin), _freq(433.920), _last_baud(0), _invert(false),
    _rx_active(false), _rx_baud(1200),
    _rx_shift_reg(0), _rx_bit_count(0),
    _rx_synced(false), _rx_inverted(false), _rx_preamble_ones(0),
    _rx_cw_index(0),
    _rx_current_ric(0), _rx_current_func(0), _rx_msg_cw_count(0), _rx_in_message(false),
    _rx_last_msg_activity_us(0),
    _rx_head(0), _rx_tail(0), _rx_callback(nullptr), _debug_callback(nullptr),
    debugSampleCount(0), debugPreambleCount(0), debugSyncCount(0), debugBatchCount(0) {
}

void OpenPager::begin(float freq_mhz, uint16_t baud) {
    _freq = freq_mhz;
    pinMode(_csn, OUTPUT);
    digitalWrite(_csn, HIGH);
    
    SPI.begin();
    initCC1101TX(baud);
    _last_baud = baud;
}

void OpenPager::setInvert(bool invert) {
    _invert = invert;
}

void OpenPager::setFreq(float freq_mhz) {
    _freq = freq_mhz;
    if (_rx_active) {
        initCC1101RX(_rx_baud);
    } else {
        initCC1101TX(_last_baud);
    }
}

// ============== RX API ==============

void OpenPager::startReceive(uint16_t baud) {
    stopReceive();
    
    _rx_baud = baud;
    _bit_period_us = 1000000.0 / (double)baud;
    _rx_sample_interval_us = (uint32_t)(_bit_period_us);
    
    // Reset state
    _rx_shift_reg = 0;
    _rx_bit_count = 0;
    _rx_synced = false;
    _rx_inverted = false;
    _rx_preamble_ones = 0;
    _rx_cw_index = 0;
    _rx_in_message = false;
    _rx_msg_cw_count = 0;
    _rx_head = 0;
    _rx_tail = 0;
    
    // Reset debug counters
    debugSampleCount = 0;
    debugPreambleCount = 0;
    debugSyncCount = 0;
    debugBatchCount = 0;
    
    // Configure CC1101 for receive
    initCC1101RX(baud);
    
    // GDO0 as input
    pinMode(_gdo0, INPUT);
    
    _rx_last_sample_us = micros();
    _rx_active = true;
}

void OpenPager::stopReceive() {
    if (_rx_active) {
        _rx_active = false;
        sendCmd(0x36);  // SIDLE
    }
}

bool OpenPager::available() {
    return (_rx_head != _rx_tail);
}

PocsagMessage OpenPager::getMessage() {
    PocsagMessage msg;
    msg.valid = false;
    
    if (_rx_head != _rx_tail) {
        msg = _rx_buffer[_rx_tail];
        _rx_tail = (_rx_tail + 1) % POCSAG_RX_BUFFER_SIZE;
    }
    return msg;
}

void OpenPager::setCallback(PocsagCallback cb) {
    _rx_callback = cb;
}

void OpenPager::setDebugCallback(PocsagDebugCallback cb) {
    _debug_callback = cb;
}

int16_t OpenPager::getRSSI() {
    uint8_t rssi_raw = readReg(0x34 | 0xC0);
    int16_t rssi_dBm;
    if (rssi_raw >= 128) {
        rssi_dBm = ((int16_t)rssi_raw - 256) / 2 - 74;
    } else {
        rssi_dBm = rssi_raw / 2 - 74;
    }
    return rssi_dBm;
}

// ============== Main Loop Processing ==============

void OpenPager::loop() {
    if (!_rx_active) return;
    
    uint32_t now = micros();
    bool currentBit = digitalRead(_gdo0);
    static bool lastBit = false;
    static uint32_t lastEdge = 0;
    static uint32_t nextSampleTime = 0;
    
    // Detect edges and resync timing
    if (currentBit != lastBit) {
        lastBit = currentBit;
        lastEdge = now;
        
        // Set next sample point to half a bit period from now (middle of bit)
        nextSampleTime = now + (uint32_t)(_bit_period_us * 0.5);
    }
    
    // Sample at the calculated mid-bit point
    if ((int32_t)(now - nextSampleTime) >= 0 && nextSampleTime != 0) {
        debugSampleCount++;
        processSample(currentBit);
        
        // Schedule next sample one bit period later
        nextSampleTime += (uint32_t)_bit_period_us;
        
        // Prevent runaway sampling if we missed many samples
        if ((int32_t)(now - nextSampleTime) > (int32_t)(_bit_period_us * 3)) {
            nextSampleTime = 0;  // Wait for next edge
        }
    }
    
    // Timeout: if we're in a message and haven't received activity in a while,
    // finalize whatever we have
    if (_rx_in_message && _rx_msg_cw_count > 0) {
        uint32_t timeout_us = (uint32_t)(_bit_period_us * 32 * 20);  // ~20 codewords
        if ((int32_t)(now - _rx_last_msg_activity_us) > (int32_t)timeout_us) {
            finalizeMessage();
        }
    }
}

void OpenPager::processSample(bool bit) {
    // Shift bit into register
    _rx_shift_reg = (_rx_shift_reg << 1) | (bit ? 1 : 0);
    _rx_bit_count++;
    
    if (!_rx_synced) {
        // Preamble detection - look for alternating pattern
        static bool lastBit = false;
        if (bit != lastBit) {
            _rx_preamble_ones++;
            if (_rx_preamble_ones > 32) {
                debugPreambleCount++;
            }
        } else {
            _rx_preamble_ones = 0;
        }
        lastBit = bit;
        
        // Check for sync word (both polarities)
        if (checkSyncWord()) {
            _rx_synced = true;
            _rx_bit_count = 0;
            _rx_cw_index = 0;
            debugSyncCount++;
        }
    } else {
        // Synced - collect 32-bit codewords
        if (_rx_bit_count == 32) {
            uint32_t cw = _rx_shift_reg;
            if (_rx_inverted) cw = ~cw;
            
            _rx_codewords[_rx_cw_index++] = cw;
            _rx_bit_count = 0;
            
            // Check if we got a full batch (16 codewords)
            if (_rx_cw_index >= 16) {
                debugBatchCount++;
                processBatch();
                
                // After batch, look for next sync
                // NOTE: Don't reset _rx_in_message here - message may continue!
                _rx_synced = false;
                _rx_cw_index = 0;
            }
        }
    }
}

bool OpenPager::checkSyncWord() {
    // Helper to count bit differences
    auto bitDiff = [](uint32_t a, uint32_t b) {
        uint32_t x = a ^ b;
        uint32_t count = 0;
        while (x) {
            count++;
            x &= (x - 1);
        }
        return count;
    };

    // Check normal polarity (allow 2 bit errors)
    if (bitDiff(_rx_shift_reg, POCSAG_SYNC_CODE) <= 2) {
        _rx_inverted = false;
        return true;
    }
    // Check inverted (allow 2 bit errors)
    if (bitDiff(_rx_shift_reg, POCSAG_SYNC_INVERTED) <= 2) {
        _rx_inverted = true;
        return true;
    }
    // Check bit-inverted
    if (bitDiff(~_rx_shift_reg, POCSAG_SYNC_CODE) <= 2) {
        _rx_inverted = true;
        return true;
    }
    return false;
}

// ============== CC1101 Driver ==============

void OpenPager::writeReg(uint8_t reg, uint8_t value) {
    digitalWrite(_csn, LOW);
    SPI.transfer(reg);
    SPI.transfer(value);
    digitalWrite(_csn, HIGH);
}

uint8_t OpenPager::readReg(uint8_t reg) {
    digitalWrite(_csn, LOW);
    SPI.transfer(reg | 0x80);
    uint8_t val = SPI.transfer(0);
    digitalWrite(_csn, HIGH);
    return val;
}

void OpenPager::sendCmd(uint8_t cmd) {
    digitalWrite(_csn, LOW);
    SPI.transfer(cmd);
    digitalWrite(_csn, HIGH);
}

void OpenPager::initCC1101TX(uint16_t baud) {
    sendCmd(0x30);  // SRES
    delay(10);
    
    uint32_t freq_reg = (uint32_t)((_freq * 65536.0) / 26.0);
    uint8_t mdm4, mdm3;
    calcDataRate(baud, &mdm4, &mdm3);

    // GDO0 config: 0x0D = Async serial data
    writeReg(0x00, 0x06);  // IOCFG2: GDO2 unused
    writeReg(0x02, 0x0D);  // IOCFG0: Async serial data
    
    writeReg(0x08, 0x32);  // PKTCTRL0: Async, infinite
    
    // Frequency
    writeReg(0x0D, (freq_reg >> 16) & 0xFF);  // FREQ2
    writeReg(0x0E, (freq_reg >> 8) & 0xFF);   // FREQ1
    writeReg(0x0F, freq_reg & 0xFF);          // FREQ0
    
    // Data rate
    writeReg(0x10, mdm4);  // MDMCFG4
    writeReg(0x11, mdm3);  // MDMCFG3
    
    // Modulation: 2-FSK
    writeReg(0x12, 0x00);  // MDMCFG2: 2-FSK, no sync
    
    // Deviation: ~4.5kHz
    writeReg(0x15, 0x34);  // DEVIATN
    
    // Main radio state machine
    writeReg(0x18, 0x18);  // MCSM0: Auto calibrate on idle->TX/RX
    
    writeReg(0x19, 0x16);  // FOCCFG
    writeReg(0x1B, 0x03);  // AGCCTRL2
    
    // Front end
    writeReg(0x21, 0x56);  // FREND1
    writeReg(0x22, 0x10);  // FREND0
    
    // Calibration
    writeReg(0x23, 0xE9);  // FSCAL3
    writeReg(0x24, 0x2A);  // FSCAL2
    writeReg(0x25, 0x00);  // FSCAL1
    writeReg(0x26, 0x1F);  // FSCAL0
    
    // TX power
    writeReg(0x3E, 0xC0);  // PATABLE
    
    sendCmd(0x33);  // SCAL
    delay(5);
}

void OpenPager::initCC1101RX(uint16_t baud) {
    sendCmd(0x30);  // SRES - Reset
    delay(10);
    
    uint32_t freq_reg = (uint32_t)((_freq * 65536.0) / 26.0);

    // === GDO Configuration ===
    // GDO0: Output async serial data (0x0D)
    writeReg(0x00, 0x06);  // IOCFG2: Asserts when sync word sent/received
    writeReg(0x02, 0x0D);  // IOCFG0: Serial data output (async mode)
    
    // === Packet Configuration ===
    writeReg(0x06, 0xFF);  // PKTLEN: Max packet length (not used in async)
    writeReg(0x07, 0x00);  // PKTCTRL1: No address check
    writeReg(0x08, 0x32);  // PKTCTRL0: Async serial mode, infinite length
    
    // === Frequency Configuration ===
    writeReg(0x0D, (freq_reg >> 16) & 0xFF);  // FREQ2
    writeReg(0x0E, (freq_reg >> 8) & 0xFF);   // FREQ1
    writeReg(0x0F, freq_reg & 0xFF);          // FREQ0
    
    // === Modem Configuration ===
    // Data rate settings based on baud
    // For 512 baud:  MDMCFG4=0xF4, MDMCFG3=0x4B
    // For 1200 baud: MDMCFG4=0xF5, MDMCFG3=0x83
    // For 2400 baud: MDMCFG4=0xF6, MDMCFG3=0x83
    
    // Channel filter bandwidth: needs to be wider for RX
    // Use 0x87 for ~58kHz BW at 512, or 0x88 for wider
    uint8_t mdmcfg4;
    if (baud == 512) {
        mdmcfg4 = 0x87;  // ~58kHz RX bandwidth, 512 baud data rate
    } else if (baud == 2400) {
        mdmcfg4 = 0x88;  // Wider BW for 2400
    } else {
        mdmcfg4 = 0x87;  // Default for 1200
    }
    
    uint8_t mdmcfg3;
    if (baud == 512) mdmcfg3 = 0x4B;
    else if (baud == 2400) mdmcfg3 = 0x83;
    else mdmcfg3 = 0x83;  // 1200 default
    
    writeReg(0x10, mdmcfg4);  // MDMCFG4: RX filter BW + data rate exponent
    writeReg(0x11, mdmcfg3);  // MDMCFG3: Data rate mantissa
    
    // MDMCFG2: 2-FSK modulation, no sync word (we do software sync)
    writeReg(0x12, 0x00);  // 2-FSK, no sync word detection
    
    // MDMCFG1/MDMCFG0: Preamble/channel settings
    writeReg(0x13, 0x22);  // MDMCFG1: 4 preamble bytes, 2 ch spacing exp
    writeReg(0x14, 0xF8);  // MDMCFG0: Channel spacing mantissa
    
    // === Deviation ===
    // POCSAG uses ±4.5kHz deviation
    // With 26MHz crystal: DEVIATION = 4500 / (26e6 / 2^17) = 22.7 -> 0x14 or 0x15
    writeReg(0x15, 0x15);  // DEVIATN: ~4.5kHz deviation
    
    // === Main Radio Control State Machine ===
    writeReg(0x17, 0x30);  // MCSM1: Stay in RX after packet
    writeReg(0x18, 0x18);  // MCSM0: Auto calibrate on IDLE->RX
    
    // === Frequency Offset Compensation ===
    writeReg(0x19, 0x16);  // FOCCFG: Freq offset compensation
    
    // === Bit Synchronization ===
    writeReg(0x1A, 0x6C);  // BSCFG: Bit sync config
    
    // === AGC Configuration ===
    // Higher sensitivity for weak signals
    writeReg(0x1B, 0x03);  // AGCCTRL2: 33dB target amplitude
    writeReg(0x1C, 0x40);  // AGCCTRL1: Carrier sense threshold
    writeReg(0x1D, 0x91);  // AGCCTRL0: AGC config
    
    // === Front End Configuration ===
    writeReg(0x21, 0x56);  // FREND1: RX front end config
    writeReg(0x22, 0x10);  // FREND0: TX front end config
    
    // === Frequency Synthesizer Calibration ===
    writeReg(0x23, 0xE9);  // FSCAL3
    writeReg(0x24, 0x2A);  // FSCAL2
    writeReg(0x25, 0x00);  // FSCAL1
    writeReg(0x26, 0x1F);  // FSCAL0
    
    // === Miscellaneous ===
    writeReg(0x29, 0x59);  // FSTEST
    writeReg(0x2C, 0x81);  // TEST2
    writeReg(0x2D, 0x35);  // TEST1
    writeReg(0x2E, 0x09);  // TEST0
    
    // Calibrate frequency synthesizer
    sendCmd(0x33);  // SCAL
    delay(5);
    
    // Enter RX mode
    sendCmd(0x36);  // SIDLE first
    delay(1);
    sendCmd(0x34);  // SRX - Start receiving
    delay(1);
}

void OpenPager::calcDataRate(uint16_t baud, uint8_t *mdm4, uint8_t *mdm3) {
    if (baud == 512) { *mdm4 = 0xF4; *mdm3 = 0x4B; }
    else if (baud == 2400) { *mdm4 = 0xF6; *mdm3 = 0x83; }
    else { *mdm4 = 0xF5; *mdm3 = 0x83; }  // Default 1200
}

// ============== POCSAG TX Encoding ==============

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

// ============== POCSAG RX Decoding ==============

bool OpenPager::bchCheck(uint32_t cw) {
    // Re-calculate BCH and parity from the data bits
    // If the received codeword is valid, it must match the re-encoded version exactly
    return (bchEncode(cw) == cw);
}

char OpenPager::bcdToChar(uint8_t bcd) {
    // Reverse the 4-bit BCD
    bcd = ((bcd & 0x01) << 3) | ((bcd & 0x02) << 1) | 
          ((bcd & 0x04) >> 1) | ((bcd & 0x08) >> 3);
    
    if (bcd <= 9) return '0' + bcd;
    switch (bcd) {
        case 0x0A: return '*';
        case 0x0B: return 'U';
        case 0x0C: return ' ';
        case 0x0D: return '-';
        case 0x0E: return ')';
        case 0x0F: return '(';
        default: return '?';
    }
}

void OpenPager::processBatch() {
    // Debug callback
    if (_debug_callback) {
        _debug_callback(_rx_codewords, 16);
    }

    for (uint8_t i = 0; i < 16; i++) {
        uint32_t cw = _rx_codewords[i];
        
        // Skip idle codewords
        if (cw == POCSAG_IDLE_CODE) {
            // If we were in a message, finalize it
            if (_rx_in_message && _rx_msg_cw_count > 0) {
                finalizeMessage();
            }
            continue;
        }
        
        // Validate BCH
        if (!bchCheck(cw)) continue;
        
        bool isMessage = (cw >> 31) & 1;
        
        if (!isMessage) {
            // Address codeword - finalize any previous message
            if (_rx_in_message && _rx_msg_cw_count > 0) {
                finalizeMessage();
            }
            
            // Decode address
            uint8_t frame = i / 2;
            uint32_t addrBits = (cw >> 13) & 0x3FFFF;
            _rx_current_ric = (addrBits << 3) | frame;
            _rx_current_func = (cw >> 11) & 0x03;
            _rx_in_message = true;
            _rx_msg_cw_count = 0;
            
        } else if (_rx_in_message) {
            // Message codeword - accumulate
            if (_rx_msg_cw_count < 40) {
                _rx_msg_cws[_rx_msg_cw_count++] = cw;
                _rx_last_msg_activity_us = micros();
            }
        }
    }
    
    // NOTE: Do NOT finalize at batch end!
    // Message may continue in next batch.
    // Only finalize when we see IDLE or new ADDRESS.
}

void OpenPager::finalizeMessage() {
    if (_rx_msg_cw_count == 0) {
        _rx_in_message = false;
        return;
    }
    
    // Determine if numeric or alpha based on function
    if (_rx_current_func == 0 || _rx_current_func == 1) {
        decodeNumericMessage();
    } else {
        decodeAlphaMessage();
    }
    
    _rx_in_message = false;
    _rx_msg_cw_count = 0;
}

void OpenPager::decodeAlphaMessage() {
    PocsagMessage msg;
    msg.ric = _rx_current_ric;
    msg.func = _rx_current_func;
    msg.isNumeric = false;
    msg.valid = true;
    msg.textLen = 0;
    
    uint64_t bitBuf = 0;
    uint8_t bitCount = 0;
    
    for (uint8_t i = 0; i < _rx_msg_cw_count; i++) {
        uint32_t data = (_rx_msg_cws[i] >> 11) & 0xFFFFF;
        
        bitBuf = (bitBuf << 20) | data;
        bitCount += 20;
        
        while (bitCount >= 7 && msg.textLen < 254) {
            bitCount -= 7;
            uint8_t ch = (bitBuf >> bitCount) & 0x7F;
            
            // Reverse bits
            ch = ((ch & 0x01) << 6) | ((ch & 0x02) << 4) | ((ch & 0x04) << 2) |
                 (ch & 0x08) | ((ch & 0x10) >> 2) | ((ch & 0x20) >> 4) | ((ch & 0x40) >> 6);
            
            // Check for EOT or null
            if (ch == 0x04 || ch == 0x00) {
                msg.text[msg.textLen] = '\0';
                pushMessage(msg);
                return;
            }
            
            msg.text[msg.textLen++] = (char)ch;
        }
    }
    
    msg.text[msg.textLen] = '\0';
    if (msg.textLen > 0) {
        pushMessage(msg);
    }
}

void OpenPager::decodeNumericMessage() {
    PocsagMessage msg;
    msg.ric = _rx_current_ric;
    msg.func = _rx_current_func;
    msg.isNumeric = true;
    msg.valid = true;
    msg.textLen = 0;
    
    for (uint8_t i = 0; i < _rx_msg_cw_count; i++) {
        uint32_t data = (_rx_msg_cws[i] >> 11) & 0xFFFFF;
        
        for (int d = 4; d >= 0 && msg.textLen < 254; d--) {
            uint8_t bcd = (data >> (d * 4)) & 0x0F;
            char c = bcdToChar(bcd);
            msg.text[msg.textLen++] = c;
        }
    }
    
    msg.text[msg.textLen] = '\0';
    if (msg.textLen > 0) {
        pushMessage(msg);
    }
}

void OpenPager::pushMessage(PocsagMessage& msg) {
    uint8_t nextHead = (_rx_head + 1) % POCSAG_RX_BUFFER_SIZE;
    
    if (nextHead != _rx_tail) {
        _rx_buffer[_rx_head] = msg;
        _rx_head = nextHead;
        
        if (_rx_callback) {
            _rx_callback(msg);
        }
    }
}

// ============== TX Functions ==============

void OpenPager::transmit(uint32_t ric, uint8_t func, String msg, uint16_t baud, bool alpha) {
    bool wasReceiving = _rx_active;
    if (wasReceiving) {
        stopReceive();
    }
    
    if (baud != _last_baud) {
        initCC1101TX(baud);
        _last_baud = baud;
    }
    
    pinMode(_gdo0, OUTPUT);
    digitalWrite(_gdo0, LOW);

    uint32_t addr_cw = createAddressCW(ric, func);
    uint32_t msg_cw[128];
    uint16_t msg_count = 0;
    
    if (alpha) encodeAlpha(msg, msg_cw, &msg_count);
    else encodeNumeric(msg, msg_cw, &msg_count);

    uint8_t frame = ric & 0x07;
    uint16_t start_slot = frame * 2;
    uint16_t total_cw = 1 + msg_count;
    uint16_t batches = (start_slot + total_cw + 15) / 16;

    sendCmd(0x35);  // STX
    
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
    sendCmd(0x36);  // SIDLE
    
    if (wasReceiving) {
        startReceive(_rx_baud);
    }
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
#if defined(ESP8266) || defined(ESP32)
    yield();
#endif
}
