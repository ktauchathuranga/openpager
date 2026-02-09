#include "OpenPager.h"

// Global instance pointer for callback trampoline
static OpenPager* g_openPager = nullptr;

// Trampoline callback implementation
void OpenPager::staticCallback(OpenPagerMessage msg) {
    if (g_openPager) {
        // Now valid: static member accessing private member of instance
        if (g_openPager->_rx_callback) {
             g_openPager->_rx_callback(msg);
        }
        
        // Buffer logic
        uint8_t nextHead = (g_openPager->_rx_head + 1) % OPENPAGER_RX_BUFFER_SIZE;
        if (nextHead != g_openPager->_rx_tail) {
            g_openPager->_rx_buffer[g_openPager->_rx_head] = msg;
            g_openPager->_rx_head = nextHead;
        }
    }
}

OpenPager::OpenPager(uint8_t csn_pin, uint8_t gdo0_pin) : 
    _csn(csn_pin), _gdo0(gdo0_pin), _freq(433.920), _last_baud(0), _invert(false),
    _dual_mode(false), _csn_tx(0), _gdo0_tx(0), _saved_csn(0), _saved_gdo0(0),
    _tx_power(OPENPAGER_TX_POWER_MAX),
    _rx_active(false), _rx_config_baud(1200),
    _decoder_count(0),
    _rx_head(0), _rx_tail(0), _rx_callback(nullptr), _debug_callback(nullptr),
    debugSampleCount(0), debugPreambleCount(0), debugSyncCount(0), debugBatchCount(0)
#ifdef ESP32
    , _rmt_rx_buf(nullptr), _rmt_rx_sym_count(0), _rmt_rx_reading(false)
#endif
{
    g_openPager = this;
    for (int i=0; i<3; i++) _decoders[i] = nullptr;
}

OpenPager::OpenPager(uint8_t csn_rx, uint8_t gdo0_rx, uint8_t csn_tx, uint8_t gdo0_tx) : 
    _csn(csn_rx), _gdo0(gdo0_rx), _freq(433.920), _last_baud(0), _invert(false),
    _dual_mode(true), _csn_tx(csn_tx), _gdo0_tx(gdo0_tx), _saved_csn(0), _saved_gdo0(0),
    _tx_power(OPENPAGER_TX_POWER_MAX),
    _rx_active(false), _rx_config_baud(1200),
    _decoder_count(0),
    _rx_head(0), _rx_tail(0), _rx_callback(nullptr), _debug_callback(nullptr),
    debugSampleCount(0), debugPreambleCount(0), debugSyncCount(0), debugBatchCount(0)
#ifdef ESP32
    , _rmt_rx_buf(nullptr), _rmt_rx_sym_count(0), _rmt_rx_reading(false)
#endif
{
    g_openPager = this;
    for (int i=0; i<3; i++) _decoders[i] = nullptr;
}

void OpenPager::swapToTxRadio() {
    if (_dual_mode) {
        _saved_csn = _csn;
        _saved_gdo0 = _gdo0;
        _csn = _csn_tx;
        _gdo0 = _gdo0_tx;
    }
}

void OpenPager::restoreRxRadio() {
    if (_dual_mode) {
        _csn = _saved_csn;
        _gdo0 = _saved_gdo0;
    }
}

void OpenPager::begin(float freq_mhz, uint16_t baud) {
    _freq = freq_mhz;
    pinMode(_csn, OUTPUT);
    digitalWrite(_csn, HIGH);
    
    if (_dual_mode) {
        pinMode(_csn_tx, OUTPUT);
        digitalWrite(_csn_tx, HIGH);
    }
    
    SPI.begin();
    
    // Init the TX radio (or single radio in non-dual mode)
    swapToTxRadio();
    initCC1101TX(baud > 0 ? baud : 2400);
    restoreRxRadio();
    
    _last_baud = baud > 0 ? baud : 2400;
}

void OpenPager::setInvert(bool invert) {
    _invert = invert;
}

void OpenPager::setTxPower(uint8_t power) {
    _tx_power = power;
    // Update the TX radio immediately if already initialized
    swapToTxRadio();
    writeReg(0x3E, _tx_power);
    restoreRxRadio();
}

void OpenPager::setFreq(float freq_mhz) {
    _freq = freq_mhz;
    if (_rx_active) {
        initCC1101RX(_rx_config_baud);
    }
    // Update TX radio frequency
    swapToTxRadio();
    initCC1101TX(_last_baud);
    restoreRxRadio();
}

// ============== RX API ==============

void OpenPager::transmit(uint32_t ric, uint8_t func, String msg, uint16_t baud, bool alpha) {
    // In dual-radio mode, RX stays active. In single-radio mode, stop RX first.
    if (!_dual_mode && _rx_active) stopReceive();

    // Point SPI/GPIO at the TX radio (no-op in single-radio mode)
    swapToTxRadio();

#ifdef ESP32
    transmitRmt(ric, func, msg, baud, alpha);
#else
    if (baud != _last_baud) {
        initCC1101TX(baud);
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
    pinMode(_gdo0, OUTPUT);
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
    pinMode(_gdo0, INPUT);
#endif

    // Restore to RX radio
    restoreRxRadio();
}

void OpenPager::startReceive(uint16_t baud) {
    stopReceive(); // Clear old decoders
    
    // Auto Mode (baud == 0) -> 2400 baud radio, 3 decoders
    // Fixed Mode -> baud radio, 1 decoder
    
    uint16_t radioBaud = (baud == 0) ? 2400 : baud;
    _rx_config_baud = radioBaud;
    
    initCC1101RX(radioBaud);
    
    if (baud == 0) {
        // Auto: 512, 1200, 2400
        _decoders[0] = new OpenPagerDecoder(512, OpenPager::staticCallback);
        _decoders[1] = new OpenPagerDecoder(1200, OpenPager::staticCallback);
        _decoders[2] = new OpenPagerDecoder(2400, OpenPager::staticCallback);
        _decoder_count = 3;
    } else {
        // Fixed
        _decoders[0] = new OpenPagerDecoder(baud, OpenPager::staticCallback);
        _decoder_count = 1;
    }
    
    // inherit settings
    for(int i=0; i<_decoder_count; i++) {
        _decoders[i]->setInvert(_invert);
    }

#ifdef ESP32
    // Use RMT hardware for edge capture — no polling needed
    if (!rmtInit(_gdo0, RMT_RX_MODE, RMT_MEM_NUM_BLOCKS_2, OPENPAGER_RMT_TICK_FREQ)) {
        // Fallback: treat as non-ESP32 path
        pinMode(_gdo0, INPUT);
        _rmt_rx_reading = false;
    } else {
        rmtSetRxMinThreshold(_gdo0, 10);    // Filter glitches < 10 µs
        rmtSetRxMaxThreshold(_gdo0, 50000);  // Idle timeout 50 ms
        
        _rmt_rx_buf = new rmt_data_t[OPENPAGER_RMT_RX_BUF_SYMBOLS];
        _rmt_rx_sym_count = OPENPAGER_RMT_RX_BUF_SYMBOLS;
        rmtReadAsync(_gdo0, _rmt_rx_buf, &_rmt_rx_sym_count);
        _rmt_rx_reading = true;
    }
#else
    // GDO0 as input (polling mode)
    pinMode(_gdo0, INPUT);
#endif
    _rx_active = true;
}

void OpenPager::stopReceive() {
    if (_rx_active) {
        _rx_active = false;
        sendCmd(0x36);  // SIDLE

#ifdef ESP32
        if (_rmt_rx_reading) {
            rmtDeinit(_gdo0);
            _rmt_rx_reading = false;
        }
        if (_rmt_rx_buf) {
            delete[] _rmt_rx_buf;
            _rmt_rx_buf = nullptr;
        }
#endif
    }
    
    // Delete decoders
    for (int i = 0; i < _decoder_count; i++) {
        if (_decoders[i]) {
            delete _decoders[i];
            _decoders[i] = nullptr;
        }
    }
    _decoder_count = 0;
}

bool OpenPager::available() {
    return (_rx_head != _rx_tail);
}

OpenPagerMessage OpenPager::getMessage() {
    OpenPagerMessage msg;
    msg.valid = false;
    
    if (_rx_head != _rx_tail) {
        msg = _rx_buffer[_rx_tail];
        _rx_tail = (_rx_tail + 1) % OPENPAGER_RX_BUFFER_SIZE;
    }
    return msg;
}

void OpenPager::setCallback(OpenPagerCallback cb) {
    _rx_callback = cb;
}

void OpenPager::setDebugCallback(OpenPagerDebugCallback cb) {
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
    if (!_rx_active || _decoder_count == 0) return;
    
#ifdef ESP32
    // RMT hardware captures edges in the background.
    // We just check if a capture completed, process it, and re-arm.
    if (_rmt_rx_reading && rmtReceiveCompleted(_gdo0)) {
        processRmtEdges();
        
        // Re-arm for next capture
        _rmt_rx_sym_count = OPENPAGER_RMT_RX_BUF_SYMBOLS;
        rmtReadAsync(_gdo0, _rmt_rx_buf, &_rmt_rx_sym_count);
    }
#else
    // Legacy polling mode: must be called as fast as possible
    uint32_t now = micros();
    bool bit = digitalRead(_gdo0);
    
    for (uint8_t i = 0; i < _decoder_count; i++) {
        _decoders[i]->process(now, bit);
    }
#endif
    
    // Update generic stats from first decoder (usually 512 or fixed)
    if (_decoders[0]) {
        debugSampleCount = _decoders[0]->debugSampleCount;
        debugSyncCount = _decoders[0]->debugSyncCount;
        debugBatchCount = _decoders[0]->debugBatchCount;
    }
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
    writeReg(0x3E, _tx_power); // PATABLE: TX Power
    
    sendCmd(0x33); // SCAL
    delay(5);
}

void OpenPager::initCC1101RX(uint16_t baud) {
    sendCmd(0x30);
    delay(10);
    
    uint32_t freq_reg = (uint32_t)((_freq * 65536.0) / 26.0);

    writeReg(0x00, 0x06);
    writeReg(0x02, 0x0D);
    
    writeReg(0x06, 0xFF);
    writeReg(0x07, 0x00);
    writeReg(0x08, 0x32);
    
    writeReg(0x0D, (freq_reg >> 16) & 0xFF);
    writeReg(0x0E, (freq_reg >> 8) & 0xFF);
    writeReg(0x0F, freq_reg & 0xFF);
    
    // Bandwidth selection
    uint8_t mdmcfg4;
    // For Auto (2400) or Fixed 2400, we want wider BW
    if (baud >= 2400) {
        mdmcfg4 = 0x88; // Wider BW
    } else {
        mdmcfg4 = 0x87; // 512/1200
    }
    
    uint8_t mdmcfg3;
     if (baud == 512) mdmcfg3 = 0x4B;
    else if (baud == 2400) mdmcfg3 = 0x83;
    else mdmcfg3 = 0x83;
    
    writeReg(0x10, mdmcfg4);
    writeReg(0x11, mdmcfg3);
    
    writeReg(0x12, 0x00);
    writeReg(0x13, 0x22);
    writeReg(0x14, 0xF8);
    writeReg(0x15, 0x15);
    
    writeReg(0x17, 0x30);
    writeReg(0x18, 0x18);
    writeReg(0x19, 0x16);
    writeReg(0x1A, 0x6C);
    
    writeReg(0x1B, 0x03);
    writeReg(0x1C, 0x40);
    writeReg(0x1D, 0x91);
    
    writeReg(0x21, 0x56);
    writeReg(0x22, 0x10);
    
    writeReg(0x23, 0xE9);
    writeReg(0x24, 0x2A);
    writeReg(0x25, 0x00);
    writeReg(0x26, 0x1F);
    
    writeReg(0x29, 0x59);
    writeReg(0x2C, 0x81);
    writeReg(0x2D, 0x35);
    writeReg(0x2E, 0x09);
    
    sendCmd(0x33);
    delay(5);
    
    sendCmd(0x36);
    delay(1);
    sendCmd(0x34);
    delay(1);
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
#if defined(ESP8266) || defined(ESP32)
    yield();
#endif
}

// ============== ESP32 RMT Implementation ==============

#ifdef ESP32

void OpenPager::processRmtEdges() {
    if (_rmt_rx_sym_count == 0) return;
    
    // Convert RMT symbols to flat duration/level arrays for decoders.
    // Each rmt_data_t has two half-symbols: (level0,duration0) and (level1,duration1).
    // Max entries = _rmt_rx_sym_count * 2.
    size_t maxPulses = _rmt_rx_sym_count * 2;
    uint32_t* durations = new uint32_t[maxPulses];
    bool* levels = new bool[maxPulses];
    size_t count = 0;
    
    for (size_t i = 0; i < _rmt_rx_sym_count; i++) {
        if (_rmt_rx_buf[i].duration0 > 0) {
            durations[count] = _rmt_rx_buf[i].duration0;  // already in µs at 1 MHz tick
            levels[count] = _rmt_rx_buf[i].level0;
            count++;
        }
        if (_rmt_rx_buf[i].duration1 > 0) {
            durations[count] = _rmt_rx_buf[i].duration1;
            levels[count] = _rmt_rx_buf[i].level1;
            count++;
        }
    }
    
    // Feed all decoders with the same edge data
    for (uint8_t i = 0; i < _decoder_count; i++) {
        _decoders[i]->processEdgeData(durations, levels, count);
    }
    
    delete[] durations;
    delete[] levels;
}

void OpenPager::transmitRmt(uint32_t ric, uint8_t func, String msg, uint16_t baud, bool alpha) {
    if (baud != _last_baud) {
        initCC1101TX(baud);
        _last_baud = baud;
    }

    // Build POCSAG frame as a flat bitstream
    bool* bits = nullptr;
    size_t bitCount = 0;
    buildPocsagBitstream(ric, func, msg, baud, alpha, &bits, &bitCount);
    
    if (!bits || bitCount == 0) return;

    // Convert bitstream to RMT symbols (2 bits per symbol)
    size_t symCount = (bitCount + 1) / 2;
    rmt_data_t* symbols = new rmt_data_t[symCount];
    uint32_t bitPeriodTicks = (uint32_t)(1000000.0 / (double)baud); // ticks at 1 MHz
    
    for (size_t i = 0; i < symCount; i++) {
        size_t idx0 = i * 2;
        size_t idx1 = i * 2 + 1;
        
        bool b0 = _invert ? !bits[idx0] : bits[idx0];
        symbols[i].level0 = b0 ? 0 : 1;  // POCSAG: mark=LOW, space=HIGH (inverted logic)
        symbols[i].duration0 = bitPeriodTicks;
        
        if (idx1 < bitCount) {
            bool b1 = _invert ? !bits[idx1] : bits[idx1];
            symbols[i].level1 = b1 ? 0 : 1;
            symbols[i].duration1 = bitPeriodTicks;
        } else {
            symbols[i].level1 = 0;
            symbols[i].duration1 = 0;
        }
    }
    
    delete[] bits;
    
    // Init RMT TX channel
    if (!rmtInit(_gdo0, RMT_TX_MODE, RMT_MEM_NUM_BLOCKS_1, OPENPAGER_RMT_TICK_FREQ)) {
        // Fallback: cannot use RMT for TX
        delete[] symbols;
        return;
    }
    rmtSetEOT(_gdo0, 0);  // Pin LOW after TX
    
    // Start CC1101 TX
    sendCmd(0x35); // STX
    
    // Send via RMT hardware (blocking)
    rmtWrite(_gdo0, symbols, symCount, RMT_WAIT_FOR_EVER);
    
    // Done
    sendCmd(0x36); // SIDLE
    rmtDeinit(_gdo0);
    delete[] symbols;
}

void OpenPager::buildPocsagBitstream(uint32_t ric, uint8_t func, String msg, uint16_t baud, bool alpha,
                                     bool** outBits, size_t* outCount) {
    uint32_t addr_cw = createAddressCW(ric, func);
    uint32_t msg_cw[128];
    uint16_t msg_count = 0;
    
    if (alpha) encodeAlpha(msg, msg_cw, &msg_count);
    else encodeNumeric(msg, msg_cw, &msg_count);

    uint8_t frame = ric & 0x07;
    uint16_t total_cw = 1 + msg_count;
    uint16_t batches = (frame * 2 + total_cw + 15) / 16;
    
    // Calculate total bits:
    // Preamble: 20 words * 32 bits = 640
    // Per batch: 1 sync word (32 bits) + 16 codewords (512 bits) = 544 bits
    // Trailing: 4 idle words * 32 bits = 128
    size_t maxBits = 640 + (size_t)batches * 544 + 128;
    bool* bits = new bool[maxBits];
    size_t pos = 0;
    
    // Helper to push a 32-bit word
    auto pushWord = [&](uint32_t word) {
        for (int i = 31; i >= 0; i--) {
            if (pos < maxBits) bits[pos++] = (word >> i) & 1;
        }
    };
    
    // Preamble
    for (int i = 0; i < 20; i++) pushWord(0xAAAAAAAA);
    
    // Batches
    uint16_t msg_idx = 0;
    bool addr_sent = false;
    for (uint16_t b = 0; b < batches; b++) {
        pushWord(POCSAG_SYNC_CODE);
        for (uint8_t f = 0; f < 8; f++) {
            for (uint8_t s = 0; s < 2; s++) {
                if (!addr_sent && f == frame) {
                    pushWord(addr_cw);
                    addr_sent = true;
                } else if (addr_sent && msg_idx < msg_count) {
                    pushWord(msg_cw[msg_idx++]);
                } else {
                    pushWord(POCSAG_IDLE_CODE);
                }
            }
        }
        if (msg_idx >= msg_count) break;
    }
    
    // Trailing idle
    for (int i = 0; i < 4; i++) pushWord(POCSAG_IDLE_CODE);
    
    *outBits = bits;
    *outCount = pos;
}

#endif // ESP32
