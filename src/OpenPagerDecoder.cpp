#include "OpenPagerDecoder.h"
#include <SPI.h> // For any shared defs if needed, though mostly math here

OpenPagerDecoder::OpenPagerDecoder(uint16_t baud, OpenPagerCallback callback) 
    : _baud(baud), _callback(callback), _invert(false), 
      _next_sample_time(0), _last_bit(false),
      _shift_reg(0), _bit_count(0),
      _synced(false), _inverted(false), 
      _cw_index(0), _bit_period_us(0),
      _msg_cw_count(0), _in_message(false), _last_msg_activity_us(0)
{
    _bit_period_us = 1000000.0 / (double)baud;
    
    // Initialize debug counters
    debugSampleCount = 0;
    debugSyncCount = 0;
    debugBatchCount = 0;
}

void OpenPagerDecoder::reset() {
    _shift_reg = 0;
    _bit_count = 0;
    _synced = false;
    _inverted = false;
    _cw_index = 0;
    _in_message = false;
    _msg_cw_count = 0;
    _next_sample_time = 0;
}

void OpenPagerDecoder::setInvert(bool invert) {
    _invert = invert;
}

void OpenPagerDecoder::processEdgeData(const uint32_t* durations_us, const bool* levels, size_t count) {
    for (size_t i = 0; i < count; i++) {
        bool level = levels[i];
        if (_invert) level = !level;
        
        uint32_t duration = durations_us[i];
        // Calculate how many bits this pulse represents at our baud rate
        double bits_f = (double)duration / _bit_period_us;
        uint32_t numBits = (uint32_t)(bits_f + 0.5); // round
        if (numBits == 0) numBits = 1;
        if (numBits > 64) numBits = 64; // sanity cap
        
        for (uint32_t b = 0; b < numBits; b++) {
            debugSampleCount++;
            processSample(level);
        }
    }
    
    // Don't finalize here — message may span multiple RMT chunks.
    // Finalization happens in processBatch() on IDLE codeword,
    // or via timeout in the polling process() path.
}

void OpenPagerDecoder::process(uint32_t now, bool bit) {
    // User inversion setting
    if (_invert) bit = !bit;
    
    // Edge detection logic specific to this baud rate
    // Note: All decoders see the same edges, but track their own timing
    if (bit != _last_bit) {
        _last_bit = bit;
        
        // Resync - set next sample to middle of bit
        _next_sample_time = now + (uint32_t)(_bit_period_us * 0.5);
    }
    
    // Check if it's time to sample
    if ((int32_t)(now - _next_sample_time) >= 0 && _next_sample_time != 0) {
        debugSampleCount++;
        processSample(bit);
        
        // Schedule next
        _next_sample_time += (uint32_t)_bit_period_us;
        
        // Anti-runaway
        if ((int32_t)(now - _next_sample_time) > (int32_t)(_bit_period_us * 3)) {
            _next_sample_time = 0;
        }
    }
    
    // Timeout logic
    if (_in_message && _msg_cw_count > 0) {
        uint32_t timeout_us = (uint32_t)(_bit_period_us * 32 * 20); // ~20 codewords
        if ((int32_t)(now - _last_msg_activity_us) > (int32_t)timeout_us) {
            finalizeMessage();
        }
    }
}

void OpenPagerDecoder::processSample(bool bit) {
     // Shift in RAW bit (no auto-inversion here)
     _shift_reg = (_shift_reg << 1) | (bit ? 1 : 0);
     _bit_count++;
     
     if (!_synced) {
         if (checkSyncWord()) {
             _synced = true;
             _bit_count = 0;
             _cw_index = 0;
             debugSyncCount++;
         }
     } else {
         if (_bit_count == 32) {
             _codewords[_cw_index] = _shift_reg;
             _cw_index++;
             _bit_count = 0;
             
             if (_cw_index >= 16) {
                 debugBatchCount++;
                 processBatch();
                 _synced = false;
                 _cw_index = 0;
             }
         }
     }
}

bool OpenPagerDecoder::checkSyncWord() {
    auto bitDiff = [](uint32_t a, uint32_t b) {
        uint32_t x = a ^ b;
        uint32_t count = 0;
        while (x) { count++; x &= (x - 1); }
        return count;
    };
    
    if (bitDiff(_shift_reg, POCSAG_SYNC_CODE) <= 2) {
        _inverted = false;
        return true;
    }
    // Check inverted sync (means signal is inverted)
    if (bitDiff(_shift_reg, POCSAG_SYNC_INVERTED) <= 2) {
        _inverted = true;
        return true;
    }
    // Check bit-inverted sync (legacy check)
    if (bitDiff(~_shift_reg, POCSAG_SYNC_CODE) <= 2) {
        _inverted = true;
        return true;
    }
    return false;
}

void OpenPagerDecoder::processBatch() {
    for (uint8_t i = 0; i < 16; i++) {
        uint32_t cw = _codewords[i];
        if (_inverted) cw = ~cw;
        
        if (cw == POCSAG_IDLE_CODE) {
            if (_in_message) finalizeMessage();
            continue;
        }
        
        bool bchOk = bchCheck(cw);
        if (!bchOk) {
            uint32_t corrected = bchRepair(cw);
            if (corrected != 0) {
                cw = corrected;
                bchOk = true;
            }
        }
        
        if (!bchOk) continue;
        
        bool isMessage = (cw >> 31) & 1;
        
        if (!isMessage) {
            if (_in_message) finalizeMessage();
            
            uint32_t address = (cw >> 13) & 0x7FFFF;
            uint8_t func = (cw >> 11) & 0x03;
            
            // Calculate full 21-bit RIC (Base 18 bits + 3 bits frame)
            _current_ric = (address << 3) | (i / 2);
            _current_func = func;
            
            _in_message = true;
            _msg_cw_count = 0;
        } else if (_in_message) {
            if (_msg_cw_count < 64) {
                _msg_cws[_msg_cw_count++] = cw;
            }
        }
    }
    _last_msg_activity_us = micros();
}

void OpenPagerDecoder::finalizeMessage() {
    if (_msg_cw_count == 0) {
        // Tone Only
        OpenPagerMessage msg;
        msg.ric = _current_ric;
        msg.func = _current_func;
        msg.baudRate = _baud;
        msg.isNumeric = (_current_func == 0 || _current_func == 1);
        strcpy(msg.text, "[Tone Only]");
        msg.textLen = strlen(msg.text);
        msg.valid = true;
        if (_callback) _callback(msg);
    } else {
        OpenPagerMessage msg;
        msg.ric = _current_ric;
        msg.func = _current_func;
        msg.baudRate = _baud;
        msg.isNumeric = (_current_func == 0 || _current_func == 1);
        msg.valid = true;
        msg.textLen = 0;
        
        if (msg.isNumeric) decodeNumericMessage(msg);
        else decodeAlphaMessage(msg);
        
        if (_callback) _callback(msg);
    }
    
    _in_message = false;
    _msg_cw_count = 0;
}

// ... Decoding helpers ...

void OpenPagerDecoder::decodeAlphaMessage(OpenPagerMessage& msg) {
    uint64_t bitBuf = 0;
    uint8_t bitCount = 0;
    
    for (uint8_t i = 0; i < _msg_cw_count; i++) {
        uint32_t data = (_msg_cws[i] >> 11) & 0xFFFFF;
        bitBuf = (bitBuf << 20) | data;
        bitCount += 20;
        
        while (bitCount >= 7 && msg.textLen < 254) {
            bitCount -= 7;
            uint8_t ch = (bitBuf >> bitCount) & 0x7F;
            
            // Reverse bits
            ch = ((ch & 0x01) << 6) | ((ch & 0x02) << 4) | ((ch & 0x04) << 2) |
                 (ch & 0x08) | ((ch & 0x10) >> 2) | ((ch & 0x20) >> 4) | ((ch & 0x40) >> 6);
            
            if (ch == 0x04 || ch == 0x00) {
                msg.text[msg.textLen] = '\0';
                return;
            }
            msg.text[msg.textLen++] = (char)ch;
        }
    }
    msg.text[msg.textLen] = '\0';
}

void OpenPagerDecoder::decodeNumericMessage(OpenPagerMessage& msg) {
    for (uint8_t i = 0; i < _msg_cw_count; i++) {
        uint32_t data = (_msg_cws[i] >> 11) & 0xFFFFF;
        for (int d = 4; d >= 0 && msg.textLen < 254; d--) {
            uint8_t bcd = (data >> (d * 4)) & 0x0F;
            char c = bcdToChar(bcd);
            msg.text[msg.textLen++] = c;
        }
    }
    msg.text[msg.textLen] = '\0';
}

uint32_t OpenPagerDecoder::bchEncode(uint32_t data) {
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

bool OpenPagerDecoder::bchCheck(uint32_t cw) {
    return (bchEncode(cw) == cw);
}

uint32_t OpenPagerDecoder::bchRepair(uint32_t cw) {
    if (bchCheck(cw)) return cw;
    for (int i = 0; i < 32; i++) {
        uint32_t corrected = cw ^ (1UL << i);
        if (bchCheck(corrected)) return corrected;
    }
    return 0;
}

char OpenPagerDecoder::bcdToChar(uint8_t bcd) {
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
