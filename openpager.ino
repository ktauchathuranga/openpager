/*
 * POCSAG Transmitter - DATARATE FIXED
 * ESP8266 + CC1101
 * 
 * THE FIX: Set CC1101 data rate to match baud rate!
 * In async mode, CC1101 samples GDO0 at 8x the configured data rate
 */

#include <SPI.h>
#include <ESP8266WiFi.h>

// ============================================================================
// Hardware
// ============================================================================
#define PIN_CSN   15
#define PIN_GDO0  5

// ============================================================================
// POCSAG
// ============================================================================
#define POCSAG_SYNC_CODE  0x7CD215D8
#define POCSAG_IDLE_CODE  0x7A89C197
#define BCH_POLY          0x0769

// FSK Polarity Configuration
// Set to 0 for standard POCSAG (high freq = 0, low freq = 1)
// Set to 1 for inverted (some receivers need this)
#define INVERT_FSK  0

// ============================================================================
// CC1101 Driver
// ============================================================================

void cc1101_write(uint8_t reg, uint8_t value) {
    digitalWrite(PIN_CSN, LOW);
    SPI.transfer(reg);
    SPI.transfer(value);
    digitalWrite(PIN_CSN, HIGH);
}

uint8_t cc1101_read(uint8_t reg) {
    digitalWrite(PIN_CSN, LOW);
    SPI.transfer(reg | 0x80);
    uint8_t val = SPI.transfer(0);
    digitalWrite(PIN_CSN, HIGH);
    return val;
}

void cc1101_command(uint8_t cmd) {
    digitalWrite(PIN_CSN, LOW);
    SPI.transfer(cmd);
    digitalWrite(PIN_CSN, HIGH);
}

// Calculate MDMCFG3 and MDMCFG4 for specific baud rate
// Formula: DataRate = (256 + DRATE_M) * 2^DRATE_E * (26MHz / 2^28)
// CRITICAL: These MUST match the baud rate for async serial mode!
void calculate_datarate_regs(uint16_t baud, uint8_t *mdmcfg4, uint8_t *mdmcfg3) {
    // Calculated correctly for 26MHz crystal:
    if (baud == 512) {
        // (256+75) * 2^4 * (26M/2^28) = 331 * 16 * 0.0969 = 513 baud
        *mdmcfg4 = 0xF4;  // DRATE_E = 4
        *mdmcfg3 = 0x4B;  // DRATE_M = 75
    } else if (baud == 1200) {
        // (256+131) * 2^5 * (26M/2^28) = 387 * 32 * 0.0969 = 1199 baud
        *mdmcfg4 = 0xF5;  // DRATE_E = 5
        *mdmcfg3 = 0x83;  // DRATE_M = 131
    } else if (baud == 2400) {
        // (256+131) * 2^6 * (26M/2^28) = 387 * 64 * 0.0969 = 2398 baud
        *mdmcfg4 = 0xF6;  // DRATE_E = 6
        *mdmcfg3 = 0x83;  // DRATE_M = 131
    } else {
        // Default to 1200
        *mdmcfg4 = 0xF5;
        *mdmcfg3 = 0x83;
    }
}

void cc1101_init(float freq_mhz, uint16_t baud) {
    Serial.println("\n========================================");
    Serial.print("Initializing CC1101 at ");
    Serial.print(freq_mhz, 3);
    Serial.print(" MHz for ");
    Serial.print(baud);
    Serial.println(" baud");
    Serial.println("========================================");
    
    pinMode(PIN_CSN, OUTPUT);
    pinMode(PIN_GDO0, OUTPUT);
    digitalWrite(PIN_CSN, HIGH);
    digitalWrite(PIN_GDO0, LOW);
    
    SPI.begin();
    SPI.setBitOrder(MSBFIRST);
    SPI.setDataMode(SPI_MODE0);
    SPI.setFrequency(4000000);
    
    delay(50);
    
    // Reset
    cc1101_command(0x30); // SRES
    delay(10);
    
    // Calculate frequency
    uint32_t freq = (uint32_t)((freq_mhz * 65536.0) / 26.0);
    
    // ========================================================================
    // CRITICAL: Set data rate to match baud rate!
    // ========================================================================
    uint8_t mdmcfg4, mdmcfg3;
    calculate_datarate_regs(baud, &mdmcfg4, &mdmcfg3);
    
    Serial.print("MDMCFG4: 0x");
    Serial.println(mdmcfg4, HEX);
    Serial.print("MDMCFG3: 0x");
    Serial.println(mdmcfg3, HEX);
    
    // GDO Configuration
    cc1101_write(0x00, 0x2E);  // IOCFG2: High-Z
    cc1101_write(0x02, 0x0D);  // IOCFG0: Async serial data input
    
    // Packet Control - ASYNC SERIAL MODE
    cc1101_write(0x08, 0x32);  // PKTCTRL0: Async serial, infinite packet
    
    // Frequency
    cc1101_write(0x0B, 0x06);  // FSCTRL1
    cc1101_write(0x0D, (freq >> 16) & 0xFF);
    cc1101_write(0x0E, (freq >> 8) & 0xFF);
    cc1101_write(0x0F, freq & 0xFF);
    
    // Modem Configuration
    cc1101_write(0x10, mdmcfg4);  // MDMCFG4: BW + Data Rate E
    cc1101_write(0x11, mdmcfg3);  // MDMCFG3: Data Rate M
    cc1101_write(0x12, 0x00);     // MDMCFG2: 2-FSK, no sync
    cc1101_write(0x13, 0x22);     // MDMCFG1
    
    // Deviation: ±4.5kHz for POCSAG
    // Formula: Dev = Fxosc/2^17 * (8+M) * 2^E
    // 0x13: E=1, M=3 → 26M/131072 * 11 * 2 = 4357 Hz (~4.36 kHz)
    cc1101_write(0x15, 0x13);  // DEVIATN
    
    // Main Radio Control
    cc1101_write(0x18, 0x18);  // MCSM0: Auto-cal
    
    // Frequency offset compensation
    cc1101_write(0x19, 0x16);  // FOCCFG
    
    // AGC
    cc1101_write(0x1B, 0x03);  // AGCCTRL2
    
    // Front end
    cc1101_write(0x21, 0x56);  // FREND1
    cc1101_write(0x22, 0x10);  // FREND0
    
    // Frequency calibration
    cc1101_write(0x23, 0xE9);  // FSCAL3
    cc1101_write(0x24, 0x2A);  // FSCAL2
    cc1101_write(0x25, 0x00);  // FSCAL1
    cc1101_write(0x26, 0x1F);  // FSCAL0
    
    // Test registers
    cc1101_write(0x2C, 0x81);  // TEST2
    cc1101_write(0x2D, 0x35);  // TEST1
    cc1101_write(0x2E, 0x09);  // TEST0
    
    // Power table
    digitalWrite(PIN_CSN, LOW);
    SPI.transfer(0x7E);  // PATABLE burst
    // 0x60 = -6dBm (Reduced from 0xC0/+10dBm)
    // 0xC0 = +10dBm, 0x50 = -15dBm, 0x12 = -20dBm
    for (int i = 0; i < 8; i++) SPI.transfer(0x60);
    digitalWrite(PIN_CSN, HIGH);
    
    // Calibrate
    cc1101_command(0x36); // SIDLE
    delay(2);
    cc1101_command(0x33); // SCAL
    delay(10);
    cc1101_command(0x36); // SIDLE
    
    // Verify
    Serial.println("\nVerifying registers:");
    Serial.print("PKTCTRL0: 0x");
    Serial.println(cc1101_read(0x08), HEX);
    Serial.print("MDMCFG2: 0x");
    Serial.println(cc1101_read(0x12), HEX);
    Serial.print("MDMCFG4: 0x");
    Serial.println(cc1101_read(0x10), HEX);
    Serial.print("MDMCFG3: 0x");
    Serial.println(cc1101_read(0x11), HEX);
    
    Serial.println("\nCC1101 ready!");
    Serial.println("========================================\n");
}

// ============================================================================
// POCSAG Encoder
// ============================================================================

// Reverse all 8 bits of a byte (matches pager-tx reverse_bits_8)
uint8_t reverse_bits_8(uint8_t x) {
    x = ((x & 0xAA) >> 1) | ((x & 0x55) << 1);
    x = ((x & 0xCC) >> 2) | ((x & 0x33) << 2);
    x = (x >> 4) | (x << 4);
    return x;
}

// Reverse lower N bits
uint8_t reverse_nbit(uint8_t x, uint8_t n) {
    uint8_t reversed = reverse_bits_8(x);
    return reversed >> (8 - n);
}

// BCD encoding for numeric mode
uint8_t char_to_bcd(char c) {
    if (c >= '0' && c <= '9') return c - '0';
    switch (c) {
        case '*': case 'U': case 'u': return 0x0B;
        case ' ': return 0x0C;
        case '-': return 0x0D;
        case ')': case ']': return 0x0E;
        case '(': case '[': return 0x0F;
    }
    return 0x0C; // Default to space
}

void encode_numeric_message(String msg, uint32_t *codewords, uint16_t *count) {
    uint16_t cw_index = 0;
    uint32_t current_word = 0;
    uint8_t digit_count = 0;

    for (uint16_t i = 0; i < msg.length(); i++) {
        uint8_t bcd = char_to_bcd(msg.charAt(i));
        uint8_t reversed_bcd = reverse_nbit(bcd, 4);

        current_word = (current_word << 4) | (reversed_bcd & 0x0F);
        digit_count++;

        if (digit_count == 5) {
            uint32_t raw = (1UL << 31) | (current_word << 11);
            codewords[cw_index++] = bch_encode(raw);
            current_word = 0;
            digit_count = 0;
            if (cw_index >= 100) { *count = cw_index; return; }
        }
    }

    // Pad remaining digits with reversed spaces (0xC reversed = 0x3)
    if (digit_count > 0) {
        while (digit_count < 5) {
            uint8_t reversed_space = reverse_nbit(0x0C, 4); // 0x3
            current_word = (current_word << 4) | reversed_space;
            digit_count++;
        }
        uint32_t raw = (1UL << 31) | (current_word << 11);
        codewords[cw_index++] = bch_encode(raw);
    }

    *count = cw_index;
}

// BCH(31,21) encoder - MATCHES pager-tx exactly
// Generator: x^10 + x^9 + x^8 + x^6 + x^5 + x^3 + 1 = 0x769
// Shifted left by 21: 0xED200000
uint32_t bch_encode(uint32_t cw) {
    uint32_t local_cw = cw & 0xFFFFF800;  // Keep bits 31..11, clear BCH and parity
    uint32_t cw_e = local_cw;
    
    // Calculate BCH bits - process 21 data bits
    for (int i = 0; i < 21; i++) {
        if (cw_e & 0x80000000) {
            cw_e ^= 0xED200000;  // Generator polynomial 0x769 << 21
        }
        cw_e <<= 1;
    }
    
    // Combine original data with calculated BCH parity (in bits 10..1)
    uint32_t result = local_cw | (cw_e >> 21);
    
    // Calculate even parity for bits 31..1
    uint8_t parity = 0;
    uint32_t w = result >> 1;
    while (w != 0) {
        parity ^= (w & 1);
        w >>= 1;
    }
    
    return result | parity;
}

uint32_t create_address_cw(uint32_t ric, uint8_t func) {
    uint32_t address = (ric >> 3) & 0x3FFFF;
    uint32_t data20 = (address << 2) | (func & 0x03);
    uint32_t raw = (data20 << 11);
    return bch_encode(raw);
}

// Encode message to match pager-tx EXACTLY
// Process message chars in loop, add EOT AFTER loop with padding
void encode_message(String msg, uint32_t *codewords, uint16_t *count) {
    uint64_t buffer = 0;
    uint8_t bits_in_buffer = 0;
    uint16_t cw_index = 0;
    
    // Process message characters (NOT including EOT yet)
    for (uint16_t i = 0; i < msg.length(); i++) {
        uint8_t c = reverse_nbit((uint8_t)msg.charAt(i), 7);
        buffer = (buffer << 7) | (c & 0x7F);
        bits_in_buffer += 7;
        
        // Extract 20-bit words when we have enough
        while (bits_in_buffer >= 20) {
            bits_in_buffer -= 20;
            uint32_t data20 = (buffer >> bits_in_buffer) & 0xFFFFF;
            uint32_t raw = (1UL << 31) | (data20 << 11);
            codewords[cw_index++] = bch_encode(raw);
            if (cw_index >= 100) { *count = cw_index; return; }
        }
    }
    
    // Add EOT (0x04) AFTER the message loop (matching pager-tx)
    uint8_t eot_reversed = reverse_nbit(0x04, 7);
    buffer = (buffer << 7) | (eot_reversed & 0x7F);
    bits_in_buffer += 7;
    
    // Pad remaining bits with zeros to complete final word
    if (bits_in_buffer > 0) {
        while (bits_in_buffer < 20) {
            buffer <<= 1;
            bits_in_buffer++;
        }
        uint32_t data20 = (buffer >> (bits_in_buffer - 20)) & 0xFFFFF;
        uint32_t raw = (1UL << 31) | (data20 << 11);
        codewords[cw_index++] = bch_encode(raw);
    }
    
    *count = cw_index;
}

// ============================================================================
// Transmission with PRECISE TIMING
// ============================================================================

// Bit timing globals
uint32_t g_bit_start_us = 0;
double g_bit_period_us = 0;
uint32_t g_total_bits_sent = 0;

void send_bit_precise(bool bit) {
    bool b = bit;
    if (INVERT_FSK) b = !b;
    
    digitalWrite(PIN_GDO0, b ? LOW : HIGH);
    
    g_total_bits_sent++;
    
    // Calculate ABSOLUTE target time from the very beginning of transmission
    // This eliminates any cumulative rounding error from integer division
    uint32_t target_us = g_bit_start_us + (uint32_t)(g_total_bits_sent * g_bit_period_us);
    
    // Wait until target time
    while ((int32_t)(target_us - micros()) > 0) {
        // High precision busy-wait
    }
}

void send_word_precise(uint32_t word) {
    // Disable interrupts for higher baud rates to prevent jitter
    noInterrupts();
    for (int i = 31; i >= 0; i--) {
        send_bit_precise((word >> i) & 1);
    }
    interrupts();
    
    ESP.wdtFeed();
}

void transmit_pocsag(uint32_t ric, uint8_t func, String msg, uint16_t baud, bool alpha) {
    Serial.println("\n========================================");
    Serial.print("RIC: "); Serial.println(ric);
    Serial.print("Function: "); Serial.println(func);
    Serial.print("Message: "); Serial.println(msg);
    Serial.print("Mode: "); Serial.println(alpha ? "Alpha" : "Numeric");
    Serial.print("Baud: "); Serial.println(baud);
    
    uint32_t bit_delay = 1000000 / baud;
    Serial.print("Bit delay: "); Serial.print(bit_delay); Serial.println(" us");
    
    // Encode
    uint32_t addr_cw = create_address_cw(ric, func);
    Serial.print("Address CW: 0x");
    Serial.println(addr_cw, HEX);
    
    uint32_t msg_cw[100];
    uint16_t msg_count = 0;
    if (alpha) {
        encode_message(msg, msg_cw, &msg_count);
    } else {
        encode_numeric_message(msg, msg_cw, &msg_count);
    }
    Serial.print("Message CWs: ");
    Serial.println(msg_count);
    
    // Debug: Print all codewords
    for (uint16_t i = 0; i < msg_count; i++) {
        Serial.print("  msg_cw[");
        Serial.print(i);
        Serial.print("] = 0x");
        Serial.println(msg_cw[i], HEX);
    }
    
    uint8_t frame = ric & 0x07;
    Serial.print("Frame: ");
    Serial.println(frame);
    Serial.println("========================================");
    
    // Start TX
    cc1101_command(0x36); // SIDLE
    delay(2);
    cc1101_command(0x33); // SCAL
    delay(5);
    cc1101_command(0x35); // STX
    delay(1);
    
    Serial.println("Transmitting...");
    
    // Initialize precise timing globals
    g_bit_period_us = 1000000.0 / (double)baud;
    g_total_bits_sent = 0;
    g_bit_start_us = micros();
    
    // Preamble - 20 words (640 bits) for extra reliability
    for (int i = 0; i < 20; i++) {
        send_word_precise(0xAAAAAAAA);
    }
    
    // Batches
    // CRITICAL: Calculate batches based on when the message starts!
    uint16_t start_slot = frame * 2;
    uint16_t total_cw = 1 + msg_count;
    uint16_t total_slots_needed = start_slot + total_cw;
    uint16_t batches = (total_slots_needed + 15) / 16;
    
    uint16_t msg_idx = 0;
    bool addr_sent = false;
    
    Serial.print("Required slots: "); Serial.println(total_slots_needed);
    Serial.print("Sending batches: "); Serial.println(batches);
    
    for (uint16_t batch = 0; batch < batches; batch++) {
        send_word_precise(POCSAG_SYNC_CODE);
        
        for (uint8_t f = 0; f < 8; f++) {
            for (uint8_t slot = 0; slot < 2; slot++) {
                if (!addr_sent && f == frame) {
                    send_word_precise(addr_cw);
                    addr_sent = true;
                } else if (addr_sent && msg_idx < msg_count) {
                    send_word_precise(msg_cw[msg_idx++]);
                } else {
                    send_word_precise(POCSAG_IDLE_CODE);
                }
            }
        }
        
        if (msg_idx >= msg_count) break;
    }
    
    // Tail - send at least 2 full idle words to ensure receiver flushes buffer
    for (int i = 0; i < 4; i++) {
        send_word_precise(POCSAG_IDLE_CODE);
    }
    
    // Stop TX
    digitalWrite(PIN_GDO0, LOW);
    delay(1);
    cc1101_command(0x36); // SIDLE
    
    Serial.println("Done!");
    Serial.println("========================================\n");
}

// ============================================================================
// Setup & Loop
// ============================================================================

void setup() {
    Serial.begin(115200);
    delay(1000);
    
    Serial.println("\n\n========================================");
    Serial.println("POCSAG TX - DATARATE FIXED");
    Serial.println("========================================");
    
    WiFi.mode(WIFI_OFF);
    WiFi.forceSleepBegin();
    delay(100);
    
    // Initialize for 1200 baud
    cc1101_init(433.920, 2400);
    
    Serial.println("Starting in 5 seconds...\n");
    delay(5000);
}

void loop() {
    // Alphanumeric test
    transmit_pocsag(1234567, 3, "ashen ashen its ashen ane", 2400, true);
    delay(5000);
    
    // Numeric test
    transmit_pocsag(1234567, 0, "0123456789*-()", 2400, false);
    delay(5000);
}