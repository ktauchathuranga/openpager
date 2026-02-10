/*
 * NonBlockingDemo — OpenPager non-blocking RX showcase
 *
 * Demonstrates that pager.loop() is lightweight and non-blocking on
 * ESP32 and ESP8266.  A hardware timer ISR samples GDO0 at ~19.2 kHz
 * in the background, so loop() only drains a ring buffer — it never
 * busy-waits or bit-bangs.
 *
 * Three independent tasks run in loop() alongside the pager:
 *   1. LED heartbeat     — toggles every 500 ms
 *   2. Sensor read       — reads analog input every 2 s
 *   3. Uptime printer    — prints seconds since boot every 10 s
 *
 * If any of these stutter or freeze, sampling is blocking loop().
 * On ESP32/ESP8266 they stay perfectly steady.
 *
 * Note: ESP8266 uses Timer1 for the ISR — Servo and analogWrite
 *       (PWM) are NOT available while receiving.
 *
 * Wiring (adjust for your board):
 *   CC1101 CSN  → GPIO 15
 *   CC1101 GDO0 → GPIO 5
 */

#include <OpenPager.h>

// --- Pin config ---
OpenPager pager(15, 5);   // CSN, GDO0

#ifndef LED_BUILTIN
#define LED_BUILTIN 2
#endif
#define HEARTBEAT_LED   LED_BUILTIN
#define HEARTBEAT_MS    500

#define SENSOR_PIN      A0          // Any analog input (or leave floating for demo)
#define SENSOR_MS       2000

#define UPTIME_MS       10000

// --- Pager callback ---
void onMessage(OpenPagerMessage msg) {
    Serial.println("\n========== MESSAGE ==========");
    Serial.printf("RIC : %lu\n", msg.ric);
    Serial.printf("Baud: %d\n", msg.baudRate);
    Serial.printf("Func: %d\n", msg.func);
    Serial.printf("Type: %s\n", msg.isNumeric ? "NUM" : "ALPHA");
    Serial.printf("Text: %s\n", msg.text);
    Serial.println("==============================\n");
}

void setup() {
    Serial.begin(115200);
    delay(500);

    Serial.println("==========================================");
    Serial.println("  OPENPAGER NON-BLOCKING RX DEMO");
    Serial.println("  LED blink + sensor read + uptime print");
    Serial.println("  all run while receiving pager messages.");
    Serial.println("==========================================\n");

    // LED
    pinMode(HEARTBEAT_LED, OUTPUT);

    // Radio
    float freq = 433.000;
    uint16_t baud = 0;  // 0 = auto-baud (512/1200/2400)

    pager.begin(freq, baud);
    pager.setCallback(onMessage);
    pager.startReceive(baud);

    Serial.printf("Listening on %.3f MHz (auto-baud)...\n\n", freq);
}

void loop() {
    // ---- Pager processing (non-blocking) ----
    pager.loop();

    uint32_t now = millis();

    // ---- Task 1: LED heartbeat ----
    static uint32_t lastBlink = 0;
    static bool ledOn = false;
    if (now - lastBlink >= HEARTBEAT_MS) {
        lastBlink = now;
        ledOn = !ledOn;
        digitalWrite(HEARTBEAT_LED, ledOn);
    }

    // ---- Task 2: Analog sensor read ----
    static uint32_t lastSensor = 0;
    if (now - lastSensor >= SENSOR_MS) {
        lastSensor = now;
        int val = analogRead(SENSOR_PIN);
        Serial.printf("[Sensor] A0 = %d\n", val);
    }

    // ---- Task 3: Uptime ----
    static uint32_t lastUptime = 0;
    if (now - lastUptime >= UPTIME_MS) {
        lastUptime = now;
        Serial.printf("[Uptime] %lu seconds | Syncs: %lu | Batches: %lu\n",
                      now / 1000,
                      pager.debugSyncCount,
                      pager.debugBatchCount);
    }
}
