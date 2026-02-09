#include <OpenPager.h>

// This example is ESP32 only — deep sleep requires RTC GPIOs and esp_sleep.
#ifndef ESP32
#error "DeepSleepReceive requires ESP32. Use SimpleReceive for other platforms."
#endif

// CSN = Pin 15, GDO0 = Pin 5 (data), GDO2 = Pin 4 (carrier sense → wake)
// GDO2 must be connected to an RTC-capable GPIO on ESP32.
// RTC GPIOs: 0, 2, 4, 12, 13, 14, 15, 25, 26, 27, 32, 33, 34, 35, 36, 39
OpenPager pager(15, 5);

#define GDO2_PIN 4  // CC1101 GDO2 → ESP32 RTC GPIO

void onMessage(OpenPagerMessage msg) {
    Serial.println("\n========== MESSAGE ==========");
    Serial.printf("RIC: %lu\n", msg.ric);
    Serial.printf("Baud: %d\n", msg.baudRate);
    Serial.printf("Func: %d\n", msg.func);
    Serial.printf("Type: %s\n", msg.isNumeric ? "NUM" : "ALPHA");
    Serial.printf("Text: %s\n", msg.text);
    Serial.println("==============================\n");
}

void setup() {
    Serial.begin(115200);
    delay(500);

    // Check if we woke from deep sleep
    if (OpenPager::wokeFromSleep()) {
        Serial.println("*** Woke from deep sleep (carrier detected!) ***");
    } else {
        Serial.println("Normal boot");
    }

    Serial.println("================================");
    Serial.println("  OPENPAGER DEEP SLEEP RX DEMO  ");
    Serial.println("================================");

    // Initialize Radio
    float freq = 433.000;
    uint16_t baud = 0; // Auto-Baud

    pager.begin(freq, baud);
    pager.setCallback(onMessage);

    // CAP code filtering (optional — omit to receive ALL messages)
    // Only deliver messages addressed to these RICs:
    // pager.addCapFilter(1234567);

    // Enable deep sleep wakeup via GDO2 (carrier sense)
    // This configures CC1101 GDO2 pin to output carrier sense signal.
    pager.setWakePin(GDO2_PIN);

    // Start listening
    pager.startReceive(baud);

    Serial.printf("Listening on %.3f MHz (auto-baud)...\n", freq);
    Serial.println("Will sleep after 30s of no messages.\n");
}

void loop() {
    pager.loop();

    // Auto-sleep after 30 seconds of no received messages.
    // The CC1101 stays in RX mode during deep sleep.
    // When a transmission is detected (carrier sense), ESP32 wakes and reboots.
    pager.sleepAfterTimeout(30000);

    // Print stats every 5 seconds
    static uint32_t lastStats = 0;
    if (millis() - lastStats > 5000) {
        lastStats = millis();
        Serial.printf("RSSI:%d dBm | Samples:%lu | Syncs:%lu | Batches:%lu\n",
                      pager.getRSSI(),
                      pager.debugSampleCount,
                      pager.debugSyncCount,
                      pager.debugBatchCount);
    }
}
