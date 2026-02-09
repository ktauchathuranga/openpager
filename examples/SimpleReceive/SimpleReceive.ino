#include <OpenPager.h>

// CSN = Pin 15 (D8 on ESP8266), GDO0 = Pin 5 (D1)
// Adjust pins for your board!
OpenPager pager(15, 5);

// LED heartbeat — proves loop() is free while the timer ISR handles sampling.
// On ESP32/ESP8266 this blinks steadily even during message reception.
#ifndef LED_BUILTIN
#define LED_BUILTIN 2
#endif
#define HEARTBEAT_LED LED_BUILTIN
#define HEARTBEAT_MS  500

// For dual-radio mode (dedicated RX + TX CC1101):
// OpenPager pager(CSN_RX, GDO0_RX, CSN_TX, GDO0_TX);

// Callback for received messages
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
    delay(1000);
    
    pinMode(HEARTBEAT_LED, OUTPUT);

    Serial.println("================================");
    Serial.println("  OPENPAGER RX DEMO             ");
    Serial.println("================================");
    
    // Initialize Radio
    float freq = 433.000;
    uint16_t baud = 0; // 0 = Auto-Baud (512/1200/2400 parallel)
    
    pager.begin(freq, baud);
    
    // Register callback
    pager.setCallback(onMessage);
    
    // CAP code filtering (optional — omit to receive ALL messages)
    // Only messages addressed to these RICs will be delivered:
    // pager.addCapFilter(1234567);
    // pager.addCapFilter(2000000);
    // pager.removeCapFilter(2000000);  // remove a single filter
    // pager.clearCapFilter();          // remove all filters (back to accept-all)
    
    // Start Listening
    pager.startReceive(baud);
    
    Serial.printf("Listening on %.3f MHz @ %d baud...\n\n", freq, baud);
}

void loop() {
    // Process radio data.
    // On ESP32/ESP8266: hardware timer samples in the background — loop() timing is NOT critical.
    // On other platforms: call loop() as frequently as possible for accurate bit sampling.
    // Note: ESP8266 timer RX uses Timer1 — do NOT use Servo or analogWrite.
    pager.loop();
    
    // --- Non-blocking heartbeat LED ---
    // Toggles every 500 ms without delay(). On ESP32/ESP8266 the hardware
    // timer samples GDO0 in the background, so this blink stays rock-steady
    // even while messages are being decoded.
    static uint32_t lastBlink = 0;
    static bool ledState = false;
    if (millis() - lastBlink >= HEARTBEAT_MS) {
        lastBlink = millis();
        ledState = !ledState;
        digitalWrite(HEARTBEAT_LED, ledState);
    }

    // Print stats every 5 seconds
    static uint32_t lastStats = 0;
    if (millis() - lastStats > 5000) {
        lastStats = millis();
        Serial.printf("RSSI:%d dBm | Edges:%lu | Syncs:%lu | Batches:%lu\n",
                      pager.getRSSI(),
                      pager.debugSampleCount, 
                      pager.debugSyncCount,
                      pager.debugBatchCount);
    }
}
