#include <OpenPager.h>

// CSN = Pin 15 (D8 on ESP8266), GDO0 = Pin 5 (D1)
// Adjust pins for your board!
OpenPager pager(15, 5);

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
    
    Serial.println("================================");
    Serial.println("  OPENPAGER RX DEMO             ");
    Serial.println("================================");
    
    // Initialize Radio
    float freq = 433.000;
    uint16_t baud = 0; // 0 = Auto-Baud (512/1200/2400 parallel)
    
    pager.begin(freq, baud);
    
    // Register callback
    pager.setCallback(onMessage);
    
    // Start Listening
    pager.startReceive(baud);
    
    Serial.printf("Listening on %.3f MHz @ %d baud...\n\n", freq, baud);
}

void loop() {
    // Process radio data.
    // On ESP32: RMT hardware captures edges in the background — loop() timing is NOT critical.
    // On other platforms: call loop() as frequently as possible for accurate bit sampling.
    pager.loop();
    
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
