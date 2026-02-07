#include <OpenPager.h>

// CSN = Pin 15 (D8 on ESP8266), GDO0 = Pin 5 (D1)
OpenPager pager(15, 5);

// Debug: print raw codewords in batch
void debugProcessBatch(uint32_t* codewords, uint8_t count) {
    Serial.println("\n--- RAW BATCH ---");
    for (uint8_t i = 0; i < count; i++) {
        uint32_t cw = codewords[i];
        bool isIdle = (cw == 0x7A89C197);
        bool isSync = (cw == 0x7CD215D8);
        bool isMsg = (cw >> 31) & 1;
        
        Serial.printf("[%2d] 0x%08lX %s\n", i, cw, 
                      isIdle ? "(IDLE)" : 
                      isSync ? "(SYNC)" : 
                      isMsg ? "(MSG)" : "(ADDR)");
    }
    Serial.println("-----------------\n");
}

// Callback for received messages
void onMessage(PocsagMessage msg) {
    Serial.println("\n========== MESSAGE ==========");
    Serial.printf("RIC: %lu\n", msg.ric);
    Serial.printf("Func: %d\n", msg.func);
    Serial.printf("Type: %s\n", msg.isNumeric ? "NUM" : "ALPHA");
    Serial.printf("Text: %s\n", msg.text);
    Serial.println("==============================\n");
}

void setup() {
    Serial.begin(115200);
    delay(1000);
    
    Serial.println("================================");
    Serial.println("  POCSAG RX DEBUG MODE          ");
    Serial.println("================================");
    
    float freq = 433.000;
    uint16_t baud = 512;
    
    pager.begin(freq, baud);
    pager.setCallback(onMessage);
    pager.setDebugCallback(debugProcessBatch);
    pager.startReceive(baud);
    
    Serial.printf("RX: %.3f MHz @ %d baud\n\n", freq, baud);
}

void loop() {
    pager.loop();
    
    static uint32_t lastStats = 0;
    if (millis() - lastStats > 3000) {
        lastStats = millis();
        
        Serial.printf("RSSI:%4d | Samples:%lu | Syncs:%lu | Batches:%lu\n",
                      pager.getRSSI(),
                      pager.debugSampleCount,
                      pager.debugSyncCount,
                      pager.debugBatchCount);
    }
}
