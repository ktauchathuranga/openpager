#include <OpenPager.h>

// CSN = Pin 15 (D8 on ESP8266), GDO0 = Pin 5 (D1)
OpenPager pager(15, 5);

void onReceive(PocsagMessage msg) {
    Serial.printf("<<< RX [%lu] %s\n", msg.ric, msg.text);
}

void setup() {
    Serial.begin(115200);
    delay(1000);
    
    Serial.println("OpenPager Loopback Test");
    Serial.println("-----------------------");
    
    pager.begin(433.920, 1200);
    pager.setCallback(onReceive);
}

void loop() {
    // Transmit
    Serial.println(">>> TX Sending 'Hello Loopback'");
    pager.transmit(1234567, 3, "Hello Loopback", 1200, true);
    
    // Start receiving
    pager.startReceive(1200);
    
    // Wait for reception (if using second device, or check for any signals)
    uint32_t start = millis();
    while (millis() - start < 3000) {
        if (pager.available()) {
            PocsagMessage msg = pager.getMessage();
            Serial.printf("RECEIVED: RIC=%lu Func=%d Msg=%s\n", 
                          msg.ric, msg.func, msg.text);
        }
        delay(10);
    }
    
    pager.stopReceive();
    Serial.println();
    delay(2000);
}
