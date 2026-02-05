#include <OpenPager.h>

// CSN = Pin 15 (D8 on many ESP nodes), GDO0 = Pin 5 (D1)
OpenPager pager(15, 5);

void setup() {
    Serial.begin(115200);
    delay(1000);
    
    // Initialize pager at 433.920 MHz, 1200 baud
    Serial.println("Initializing OpenPager...");
    pager.begin(433.920, 1200);
    
    Serial.println("Ready to transmit!");
}

void loop() {
    // --- 512 Baud Examples ---
    Serial.println("\n>>> Testing 512 Baud <<<");
    pager.transmit(1234567, 3, "Alpha 512 Baud Test", 512, true);
    delay(4000);
    pager.transmit(1234567, 0, "512-512-512", 512, false);
    delay(4000);

    // --- 1200 Baud Examples ---
    Serial.println("\n>>> Testing 1200 Baud <<<");
    pager.transmit(1234567, 3, "Alpha 1200 Baud Test Word", 1200, true);
    delay(4000);
    pager.transmit(1234567, 0, "1200-1200", 1200, false);
    delay(4000);

    // --- 2400 Baud Examples ---
    Serial.println("\n>>> Testing 2400 Baud <<<");
    pager.transmit(1234567, 3, "Alpha 2400 Baud High Speed", 2400, true);
    delay(4000);
    pager.transmit(1234567, 0, "2400-9999", 2400, false);
    delay(8000); // Wait longer before next cycle
}
