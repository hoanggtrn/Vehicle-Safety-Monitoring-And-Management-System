#include <WiFi.h>

void setup() {
    Serial.begin(115200);
    delay(1000);
    
    // Kiểm tra địa chỉ MAC hiện tại
    String currentMac = WiFi.macAddress();
    Serial.print("Địa chỉ MAC hiện tại: ");
    Serial.println(currentMac);
    // D8:BC:38:FB:B1:90 mac bên trái
    // D8:BC:38:FB:73:44 mac bên phải
    // CC:DB:A7:8F:C0:F4 master
    // 2C:BC:BB:0C:50:7C slave
    // D8:BC:38:FB:B1:90 debug

}

void loop() {}
