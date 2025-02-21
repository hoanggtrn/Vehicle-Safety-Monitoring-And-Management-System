#include <SPI.h>
#include <MFRC522.h>

#define SS_PIN 21  // Chân SDA (SS) nối với GPIO 21
#define RST_PIN 22 // Chân RST nối với GPIO 22
#define SCK_PIN 18 // Chân SCK nối với GPIO 18
#define MOSI_PIN 23 // Chân MOSI nối với GPIO 23
#define MISO_PIN 19 // Chân MISO nối với GPIO 19

MFRC522 rfid(SS_PIN, RST_PIN); // Khởi tạo đối tượng MFRC522

void setup() {
  Serial.begin(115200); // Khởi động Serial Monitor
  SPI.begin(SCK_PIN, MISO_PIN, MOSI_PIN, SS_PIN); // Khởi động giao thức SPI
  rfid.PCD_Init();     // Khởi động module RFID
  Serial.println("RFID Reader Initialized");
}

void loop() {
  // Kiểm tra xem có thẻ nào không
  if (rfid.PICC_IsNewCardPresent()) {
    if (rfid.PICC_ReadCardSerial()) {
      Serial.print("UID:");
      // In UID của thẻ ra Serial Monitor
      for (byte i = 0; i < rfid.uid.size; i++) {
        Serial.print(rfid.uid.uidByte[i], HEX);
        Serial.print(" ");
      }
      Serial.println();
      rfid.PICC_HaltA(); // Dừng thẻ
    }
  }
}
