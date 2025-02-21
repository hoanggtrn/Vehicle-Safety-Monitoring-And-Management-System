#include <TinyGPS++.h>
#include <HardwareSerial.h>

TinyGPSPlus gps;
HardwareSerial ss(1);

void setup() {
  Serial.begin(115200);
  ss.begin(9600, SERIAL_8N1, 16, 17);  // Chân RX là GPIO 16 và TX là GPIO 17
  Serial.println("Đang chờ dữ liệu GPS...");
}

void loop() {
  // Đọc dữ liệu từ GPS
  while (ss.available() > 0) {
    gps.encode(ss.read());
  }

  // Kiểm tra và hiển thị thông tin GPS nếu có
  if (gps.location.isUpdated()) {
    Serial.print("Vĩ độ: ");
    Serial.print(gps.location.lat(), 6);
    Serial.print("Kinh độ: ");
    Serial.print(gps.location.lng(), 6);
    Serial.print("Độ cao: ");
    Serial.print(gps.altitude.meters());
    Serial.print("Số vệ tinh: ");
    Serial.print(gps.satellites.value());
    Serial.print("Tốc độ: ");
    Serial.print(gps.speed.kmph());
    Serial.println();
  }

  delay(1000);  // Dừng một giây giữa các lần cập nhật
}
