#include <TinyGPS++.h>
#include <HardwareSerial.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

#define SCREEN_WIDTH 128 // Width of the screen in pixels
#define SCREEN_HEIGHT 64  // Height of the screen in pixels
#define RX_PIN 16         // Chân nhận dữ liệu từ GPS
#define TX_PIN 17         // Chân truyền dữ liệu tới GPS
#define OLED_RESET -1     // Không sử dụng chân reset

TinyGPSPlus gps; // Khởi tạo đối tượng GPS
HardwareSerial mySerial(1); // Khởi tạo serial thứ hai cho GPS
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET); // Khởi tạo màn hình OLED

void setup() {
  Serial.begin(9600); // Khởi động Serial Monitor
  mySerial.begin(9600, SERIAL_8N1, RX_PIN, TX_PIN); // Khởi động Serial GPS
  
  // Khởi động màn hình OLED
  display.begin(SSD1306_SWITCHCAPVCC, 0x3C); // Địa chỉ I2C của màn hình OLED
  display.clearDisplay();

  // Hiển thị lời chào
  display.setTextSize(2); // Kích thước chữ lớn hơn cho lời chào
  display.setTextColor(SSD1306_WHITE); // Màu chữ
  display.setCursor(0, 0);
  display.println("Chao mung!"); // Lời chào
  display.setTextSize(1); // Đặt kích thước chữ về bình thường
  display.setCursor(0, 30);
  display.println("Dang cho GPS..."); // Dòng chờ
  display.display(); // Cập nhật màn hình OLED

  delay(2000); // Giữ hiển thị lời chào trong 2 giây
  display.clearDisplay(); // Xóa màn hình sau khi hiển thị
}

void loop() {
  // Kiểm tra dữ liệu từ GPS
  while (mySerial.available()) {
    gps.encode(mySerial.read()); // Nhận dữ liệu từ GPS

    // Nếu có dữ liệu GPS hợp lệ
    if (gps.location.isUpdated()) {
      // Hiển thị thông tin lên màn hình OLED
      display.clearDisplay();
      display.setCursor(0, 0);
      display.print("Vĩ độ: ");
      display.println(gps.location.lat(), 6); // In ra vĩ độ với 6 chữ số thập phân
      display.print("Kinh độ: ");
      display.println(gps.location.lng(), 6); // In ra kinh độ với 6 chữ số thập phân
      display.display(); // Cập nhật màn hình OLED
    }

    // In ra tốc độ (nếu có)
    if (gps.speed.isUpdated()) {
      display.setCursor(0, 40);
      display.print("Tốc độ: ");
      display.print(gps.speed.kmph()); // Tốc độ km/h
      display.println(" km/h");
      display.display(); // Cập nhật màn hình OLED
    }
  }
}
