#include <SPI.h>
#include <SD.h>

#define SD_CS 5  // Chân CS cho module SD, bạn có thể thay đổi nếu cần

void setup() {
  Serial.begin(115200);
  
  // Thiết lập giao tiếp VSPI
  SPIClass spi = SPIClass(VSPI);
  spi.begin(18, 19, 23, SD_CS);  // SCK: 18, MISO: 19, MOSI: 23, CS: SD_CS

  // Khởi tạo thẻ SD
  if (!SD.begin(SD_CS, spi)) {
    Serial.println("Khởi tạo thẻ SD thất bại!");
    return;
  }
  Serial.println("Khởi tạo thẻ SD thành công!");

  // Tạo và ghi file test.txt
  File file = SD.open("/test.txt", FILE_WRITE);
  if (file) {
    file.println("Đây là file test ghi vào thẻ SD sử dụng VSPI");
    file.close();
    Serial.println("Ghi dữ liệu thành công!");
  } else {
    Serial.println("Không thể mở file để ghi!");
  }

  // Đọc nội dung file test.txt
  file = SD.open("/test.txt");
  if (file) {
    Serial.println("Nội dung file test.txt:");
    while (file.available()) {
      Serial.write(file.read());
    }
    file.close();
  } else {
    Serial.println("Không thể mở file để đọc!");
  }
}

void loop() {
  // Không làm gì trong loop
}
