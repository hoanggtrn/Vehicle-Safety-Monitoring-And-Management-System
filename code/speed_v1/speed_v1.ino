#include <U8g2lib.h>

// Khởi tạo màn hình OLED (SSD1306, giao tiếp I2C, địa chỉ mặc định 0x3C)
U8G2_SSD1306_128X64_NONAME_F_HW_I2C u8g2(U8G2_R0, /* reset=*/U8X8_PIN_NONE);

// Định nghĩa chân cảm biến Hall
#define hallPin 39  // Chân analog của cảm biến Hall

int analogThreshold = 4000;  // Ngưỡng phát hiện xung

// Các thông số cảm biến
float magnetRadius = 0.075;       // Bán kính vị trí nam châm (m)
float wheelRadius = 0.28;         // Bán kính bánh xe (m)
unsigned long lastPulseTime = 0;  // Thời gian xung cuối cùng (microseconds)
unsigned long pulseInterval = 0;  // Thời gian giữa hai xung (microseconds)
float speed = 0.0;                // Vận tốc (km/h)

unsigned long maxIdleTime = 2000000;     // Thời gian tối đa không có xung (5 giây) trước khi đặt tốc độ về 0

void setup() {
  Serial.begin(115200);

  // Khởi tạo cảm biến Hall
  pinMode(hallPin, INPUT);

  // Khởi tạo màn hình OLED
  u8g2.begin();

  // Tạo các task
  xTaskCreatePinnedToCore(calculateSpeedTask, "Calculate Speed", 8000, NULL, 1, NULL, 0);
  xTaskCreatePinnedToCore(displaySpeedTask, "Display Speed", 8000, NULL, 1, NULL, 1);
}

void loop() {
  // Không dùng loop, toàn bộ xử lý trong các task
}

void calculateSpeedTask(void* parameter) {
  while (true) {
    unsigned long currentTime = micros();  // Lấy thời gian hiện tại (microseconds)
    int sensorValue = analogRead(hallPin);

    if (sensorValue < analogThreshold) {                     // Phát hiện xung
        pulseInterval = currentTime - lastPulseTime;         // Tính thời gian giữa hai xung
        // Tính tốc độ
        float distancePerPulse = 2 * PI * magnetRadius / 9;            // Quãng đường giữa hai xung (m)
        float magnetSpeed = distancePerPulse / (pulseInterval / 1e6);  // Tốc độ tại nam châm (m/s)
        speed = magnetSpeed * (wheelRadius / magnetRadius) * 3.6;      // Chuyển đổi sang km/h
        lastPulseTime = currentTime;                                   // Cập nhật thời gian xung cuối
        while (analogRead(hallPin) < analogThreshold)
          ;
    }

    // Kiểm tra nếu không có xung trong thời gian dài
    if (currentTime - lastPulseTime > maxIdleTime) {
      speed = 0.0;  // Đặt tốc độ về 0 nếu không có xung trong `maxIdleTime` microseconds
    }
    vTaskDelay(5 / portTICK_PERIOD_MS);  // Chờ giảm tải CPU
  }
}

void displaySpeedTask(void* parameter) {
  while (true) {
    u8g2.clearBuffer();  // Xóa bộ nhớ đệm

    // Hiển thị tiêu đề
    u8g2.setFont(u8g2_font_6x10_tr);  // Chọn font chữ
    u8g2.setCursor(0, 10);
    u8g2.print(F("Speed Test (km/h):"));

    // Vẽ đường kẻ ngang
    u8g2.drawLine(0, 12, 128, 12);

    // Hiển thị tốc độ
    u8g2.setCursor(0, 20);
    u8g2.print(F("Speed: "));
    u8g2.print(speed, 2);  // Hiển thị tốc độ với 2 chữ số thập phân

    u8g2.sendBuffer();  // Gửi dữ liệu đến màn hình

    vTaskDelay(100 / portTICK_PERIOD_MS);  // Chờ giảm tải CPU
  }
}
