#include <TinyGPSPlus.h>

// Khai báo đối tượng TinyGPSPlus
TinyGPSPlus gps;

// Cấu hình các thông số cho module GPS
HardwareSerial gpsSerial(1);    // Hardware Serial sử dụng UART1 trên ESP32
static const uint32_t GPSBaud = 9600; // Tốc độ baud cho GPS NEO-6M V2

void setup() {
  Serial.begin(115200); // Serial để in thông tin ra màn hình
  gpsSerial.begin(GPSBaud, SERIAL_8N1, 16, 17); // Cấu hình Hardware Serial với RX=16, TX=17

  // In tiêu đề
  Serial.printf("%-15s %-15s %-15s %-10s %-10s\n", "Latitude", "Longitude", "Time", "Satellites", "HDOP");
  Serial.println(F("-----------------------------------------------------------"));

  // Tạo task đọc và hiển thị dữ liệu GPS
  xTaskCreatePinnedToCore(readGPSTask, "Read GPS Task", 2048, NULL, 1, NULL, 1);
}

void loop() {
  // Không cần code trong loop do sử dụng FreeRTOS task
}

// Task đọc và hiển thị thông tin GPS
void readGPSTask(void *parameter) {
  for (;;) {
    // Xử lý và đọc dữ liệu GPS
    while (gpsSerial.available() > 0) {
      gps.encode(gpsSerial.read());
    }

    // In ra thông tin GPS
    if (gps.location.isValid() && gps.date.isValid() && gps.time.isValid() && gps.satellites.isValid() && gps.hdop.isValid()) {
      // Tính toán giờ và ngày chính xác
      int hour = (gps.time.hour() + 7) % 24; // Cộng 7 giờ
      int day = gps.date.day();
      int month = gps.date.month();
      int year = gps.date.year();

      // Kiểm tra nếu giờ đã vượt quá 24, thì cần điều chỉnh ngày
      if (hour < gps.time.hour()) { // Nếu giờ sau khi cộng 7 nhỏ hơn giờ gốc, có thể phải cộng thêm một ngày
        day++;
        // Kiểm tra tháng và năm để điều chỉnh ngày
        if ((month == 2 && day > 28) || (day > 30 && (month == 4 || month == 6 || month == 9 || month == 11)) || (day > 31)) {
          day = 1; // Reset lại ngày
          month++;
          if (month > 12) {
            month = 1;
            year++;
          }
        }
      }

      Serial.printf("%-15.6f %-15.6f %02d/%02d/%-10d %02d:%02d:%-05d %-05d %-10.1f\n",
                    gps.location.lat(), 
                    gps.location.lng(),
                    day,
                    month,
                    year,
                    hour,
                    gps.time.minute(), 
                    gps.time.second(),
                    gps.satellites.value(), 
                    gps.hdop.hdop());
    } else {
      Serial.println("Location: Invalid | Day: Invalid | Time: Invalid | Satellites: Invalid | HDOP: Invalid");
    }

    // Xử lý cảnh báo khi không có dữ liệu GPS
    if (millis() > 5000 && gps.charsProcessed() < 10) {
      Serial.println(F("No GPS data received: check wiring"));
    }

    vTaskDelay(1000 / portTICK_PERIOD_MS); // Dừng 1 giây để chờ dữ liệu GPS cập nhật
  }
}
