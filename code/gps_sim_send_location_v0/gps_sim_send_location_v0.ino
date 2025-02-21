#include <TinyGPSPlus.h>
#include <Arduino.h>
#include <HardwareSerial.h>

// Khai báo đối tượng TinyGPSPlus
TinyGPSPlus gps;

// Khai báo UART cho module SIM7680C
HardwareSerial simSerial(2);  // UART2
#define SIM_TX 32
#define SIM_RX 33

// Cấu hình các thông số cho module GPS
HardwareSerial gpsSerial(1);    // Hardware Serial sử dụng UART1 trên ESP32
static const uint32_t GPSBaud = 9600; // Tốc độ baud cho GPS NEO-6M V2

// Prototype các hàm
void setupSIM();
String sendATCommand(const char* command, unsigned long timeout = 10000);
void sendSMS(const char* phoneNumber, const char* message);
void listenForMessages(void *pvParameters);
void readGPSTask(void *parameter);

void setup() {
  Serial.begin(115200); // Serial để in thông tin ra màn hình

  // Khởi tạo UART cho SIM
  setupSIM();
    
  gpsSerial.begin(GPSBaud, SERIAL_8N1, 16, 17); // Cấu hình Hardware Serial với RX=16, TX=17

  // In tiêu đề
  Serial.printf("%-15s %-15s %-15s %-10s %-10s\n", "Latitude", "Longitude", "Time", "Satellites", "HDOP");
  Serial.println(F("-----------------------------------------------------------"));

  // Tạo task đọc và hiển thị dữ liệu GPS
  xTaskCreatePinnedToCore(readGPSTask, "Read GPS Task", 2048, NULL, 1, NULL, 1);
  
  // Tạo task để lắng nghe tin nhắn đến và gửi tin nhắn trực tiếp
  xTaskCreate(listenForMessages, "Listen for Messages", 4096, NULL, 1, NULL);

  // Gửi lệnh AT để kiểm tra kết nối
  String response = sendATCommand("AT");
  Serial.println(response);
}

void loop() {
  // Không cần code trong loop do sử dụng FreeRTOS task
}

void readGPSTask(void *parameter) {
  for (;;) {
    while (gpsSerial.available() > 0) {
      gps.encode(gpsSerial.read());
    }

    if (gps.location.isValid() && gps.date.isValid() && gps.time.isValid() && gps.satellites.isValid() && gps.hdop.isValid()) {
      int hour = (gps.time.hour() + 7) % 24;
      int day = gps.date.day();
      int month = gps.date.month();
      int year = gps.date.year();

      if (hour < gps.time.hour()) {
        day++;
        if ((month == 2 && day > 28) || (day > 30 && (month == 4 || month == 6 || month == 9 || month == 11)) || (day > 31)) {
          day = 1;
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

    if (millis() > 5000 && gps.charsProcessed() < 10) {
      Serial.println(F("No GPS data received: check wiring"));
    }

    vTaskDelay(1000 / portTICK_PERIOD_MS);
  }
}

void setupSIM() {
  simSerial.begin(115200, SERIAL_8N1, SIM_RX, SIM_TX);  // Khởi tạo UART cho SIM
  delay(100);
}

String sendATCommand(const char* command, unsigned long timeout) {
  simSerial.println(command);
  unsigned long startTime = millis();
  while (millis() - startTime < timeout) {
    if (simSerial.available()) {
      return simSerial.readString();
    }
  }
  return "";
}

void sendSMS(const char* phoneNumber, const char* message) {
  simSerial.println("AT+CMGF=1");
  // vTaskDelay(pdMS_TO_TICKS(1000));  // Thời gian chờ để nhận phản hồi
  simSerial.print("AT+CMGS=\"");
  simSerial.print(phoneNumber);
  simSerial.println("\"\r");

  vTaskDelay(pdMS_TO_TICKS(800));  // Thời gian chờ để nhận phản hồi

  simSerial.print(message);
  vTaskDelay(pdMS_TO_TICKS(2000));  // Thời gian chờ để nhận phản hồi
  simSerial.write(26);  // Ký tự kết thúc tin nhắn
  Serial.println("Sent: " + String(message)); // In thông tin gửi đi
  vTaskDelay(pdMS_TO_TICKS(2000));  // Thời gian chờ trước khi tiếp tục

  // Kiểm tra phản hồi từ SIM
  simSerial.println("AT+CSQ");
  simSerial.println("AT+CREG?");
  // simSerial.println("AT+CMGD = 1,4");
  String response = simSerial.readString();
  Serial.println("Response: " + response); // In ra phản hồi từ module SIM
}

void listenForMessages(void *pvParameters) {
  String incomingMessage;
  String senderNumber;

  for (;;) {
    if (simSerial.available()) {
      incomingMessage = simSerial.readString();

      // Tìm và trích xuất số điện thoại từ chuỗi +CMT
      int start = incomingMessage.indexOf("+CMT: \"") + 7;
      int end = incomingMessage.indexOf("\"", start);
      if (start != -1 && end != -1) {
        senderNumber = incomingMessage.substring(start, end);
      }

      // In tin nhắn nhận được ra Serial Monitor để kiểm tra
      Serial.println(incomingMessage);

      // Phân tích cú pháp và kiểm tra tin nhắn
      if (incomingMessage.indexOf("report") != -1) {

        if (gps.location.isValid()) {
          // Chuẩn bị tin nhắn với tọa độ GPS
          String gpsMessage = "Current Location:\n";
          gpsMessage += "https://www.google.com/maps?q=" + String(gps.location.lat(), 6) + "," + String(gps.location.lng(), 6) + "\n";
          gpsMessage += "Satellites: " + String(gps.satellites.value()) + "\n";
          gpsMessage += "HDOP: " + String(gps.hdop.hdop(), 1);
        // Nếu tin nhắn là "report", gửi báo cáo trạng thái trực tiếp
        if (!senderNumber.isEmpty()) {
          sendSMS(senderNumber.c_str(),gpsMessage.c_str()); // Gửi trực tiếp không cần qua hàng đợi
        } else {
          Serial.println("Không tìm thấy số điện thoại người gửi.");
        }
      }}
    }
    vTaskDelay(pdMS_TO_TICKS(200)); // Đợi 200ms trước khi tiếp tục kiểm tra
  }
}
