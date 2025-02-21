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
HardwareSerial gpsSerial(1);           // Hardware Serial sử dụng UART1 trên ESP32
static const uint32_t GPSBaud = 9600;  // Tốc độ baud cho GPS NEO-6M V2

//Relay công tắc xe
#define RELAY_SWITCH 4
// Biến lưu trạng thái relay
// bool relayState = false;
// Kiểm tra trạng thái relay và cập nhật chân GPIO
// void updateRelay() {
// pinMode(RELAY_PIN, OUTPUT);
// digitalWrite(RELAY_SWITCH, HIGH);
// }

// Prototype các hàm
void setupSIM();
String sendATCommand(const char* command, unsigned long timeout = 15000);
void sendSMS(const char* phoneNumber, const char* message);
void listenForMessages(void* pvParameters);
void readGPSTask(void* parameter);

void setup() {
  Serial.begin(115200);  // Serial để in thông tin ra màn hình
  pinMode(RELAY_SWITCH, OUTPUT);
  digitalWrite(RELAY_SWITCH, HIGH);
  // Khởi tạo UART cho SIM
  setupSIM();

  gpsSerial.begin(GPSBaud, SERIAL_8N1, 16, 17);  // Cấu hình Hardware Serial với RX=16, TX=17

  // In tiêu đề
  Serial.printf("%-15s %-15s %-15s %-10s %-10s\n", "Latitude", "Longitude", "Time", "Satellites", "HDOP");
  Serial.println(F("-----------------------------------------------------------"));

  // Tạo task đọc và hiển thị dữ liệu GPS
  xTaskCreatePinnedToCore(readGPSTask, "Read GPS Task", 2048, NULL, 1, NULL, 1);

  // Tạo task để lắng nghe tin nhắn đến và gửi tin nhắn trực tiếp
  xTaskCreate(listenForMessages, "Listen for Messages", 8000, NULL, 1, NULL);

  // Gửi lệnh AT để kiểm tra kết nối
  String response = sendATCommand("AT");
  Serial.println(response);
}

void loop() {
  // Không cần code trong loop do sử dụng FreeRTOS task
}

void readGPSTask(void* parameter) {
  static int lastCharsProcessed = 0;  // Lưu trữ số ký tự đã xử lý ở lần lặp trước
  static unsigned long lastGPSTime = 0; // Lưu thời gian lần cập nhật GPS cuối cùng

  for (;;) {
    // Cập nhật GPS
    while (gpsSerial.available() > 0) {
      gps.encode(gpsSerial.read());
    }

    int currentCharsProcessed = gps.charsProcessed();

    // Kiểm tra nếu dữ liệu mới được xử lý và không có tín hiệu GPS
    if (currentCharsProcessed != lastCharsProcessed) {
      lastCharsProcessed = currentCharsProcessed;
      lastGPSTime = millis(); // Lưu thời gian cập nhật GPS mới

      if (gps.location.isValid() && gps.date.isValid() && gps.time.isValid() && gps.satellites.isValid() && gps.hdop.isValid()) {
        int hour = (gps.time.hour() + 7) % 24;
        int day = gps.date.day();
        int month = gps.date.month();
        int year = gps.date.year();

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
        Serial.println("No valid GPS data available.");
      }
    } else if (millis() - lastGPSTime > 5000) {
      // Nếu không có dữ liệu GPS mới trong 5 giây
      Serial.println("No GPS data received: check wiring");
    }

    vTaskDelay(1000 / portTICK_PERIOD_MS);  // Delay 1 giây giữa các lần quét
  }
}



void setupSIM() {
  simSerial.begin(115200, SERIAL_8N1, SIM_RX, SIM_TX);  // Khởi tạo UART cho SIM
  vTaskDelay(100 / portTICK_PERIOD_MS);
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
  // Đặt chế độ tin nhắn văn bản
  simSerial.println("AT+CMGF=1");
  vTaskDelay(pdMS_TO_TICKS(100));  // Chờ để nhận phản hồi

  // Gửi lệnh gửi tin nhắn
  simSerial.print("AT+CMGS=\"");
  simSerial.print(phoneNumber);  // Số điện thoại người nhận
  simSerial.println("\"");

  vTaskDelay(pdMS_TO_TICKS(100));  // Chờ để nhận phản hồi

  // Gửi nội dung tin nhắn
  simSerial.print(message);
  vTaskDelay(pdMS_TO_TICKS(100));  // Chờ để nhận phản hồi

  // Ký tự ASCII 26 (Ctrl+Z) để kết thúc tin nhắn
  simSerial.write(26);
  vTaskDelay(pdMS_TO_TICKS(2000));  // Chờ 2 giây trước khi tiếp tục

  // Kiểm tra phản hồi từ module SIM
  String response = simSerial.readString();
  Serial.println("Response: " + response);  // In phản hồi từ module SIM
}

String getGSMStatus() {
  simSerial.println("AT+CSQ");     // Lệnh kiểm tra mức tín hiệu GSM
  vTaskDelay(pdMS_TO_TICKS(100));  // Chờ 100ms để nhận phản hồi từ module SIM

  if (simSerial.available()) {
    String response = simSerial.readString();              // Đọc phản hồi từ module SIM
    int signalStrengthIndex = response.indexOf("+CSQ: ");  // Tìm vị trí của tín hiệu GSM

    if (signalStrengthIndex != -1) {
      String signalStrength = response.substring(signalStrengthIndex + 6, signalStrengthIndex + 8);  // Trích xuất mức tín hiệu
      if (signalStrength == "99") {
        return "Không có tín hiệu GSM";  // Nếu tín hiệu là 99, không có tín hiệu GSM
      } else {
        return signalStrength;  // Trả về mức tín hiệu GSM
      }
    }
  }
  return "Không thể lấy tín hiệu GSM.";  // Nếu không có phản hồi từ module SIM
}



void listenForMessages(void* pvParameters) {
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
        String reportMessage = generateReportMessage();
        String alertMessage = generateAlertMessage();

        if (!senderNumber.isEmpty()) {
          sendSMS(senderNumber.c_str(), reportMessage.c_str());  // Gửi trực tiếp không cần qua hàng đợi
          sendSMS(senderNumber.c_str(), alertMessage.c_str());
        } else {
          Serial.println("Không tìm thấy số điện thoại người gửi.");
        }
      }

      if (incomingMessage.indexOf("unlock") != -1) {
        // relayState = false;
        // updateRelay();
        digitalWrite(RELAY_SWITCH, LOW);
        Serial.println("Đã mở khóa xe");  // In trạng thái relay
        String unlockMessage = "Da mo khoa xe";
        if (!senderNumber.isEmpty()) {
          sendSMS(senderNumber.c_str(), unlockMessage.c_str());  // Gửi trực tiếp không cần qua hàng đợi
        } else {
          Serial.println("Không tìm thấy số điện thoại người gửi.");
        }
      }

      else if (incomingMessage.indexOf("lock") != -1) {
        // relayState = true;
        // updateRelay();
        digitalWrite(RELAY_SWITCH, HIGH);
        Serial.println("Đã khóa xe");  // In trạng thái relay
        String lockMessage = "Da khoa xe!";
        if (!senderNumber.isEmpty()) {
          sendSMS(senderNumber.c_str(), lockMessage.c_str());  // Gửi trực tiếp không cần qua hàng đợi
        } else {
          Serial.println("Không tìm thấy số điện thoại người gửi.");
        }
      }
    }
    vTaskDelay(pdMS_TO_TICKS(200));  // Đợi 200ms trước khi tiếp tục kiểm tra
  }
}

String generateReportMessage() {
  String reportMessage = "- Status: \n";
  reportMessage += "- User: \n";
  reportMessage += "- Key status: \n";
  if (gps.location.isValid()) {
    // Chuẩn bị tin nhắn với tọa độ GPS
    reportMessage += "- Location: https://www.google.com/maps?q=" + String(gps.location.lat(), 6) + "," + String(gps.location.lng(), 6) + "\n";
    reportMessage += "- GPS satellites: " + String(gps.satellites.value()) + "\n";
    reportMessage += "- HDOP: " + String(gps.hdop.hdop(), 1) + "\n";
  } else {
    reportMessage += "- No GPS signals!!!\n";
  }

  String gsmStatus = getGSMStatus();
  reportMessage += "- GSM: " + gsmStatus + "\n";
  return reportMessage;
}

String generateAlertMessage() {
  String alertMessage = "- Speed: \n";
  alertMessage += "- Drive times: \n";
  alertMessage += "- Overspeeding: \n";
  alertMessage += "- Sleepy times: \n";
  alertMessage += "- Impact: \n";
  return alertMessage;
}
