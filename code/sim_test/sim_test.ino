#include <HardwareSerial.h>

HardwareSerial simA7680C(1);  // Tạo đối tượng Serial cho module SIM A7680C

#define SIM_RX 33  // Chân RX của ESP32 kết nối với TXD của SIM A7680C
#define SIM_TX 32  // Chân TX của ESP32 kết nối với RXD của SIM A7680C
#define SIM_PWRKEY 23  // Chân điều khiển bật/tắt module
#define SIM_RST 18  // Chân điều khiển reset module

#define PHONE_NUMBER "+84942750136"  // Số điện thoại nhận tin nhắn

void setup() {
  Serial.begin(115200);  // Serial Monitor để hiển thị đầu ra
  simA7680C.begin(115200, SERIAL_8N1, SIM_RX, SIM_TX);  // Serial cho SIM A7680C

  // Cấu hình chân PWRKEY và RST
  pinMode(SIM_PWRKEY, OUTPUT);
  pinMode(SIM_RST, OUTPUT);

  // Bật module bằng cách kéo PWRKEY xuống
  digitalWrite(SIM_PWRKEY, LOW);  // Kéo PWRKEY xuống để bật module
  delay(1000);  // Giữ trong 1 giây
  digitalWrite(SIM_PWRKEY, HIGH);  // Ngừng kéo PWRKEY
  delay(5000);  // Chờ module khởi động

  // Reset module (nếu cần)
  resetModule();

  Serial.println("Bắt đầu kiểm tra module SIM A7680C...");

  delay(3000);  // Chờ module khởi động
  sendATCommand("AT", "OK", 2000);            // Kiểm tra kết nối AT
  sendATCommand("AT+CPIN?", "+CPIN: READY", 2000);  // Kiểm tra thẻ SIM có sẵn không
  sendATCommand("AT+CSQ", "OK", 2000);        // Kiểm tra cường độ tín hiệu
  sendATCommand("AT+CREG?", "OK", 2000);      // Kiểm tra trạng thái đăng ký mạng
  sendATCommand("AT+CGATT?", "OK", 2000);     // Kiểm tra trạng thái gắn vào mạng GPRS

  // Gửi SMS
  sendSMS("ESP32-SIM7600X From linhkienthuduc.com");
}

void loop() {
  // Không có gì trong loop
}

// Hàm gửi lệnh AT và chờ phản hồi
void sendATCommand(String command, String expectedResponse, uint16_t timeout) {
  simA7680C.println(command);  // Gửi lệnh AT tới module
  Serial.print("Gửi lệnh: ");
  Serial.println(command);

  long int time = millis();
  while ((time + timeout) > millis()) {
    while (simA7680C.available()) {
      String response = simA7680C.readString();  // Đọc phản hồi từ module
      Serial.print("Phản hồi: ");
      Serial.println(response);

      if (response.indexOf(expectedResponse) != -1) {
        Serial.println("Lệnh thành công!\n");
        return;
      }
    }
  }
  Serial.println("Lệnh thất bại hoặc không nhận được phản hồi.\n");
}

// Hàm reset module SIM A7680C bằng chân RST
void resetModule() {
  Serial.println("Đang reset module...");
  digitalWrite(SIM_RST, LOW);  // Kéo RST xuống để reset module
  delay(100);  // Giữ trong 100ms
  digitalWrite(SIM_RST, HIGH);  // Ngừng reset
  delay(5000);  // Chờ module khởi động lại
  Serial.println("Module đã reset xong.");
}

// Hàm gửi SMS
void sendSMS(String message) {
  sendATCommand("AT+CMGF=1", "OK", 2000);  // Đặt chế độ SMS sang Text mode
  String command = "AT+CMGS=\"" + String(PHONE_NUMBER) + "\"";  // Lệnh gửi SMS
  sendATCommand(command, ">", 2000);  // Gửi lệnh đến số điện thoại

  simA7680C.print(message);  // Gửi nội dung tin nhắn
  delay(1000);  // Chờ một chút

  simA7680C.write(0x1A);  // Gửi ký tự kết thúc tin nhắn (CTRL+Z)
  Serial.println("Tin nhắn đã được gửi.");
}
