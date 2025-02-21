#include <SPI.h>
#include <MFRC522.h>

#define SS_PIN 21
#define RST_PIN 22
#define RELAY_PIN 17   // Chân điều khiển relay
#define LED_PIN 16     // Chân điều khiển LED
#define BUZZER_PIN 27   // Chân điều khiển Buzzer
#define BUZZER_FREQ 4000 // Tần số Buzzer (Hz)
#define BEEP_DURATION 100  // Thời gian mỗi tiếng bíp (ms)
#define LONG_BEEP_DURATION 1000 // Thời gian bíp dài (ms) giảm xuống còn 1 giây

MFRC522 rfid(SS_PIN, RST_PIN);  // Khởi tạo module RFID với chân SS và RST

// UID của hai thẻ RFID
byte parentUID[] = {0x03, 0xCD, 0x7A, 0x29}; // Thẻ phụ huynh
byte studentUID[] = {0x93, 0xE7, 0x46, 0x16}; // Thẻ học sinh

bool relayState = false;  // Trạng thái hiện tại của relay
bool isLoggedIn = false;  // Trạng thái đăng nhập
unsigned long previousMillis = 0;  // Biến lưu thời gian để điều khiển LED nháy
unsigned long ledInterval = 200;     // Thời gian nháy LED (0.2 giây cho nháy nhanh)

void setup() {
  Serial.begin(9600);
  SPI.begin();
  rfid.PCD_Init();  // Khởi động module RFID

  pinMode(RELAY_PIN, OUTPUT);  // Đặt relay là ngõ ra
  digitalWrite(RELAY_PIN, LOW);  // Ban đầu relay tắt (mức thấp)

  pinMode(LED_PIN, OUTPUT);  // Đặt LED là ngõ ra
  digitalWrite(LED_PIN, LOW);  // Tắt LED ban đầu

  pinMode(BUZZER_PIN, OUTPUT); // Đặt Buzzer là ngõ ra
  digitalWrite(BUZZER_PIN, LOW); // Tắt Buzzer ban đầu

  // Thiết lập Buzzer
  ledcSetup(0, BUZZER_FREQ, 8);  // Kênh 0, tần số đã định nghĩa, độ phân giải 8 bit
  ledcAttachPin(BUZZER_PIN, 0);  // Gắn chân BUZZER_PIN vào kênh 0

  Serial.println("Đang chờ thẻ RFID...");
}

void loop() {
  // Điều khiển LED nháy nhanh/chậm dựa vào trạng thái đăng nhập
  blinkLED();

  // Quét thẻ RFID
  if (!rfid.PICC_IsNewCardPresent() || !rfid.PICC_ReadCardSerial()) {
    return;  // Không có thẻ, tiếp tục vòng lặp
  }

  Serial.println("RFID Tag detected.");

  // In mã UID của thẻ ra Serial Monitor
  Serial.print("UID của thẻ: ");
  for (byte i = 0; i < rfid.uid.size; i++) {
    Serial.print(rfid.uid.uidByte[i] < 0x10 ? " 0" : " ");
    Serial.print(rfid.uid.uidByte[i], HEX);  // In từng byte của UID theo dạng HEX
  }
  Serial.println();

  // Kiểm tra thẻ từ có khớp với parentUID hay studentUID
  if (checkUID(rfid.uid.uidByte, parentUID) || checkUID(rfid.uid.uidByte, studentUID)) {
    toggleLoginState();  // Đổi trạng thái đăng nhập/đăng xuất và relay
  } else {
    Serial.println("Access denied. Unknown card.");
  }

  rfid.PICC_HaltA(); // Ngưng kết nối với thẻ
}

// Hàm kiểm tra UID có trùng với thẻ hợp lệ không
bool checkUID(byte* scannedUID, byte* validUID) {
  for (byte i = 0; i < 4; i++) {
    if (scannedUID[i] != validUID[i]) {
      return false;
    }
  }
  return true;
}

// Hàm đổi trạng thái relay và đăng nhập/đăng xuất
void toggleLoginState() {
  isLoggedIn = !isLoggedIn;  // Đổi trạng thái đăng nhập/đăng xuất

  if (isLoggedIn) {
    Serial.println("Login successful.");
    digitalWrite(RELAY_PIN, HIGH);  // Bật relay khi đăng nhập
    ledInterval = 1000;  // Nháy LED chậm khi đăng nhập (1 giây)
    beep(3, BEEP_DURATION);  // Kêu 3 tiếng bíp khi đăng nhập
  } else {
    Serial.println("Logout successful.");
    digitalWrite(RELAY_PIN, LOW);  // Tắt relay khi đăng xuất
    ledInterval = 200;  // Nháy LED nhanh khi chưa đăng nhập (0.2 giây)
    beep(1, LONG_BEEP_DURATION);  // Kêu 1 tiếng bíp dài khi đăng xuất
    // Không chờ delay sau khi đăng xuất
  }
}

// Hàm nháy LED với thời gian đã định
void blinkLED() {
  unsigned long currentMillis = millis();

  // Kiểm tra nếu đã đến thời điểm để nháy LED
  if (currentMillis - previousMillis >= ledInterval) {
    previousMillis = currentMillis;  // Cập nhật thời điểm hiện tại
    digitalWrite(LED_PIN, !digitalRead(LED_PIN));  // Đổi trạng thái LED (ON/OFF)
  }
}

// Hàm kêu bíp cho Buzzer
void beep(int times, int duration) {
  for (int i = 0; i < times; i++) {
    ledcWriteTone(0, BUZZER_FREQ);  // Phát âm thanh bíp
    delay(duration);                 // Thời gian bíp
    ledcWriteTone(0, 0);             // Tắt âm thanh
    delay(100);                     // Thời gian nghỉ giữa các tiếng bíp
  }
}
