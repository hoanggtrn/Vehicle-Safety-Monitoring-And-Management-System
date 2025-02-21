#include <SPI.h>
#include <MFRC522.h>
#include <SoftwareSerial.h>
#include <HardwareSerial.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

#define SS_PIN 21
#define RST_PIN 22
#define RELAY_PIN 2   // Chân điều khiển relay
#define LED_PIN 4    // Chân điều khiển LED
#define BUZZER_PIN 5   // Chân điều khiển Buzzer
#define BUZZER_FREQ 4000 // Tần số Buzzer (Hz)
#define BEEP_DURATION 100  // Thời gian mỗi tiếng bíp (ms)
#define LONG_BEEP_DURATION 1000 // Thời gian bíp dài (ms)
#define RX_PIN 16         // Chân nhận dữ liệu từ GPS
#define TX_PIN 17         // Chân truyền dữ liệu tới GPS
#define LED_GPS_PIN 15    // Chân LED báo trạng thái GPS

MFRC522 rfid(SS_PIN, RST_PIN);  // Khởi tạo module RFID với chân SS và RST
HardwareSerial gpsSerial(RX_PIN, TX_PIN); // Khởi tạo kết nối GPS qua Serial

// UID của hai thẻ RFID
byte parentUID[] = {0x03, 0xCD, 0x7A, 0x29}; // Thẻ phụ huynh
byte studentUID[] = {0x93, 0xE7, 0x46, 0x16}; // Thẻ học sinh

bool relayState = false;  // Trạng thái hiện tại của relay
bool isLoggedIn = false;  // Trạng thái đăng nhập

void setup() {
  Serial.begin(115200);  // Đặt tốc độ truyền thành 115200
  SPI.begin();
  rfid.PCD_Init();  // Khởi động module RFID
  gpsSerial.begin(9600); // Khởi động kết nối GPS

  pinMode(RELAY_PIN, OUTPUT);  // Đặt relay là ngõ ra
  digitalWrite(RELAY_PIN, LOW);  // Ban đầu relay tắt (mức thấp)

  pinMode(LED_PIN, OUTPUT);  // Đặt LED là ngõ ra
  digitalWrite(LED_PIN, LOW);  // Tắt LED ban đầu

  pinMode(BUZZER_PIN, OUTPUT); // Đặt Buzzer là ngõ ra
  digitalWrite(BUZZER_PIN, LOW); // Tắt Buzzer ban đầu

  pinMode(LED_GPS_PIN, OUTPUT); // Đặt LED GPS là ngõ ra
  digitalWrite(LED_GPS_PIN, LOW); // Tắt LED GPS ban đầu

  // Thiết lập Buzzer
  ledcSetup(0, BUZZER_FREQ, 8);  // Kênh 0, tần số đã định nghĩa, độ phân giải 8 bit
  ledcAttachPin(BUZZER_PIN, 0);  // Gắn chân BUZZER_PIN vào kênh 0

  // Test buzzer
  testBuzzer();

  // Tạo các tác vụ FreeRTOS
  xTaskCreate(TaskRFID, "TaskRFID", 1000, NULL, 1, NULL);
  xTaskCreate(TaskGPS, "TaskGPS", 1000, NULL, 1, NULL);
  xTaskCreate(TaskLED, "TaskLED", 1000, NULL, 1, NULL);
}


void loop() {
  // Không cần code ở đây, để FreeRTOS quản lý
}

// Tác vụ quét thẻ RFID
void TaskRFID(void *pvParameters) {
  while (true) {
    if (!rfid.PICC_IsNewCardPresent() || !rfid.PICC_ReadCardSerial()) {
      vTaskDelay(100 / portTICK_PERIOD_MS); // Nếu không có thẻ, chờ một chút
      continue;
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
    vTaskDelay(100 / portTICK_PERIOD_MS); // Đợi một chút trước khi quét thẻ tiếp theo
  }
}

// Tác vụ kiểm tra trạng thái GPS và nháy LED GPS
void TaskGPS(void *pvParameters) {
  while (true) {
    if (gpsSerial.available()) {
      // Nếu có tín hiệu GPS, nháy LED với tần số 0.5 giây
      digitalWrite(LED_GPS_PIN, HIGH);
      vTaskDelay(250 / portTICK_PERIOD_MS); // Nháy LED trong 250ms
      digitalWrite(LED_GPS_PIN, LOW);
      vTaskDelay(250 / portTICK_PERIOD_MS);
    } else {
      // Nếu không có tín hiệu GPS, nháy LED với tần số 0.2 giây
      digitalWrite(LED_GPS_PIN, HIGH);
      vTaskDelay(100 / portTICK_PERIOD_MS); // Nháy LED trong 100ms
      digitalWrite(LED_GPS_PIN, LOW);
      vTaskDelay(100 / portTICK_PERIOD_MS);
    }
    vTaskDelay(100 / portTICK_PERIOD_MS); // Đợi một chút trước khi kiểm tra lại
  }
}

// Tác vụ nháy LED với trạng thái đăng nhập
void TaskLED(void *pvParameters) {
  unsigned long ledInterval = 200; // Thời gian nháy LED nhanh (0.2 giây)
  unsigned long previousMillis = 0; // Biến lưu thời gian để điều khiển LED nháy

  while (true) {
    unsigned long currentMillis = millis();

    // Kiểm tra nếu đã đến thời điểm để nháy LED
    if (currentMillis - previousMillis >= ledInterval) {
      previousMillis = currentMillis;  // Cập nhật thời điểm hiện tại
      digitalWrite(LED_PIN, !digitalRead(LED_PIN));  // Đổi trạng thái LED (ON/OFF)
    }

    // Thay đổi tốc độ nháy LED dựa vào trạng thái đăng nhập
    if (isLoggedIn) {
      ledInterval = 1000; // Nháy LED chậm khi đăng nhập (1 giây)
    } else {
      ledInterval = 200;  // Nháy LED nhanh khi chưa đăng nhập (0.2 giây)
    }

    vTaskDelay(50 / portTICK_PERIOD_MS); // Đợi một chút trước khi kiểm tra lại
  }
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
    beep(3, BEEP_DURATION);  // Kêu 3 tiếng bíp khi đăng nhập
  } else {
    Serial.println("Logout successful.");
    digitalWrite(RELAY_PIN, LOW);  // Tắt relay khi đăng xuất
    beep(1, LONG_BEEP_DURATION);  // Kêu 1 tiếng bíp dài khi đăng xuất
  }
}

void beep(int times, int duration) {
  for (int i = 0; i < times; i++) {
    ledcWriteTone(0, BUZZER_FREQ);  // Phát âm thanh bíp
    vTaskDelay(duration / portTICK_PERIOD_MS); // Thời gian bíp
    ledcWriteTone(0, 0);              // Tắt âm thanh
    vTaskDelay(100 / portTICK_PERIOD_MS); // Thời gian nghỉ giữa các tiếng bíp
  }
}

// Hàm kiểm tra buzzer
void testBuzzer() {
  Serial.println("Testing buzzer...");
  for (int i = 0; i < 3; i++) {
    ledcWriteTone(0, BUZZER_FREQ); // Phát âm thanh bíp
    vTaskDelay(500 / portTICK_PERIOD_MS); // Bíp trong 500ms
    ledcWriteTone(0, 0);            // Tắt âm thanh
    vTaskDelay(500 / portTICK_PERIOD_MS); // Đợi 500ms trước bíp tiếp theo
  }
}
