#include <SPI.h>
#include <MFRC522.h>

#define SS_PIN 21
#define RST_PIN 22
#define RELAY_PIN 2
#define LED_PIN 14
#define BUZZER_PIN 5
#define BUZZER_CHANNEL 0
#define PWM_FREQ 5000
#define PWM_RESOLUTION 8

MFRC522 rfid(SS_PIN, RST_PIN);

// UID của các thẻ hợp lệ
byte uid1[] = {0x03, 0xCD, 0x7A, 0x29};
byte uid2[] = {0x93, 0xE7, 0x46, 0x16};

// Trạng thái đăng nhập của từng thẻ
bool isLoggedIn1 = false;
bool isLoggedIn2 = false;

unsigned long ledInterval = 200;
byte lastLoggedUID[4] = {0};  // Lưu UID của thẻ đã đăng nhập

void setup() {
  Serial.begin(115200);
  SPI.begin();
  rfid.PCD_Init();

  pinMode(RELAY_PIN, OUTPUT);
  pinMode(LED_PIN, OUTPUT);
  pinMode(BUZZER_PIN, OUTPUT);

  ledcSetup(BUZZER_CHANNEL, PWM_FREQ, PWM_RESOLUTION);
  ledcAttachPin(BUZZER_PIN, BUZZER_CHANNEL);

  Serial.println("Đang chờ thẻ RFID...");

  // Tạo các task cho FreeRTOS
  xTaskCreate(taskRFID, "RFID Task", 2048, NULL, 1, NULL);
  xTaskCreate(taskLED, "LED Task", 1024, NULL, 1, NULL);
}

void loop() {
  // Không sử dụng code trong loop vì mọi thứ đã xử lý trong các task
}

// Task đọc và xử lý thẻ RFID
void taskRFID(void *parameter) {
  for (;;) {
    if (rfid.PICC_IsNewCardPresent() && rfid.PICC_ReadCardSerial()) {
      Serial.print("UID của thẻ: ");
      for (byte i = 0; i < rfid.uid.size; i++) {
        Serial.print(rfid.uid.uidByte[i] < 0x10 ? " 0" : " ");
        Serial.print(rfid.uid.uidByte[i], HEX);
      }
      Serial.println();

      if (checkUID(rfid.uid.uidByte, uid1)) {
        handleLoginState(&isLoggedIn1, uid1);
      } else if (checkUID(rfid.uid.uidByte, uid2)) {
        handleLoginState(&isLoggedIn2, uid2);
      } else {
        Serial.println("Access denied. Unknown card.");
      }

      rfid.PICC_HaltA();
    }
    vTaskDelay(100 / portTICK_PERIOD_MS);
  }
}

// Task điều khiển nháy LED
void taskLED(void *parameter) {
  for (;;) {
    digitalWrite(LED_PIN, !digitalRead(LED_PIN));
    vTaskDelay(ledInterval / portTICK_PERIOD_MS);
  }
}

// Hàm kiểm tra UID có trùng với thẻ hợp lệ không
bool checkUID(byte* scannedUID, byte* validUID) {
  for (byte i = 0; i < 4; i++) {
    if (scannedUID[i] != validUID[i]) return false;
  }
  return true;
}

// Hàm đổi trạng thái đăng nhập/đăng xuất và kích hoạt Buzzer
void handleLoginState(bool* isLoggedIn, byte* currentUID) {
  if (*isLoggedIn && checkUID(currentUID, lastLoggedUID)) {
    *isLoggedIn = false;
    Serial.println("Logout successful.");
    digitalWrite(RELAY_PIN, LOW);
    ledInterval = 200;
    // beep(1, LONG_BEEP_DURATION);
    playBuzzerTone(1, 1000, 4000);
    memset(lastLoggedUID, 0, 4);  // Xóa UID đã lưu khi đăng xuất
  } else if (!*isLoggedIn && (lastLoggedUID[0] == 0 && lastLoggedUID[1] == 0 && lastLoggedUID[2] == 0 && lastLoggedUID[3] == 0)) {
    *isLoggedIn = true;
    Serial.println("Login successful.");
    digitalWrite(RELAY_PIN, HIGH);
    ledInterval = 1000;
    // beep(3, BEEP_DURATION);
    playBuzzerTone(3, 100, 4000);
    memcpy(lastLoggedUID, currentUID, 4);  // Lưu UID của thẻ khi đăng nhập
  } else {
    Serial.println("Access denied. UID does not match the logged-in card.");
  }
}

// Hàm điều khiển Buzzer
// void beep(int times, int duration) {
//   for (int i = 0; i < times; i++) {
//     ledcWriteTone(0, BUZZER_FREQ);
//     vTaskDelay(duration / portTICK_PERIOD_MS);
//     ledcWriteTone(0, 0);
//     vTaskDelay(100 / portTICK_PERIOD_MS);
//   }
// }

void playBuzzerTone(int repeats, int duration, int frequency) {
  for (int i = 0; i < repeats; i++) {
    ledcWriteTone(BUZZER_CHANNEL, frequency);
    vTaskDelay(duration / portTICK_PERIOD_MS);
    ledcWriteTone(BUZZER_CHANNEL, 0);
    vTaskDelay(duration / portTICK_PERIOD_MS);
  }
}