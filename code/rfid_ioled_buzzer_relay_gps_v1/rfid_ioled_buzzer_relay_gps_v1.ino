#include <SPI.h>
#include <MFRC522.h>
#include <SoftwareSerial.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <TinyGPS++.h>
#include <HardwareSerial.h>

#define SS_PIN 21
#define RST_PIN 22
#define RELAY_PIN 2
#define LED_PIN 4
#define BUZZER_PIN 5
#define BUZZER_FREQ 4000
#define BEEP_DURATION 100
#define LONG_BEEP_DURATION 1000
#define RX_PIN 16
#define TX_PIN 17
#define LED_GPS_PIN 15

MFRC522 rfid(SS_PIN, RST_PIN);
TinyGPSPlus gps;
HardwareSerial gpsSerial(1);

const byte parentUID[] PROGMEM = {0x03, 0xCD, 0x7A, 0x29};
const byte studentUID[] PROGMEM = {0x93, 0xE7, 0x46, 0x16};

bool relayState = false;
bool isLoggedIn = false;

void setup() {
  Serial.begin(115200);
  SPI.begin();
  rfid.PCD_Init();
  gpsSerial.begin(9600, SERIAL_8N1, RX_PIN, TX_PIN);

  pinMode(RELAY_PIN, OUTPUT);
  pinMode(LED_PIN, OUTPUT);
  pinMode(BUZZER_PIN, OUTPUT);
  pinMode(LED_GPS_PIN, OUTPUT);

  digitalWrite(RELAY_PIN, LOW);
  digitalWrite(LED_PIN, LOW);
  digitalWrite(BUZZER_PIN, LOW);
  digitalWrite(LED_GPS_PIN, LOW);

  ledcSetup(0, BUZZER_FREQ, 8);
  ledcAttachPin(BUZZER_PIN, 0);

  testBuzzer();

  xTaskCreate(TaskRFID, "TaskRFID", 2048, NULL, 1, NULL);
  xTaskCreate(TaskGPS, "TaskGPS", 1024, NULL, 1, NULL);
  xTaskCreate(TaskLED, "TaskLED", 1024, NULL, 1, NULL);
}

void loop() {}

void TaskGPS(void *pvParameters) {
  for (;;) {
    while (gpsSerial.available() > 0) {
      gps.encode(gpsSerial.read());
    }

    if (gps.location.isUpdated()) {
      Serial.print(F("Vĩ độ: ")); Serial.println(gps.location.lat(), 6);
      Serial.print(F("Kinh độ: ")); Serial.println(gps.location.lng(), 6);
      Serial.print(F("Độ cao: ")); Serial.println(gps.altitude.meters());
      Serial.print(F("Số vệ tinh: ")); Serial.println(gps.satellites.value());
      Serial.print(F("Tốc độ: ")); Serial.println(gps.speed.kmph());
      Serial.println();

      digitalWrite(LED_GPS_PIN, HIGH);
      vTaskDelay(250 / portTICK_PERIOD_MS);
      digitalWrite(LED_GPS_PIN, LOW);
      vTaskDelay(250 / portTICK_PERIOD_MS);
    } else {
      Serial.println(F("Không có dữ liệu GPS."));
      for (int i = 0; i < 3; i++) {
        digitalWrite(LED_GPS_PIN, HIGH);
        vTaskDelay(100 / portTICK_PERIOD_MS);
        digitalWrite(LED_GPS_PIN, LOW);
        vTaskDelay(100 / portTICK_PERIOD_MS);
      }
    }

    vTaskDelay(1000 / portTICK_PERIOD_MS);
  }
}

void TaskRFID(void *pvParameters) {
  for (;;) {
    if (!rfid.PICC_IsNewCardPresent() || !rfid.PICC_ReadCardSerial()) {
      vTaskDelay(100 / portTICK_PERIOD_MS);
      continue;
    }

    Serial.println(F("RFID Tag detected."));
    Serial.print(F("UID của thẻ: "));
    for (byte i = 0; i < rfid.uid.size; i++) {
      Serial.print(rfid.uid.uidByte[i] < 0x10 ? " 0" : " ");
      Serial.print(rfid.uid.uidByte[i], HEX);
    }
    Serial.println();

    if (checkUID(rfid.uid.uidByte, parentUID) || checkUID(rfid.uid.uidByte, studentUID)) {
      toggleLoginState();
    } else {
      Serial.println(F("Access denied. Unknown card."));
    }

    rfid.PICC_HaltA();
    vTaskDelay(100 / portTICK_PERIOD_MS);
  }
}

void TaskLED(void *pvParameters) {
  unsigned long ledInterval = 200;
  unsigned long previousMillis = 0;

  for (;;) {
    unsigned long currentMillis = millis();
    if (currentMillis - previousMillis >= ledInterval) {
      previousMillis = currentMillis;
      digitalWrite(LED_PIN, !digitalRead(LED_PIN));
    }
    ledInterval = isLoggedIn ? 1000 : 200;
    vTaskDelay(50 / portTICK_PERIOD_MS);
  }
}

bool checkUID(byte* scannedUID, const byte* validUID) {
  for (byte i = 0; i < 4; i++) {
    if (scannedUID[i] != pgm_read_byte_near(validUID + i)) {
      return false;
    }
  }
  return true;
}

void toggleLoginState() {
  isLoggedIn = !isLoggedIn;
  digitalWrite(RELAY_PIN, isLoggedIn ? HIGH : LOW);
  Serial.println(isLoggedIn ? F("Login successful.") : F("Logout successful."));
  beep(isLoggedIn ? 3 : 1, isLoggedIn ? BEEP_DURATION : LONG_BEEP_DURATION);
}

void beep(int times, int duration) {
  for (int i = 0; i < times; i++) {
    ledcWrite(0, 128);
    vTaskDelay(duration / portTICK_PERIOD_MS);
    ledcWrite(0, 0);
    vTaskDelay(50 / portTICK_PERIOD_MS);
  }
}

void testBuzzer() {
  Serial.println(F("Testing buzzer..."));
  beep(5, 100);
  Serial.println(F("Buzzer test completed."));
}
