#include <TinyGPSPlus.h>
#include <Arduino.h>
#include <HardwareSerial.h>
#include <MPU6050_tockn.h>
#include <Wire.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <esp_now.h>
#include <WiFi.h>
#include <SPI.h>
#include <MFRC522.h>

//Khai báo buzzer
#define BUZZER_PIN 27
#define BUZZER_CHANNEL 0
#define PWM_FREQ 5000
#define PWM_RESOLUTION 8

// Relay công tắc xe
#define RELAY_KEY 12

//---Khai báo RFID---
#define SS_PIN 21
#define RST_PIN 22
#define RELAY_START 14
#define LED_RFID 13
MFRC522 rfid(SS_PIN, RST_PIN);
// UID của các thẻ hợp lệ
byte uid1[] = { 0x03, 0xCD, 0x7A, 0x29 };
byte uid2[] = { 0x93, 0xE7, 0x46, 0x16 };

// Trạng thái đăng nhập của từng thẻ
bool isLoggedIn1 = false;
bool isLoggedIn2 = false;

unsigned long ledInterval = 200;
byte lastLoggedUID[4] = { 0 };  // Lưu UID của thẻ đã đăng nhập

// Khai báo đối tượng TinyGPSPlus cho NEO 6MV2
TinyGPSPlus gps;
HardwareSerial gpsSerial(1);
static const uint32_t GPSBaud = 9600;

//Khai báo UART SIM A7680c
HardwareSerial simSerial(2);  // UART2
#define SIM_TX 32
#define SIM_RX 33
#define LED_GSM 4

//---Khai báo các biến---
const char* alertphoneNumber = "+84942750136";  //SĐT khẩn cấp

//Ngưỡng gia tốc va chạm
const float LIGHT_COLLISION_THRESHOLD = 1.7;
const float MODERATE_COLLISION_THRESHOLD = 2;
const float SEVERE_COLLISION_THRESHOLD = 2.2;

// Cờ trạng thái cảnh báo
bool sleepGucWarningFlag = false;
bool collisionFlag = false;

// Biến đếm
int sleepGucCount = 0;
int collisionCount = 0;

//Khai báo biến trạng thái Nguồn
int analogInPin = 36;   // Analog input pin
int sensorValue;        // Analog Output of Sensor
float calibration = 0;  // Check Battery voltage using multimeter & add/subtract the value
float voltageInput = 0.0;
//Khai báo biến trạng thái pin
int bat_percentage = 0;

//---Khai báo ESP NOW----
uint8_t broadcastAddress[] = { 0xE8, 0x68, 0xE7, 0x24, 0x9D, 0xC0 };  //MAC của điểm đến

//Kiểu dữ liệu gửi/nhận
typedef struct struct_message {
  bool sleepGucWarning;
  char message[32];
  float acceleration;
  char collision_type[20];
  int bat_percentage;
} struct_message;

struct_message myData;

esp_now_peer_info_t peerInfo;

//Khai báo semaphore
SemaphoreHandle_t buzzerMutex;

//Khai báo cảm biến gia tốc
MPU6050 mpu6050(Wire);

//Hàm nhận dữ liệu
void OnDataRecv(const uint8_t* mac, const uint8_t* incomingData, int len) {
  memcpy(&myData, incomingData, sizeof(myData));

  if (myData.sleepGucWarning) {
    if (!sleepGucWarningFlag) {
      sleepGucWarningFlag = true;
      sleepGucCount++;  // Tăng đếm khi cảnh báo ngủ gục xảy ra
      sendSleepSMS();
      // vTaskDelay(2000 / portTICK_PERIOD_MS);
    }
    Serial.println("Cảnh báo: Ngủ gục!");
  } else {
    if (sleepGucWarningFlag) {
      sleepGucWarningFlag = false;
      Serial.println("Ngừng cảnh báo ngủ gục.");
    }
  }
  bat_percentage = myData.bat_percentage;
  // Serial.println(bat_percentage);
}

//---Hàm gửi dữ liệu---
void OnDataSent(const uint8_t* mac_addr, esp_now_send_status_t status) {
  Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Delivery Success" : "Delivery Fail");
  // vTaskDelay(1000 / portTICK_PERIOD_MS);
}

void sendCollisionData(const char* collision_type, float acceleration) {
  static String previousCollisionType = "No Collision";  // Lưu trạng thái va chạm trước đó

  // Chỉ gửi cảnh báo khi trạng thái va chạm thay đổi
  if (previousCollisionType != collision_type) {
    strcpy(myData.collision_type, collision_type);
    strcpy(myData.message, strcmp(collision_type, "No Collision") == 0 ? "No Collision" : "Collision Detected!");
    myData.acceleration = acceleration;
    esp_now_send(broadcastAddress, (uint8_t*)&myData, sizeof(myData));

    if (strcmp(collision_type, "No Collision") != 0) {
      if (!collisionFlag) {
        collisionFlag = true;
        collisionCount++;  // Tăng đếm khi va chạm xảy ra
        sendCollisionSMS();
      }
      Serial.println("Cảnh báo: Va chạm");
    } else {
      collisionFlag = false;
      Serial.println("Ngừng cảnh báo va chạm.");
    }

    // Cập nhật trạng thái va chạm trước đó
    previousCollisionType = collision_type;
  }
}

//Hàm thiết lập ESP NOW
void initializeESPNow() {
  WiFi.mode(WIFI_STA);
  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }
  esp_now_register_send_cb(OnDataSent);
  esp_now_register_recv_cb(OnDataRecv);

  memcpy(peerInfo.peer_addr, broadcastAddress, 6);
  peerInfo.channel = 0;
  peerInfo.encrypt = false;
  esp_now_add_peer(&peerInfo);
}

//Hàm test còi
void testBuzzer() {
  ledcWriteTone(BUZZER_CHANNEL, 1000);
  vTaskDelay(200 / portTICK_PERIOD_MS);
  ledcWriteTone(BUZZER_CHANNEL, 1500);
  vTaskDelay(200 / portTICK_PERIOD_MS);
  ledcWriteTone(BUZZER_CHANNEL, 2000);
  vTaskDelay(200 / portTICK_PERIOD_MS);
  ledcWriteTone(BUZZER_CHANNEL, 2500);
  vTaskDelay(200 / portTICK_PERIOD_MS);
  ledcWriteTone(BUZZER_CHANNEL, 0);
}

// Prototype các hàm
void setupSIM();
String sendATCommand(const char* command, unsigned long timeout = 15000);
void sendSMS(const char* phoneNumber, const char* message);
void listenForMessages(void* pvParameters);
void readGPSTask(void* parameter);
void printGPSData();

void setup() {
  Serial.begin(115200);

  //Buzzer
  pinMode(BUZZER_PIN, OUTPUT);
  ledcSetup(BUZZER_CHANNEL, PWM_FREQ, PWM_RESOLUTION);
  ledcAttachPin(BUZZER_PIN, BUZZER_CHANNEL);
  testBuzzer();

  //Relay công tắc xe
  pinMode(RELAY_KEY, OUTPUT);
  digitalWrite(RELAY_KEY, HIGH);

  //Relay khởi động xe
  pinMode(RELAY_START, OUTPUT);
  pinMode(LED_RFID, OUTPUT);
  pinMode(BUZZER_PIN, OUTPUT);

  buzzerMutex = xSemaphoreCreateMutex();


  //RFID
  SPI.begin();
  rfid.PCD_Init();
  Serial.println("Đang chờ thẻ RFID...");

  setupSIM();  //SIM
  pinMode(LED_GSM, OUTPUT);

  gpsSerial.begin(GPSBaud, SERIAL_8N1, 16, 17);  //GPS

  //Cảm biến va chạm
  Wire.begin(25, 26);
  Serial.println("Đang khởi tạo MPU6050...");
  mpu6050.begin();
  mpu6050.calcGyroOffsets(true);
  Serial.println("MPU6050 đã sẵn sàng!");

  initializeESPNow();  //ESP NOW


  // In tiêu đề
  // Serial.printf("%-15s %-15s %-15s %-10s %-10s\n", "Latitude", "Longitude", "Time", "Satellites", "HDOP");
  // Serial.println(F("-----------------------------------------------------------"));

  xTaskCreate(taskRFID, "RFID Task", 2048, NULL, 1, NULL);
  xTaskCreate(taskLED, "LED Task", 1024, NULL, 1, NULL);
  xTaskCreatePinnedToCore(readGPSTask, "Read GPS Task", 2048, NULL, 1, NULL, 1);
  xTaskCreate(listenForMessages, "Listen for Messages", 8000, NULL, 1, NULL);
  xTaskCreatePinnedToCore(TaskReadSensor, "TaskReadSensor", 2048, NULL, 1, NULL, 0);
  xTaskCreatePinnedToCore(TaskHandleBuzzer, "TaskHandleBuzzer", 2048, NULL, 1, NULL, 1);

  Serial.println(sendATCommand("AT"));
  Serial.println(sendATCommand("AT+CMGD=1,4"));
  Serial.println(sendATCommand("AT+CNMI=2,2,0,0,0"));
}

void loop() {}

// Task đọc và xử lý thẻ RFID
void taskRFID(void* parameter) {
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
void taskLED(void* parameter) {
  for (;;) {
    digitalWrite(LED_RFID, !digitalRead(LED_RFID));
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
    digitalWrite(RELAY_START, LOW);
    ledInterval = 200;
    // beep(1, LONG_BEEP_DURATION);
    playBuzzerTone(1, 1000, 4000);
    memset(lastLoggedUID, 0, 4);  // Xóa UID đã lưu khi đăng xuất
    sendSMS(alertphoneNumber, generateReportMessage().c_str());
    sendSMS(alertphoneNumber, generateAlertMessage().c_str());
  } else if (!*isLoggedIn && (lastLoggedUID[0] == 0 && lastLoggedUID[1] == 0 && lastLoggedUID[2] == 0 && lastLoggedUID[3] == 0)) {
    *isLoggedIn = true;
    Serial.println("Login successful.");
    digitalWrite(RELAY_START, HIGH);
    ledInterval = 1000;
    // beep(3, BEEP_DURATION);
    playBuzzerTone(3, 100, 4000);
    memcpy(lastLoggedUID, currentUID, 4);  // Lưu UID của thẻ khi đăng nhập
    sendSMS(alertphoneNumber, generateReportMessage().c_str());
    sendSMS(alertphoneNumber, generateAlertMessage().c_str());
  } else {
    Serial.println("Access denied. UID does not match the logged-in card.");
  }
}

void buzzerControlTask(void* pvParameters) {
  while (true) {
    if (xSemaphoreTake(buzzerMutex, portMAX_DELAY)) {  // Lấy mutex
      if (isLoggedIn1 || isLoggedIn2) {
        // Âm thanh đăng nhập (3 tiếng bíp ngắn)
        for (int i = 0; i < 3; i++) {
          ledcWriteTone(BUZZER_CHANNEL, 4000);
          vTaskDelay(100 / portTICK_PERIOD_MS);
          ledcWriteTone(BUZZER_CHANNEL, 0);
          vTaskDelay(100 / portTICK_PERIOD_MS);
        }
      } else {
        // Âm thanh đăng xuất (1 tiếng bíp dài)
        ledcWriteTone(BUZZER_CHANNEL, 4000);
        vTaskDelay(1000 / portTICK_PERIOD_MS);
        ledcWriteTone(BUZZER_CHANNEL, 0);
      }

      xSemaphoreGive(buzzerMutex);  // Giải phóng mutex
    }
    vTaskDelay(10 / portTICK_PERIOD_MS);
  }
}


String printLoginStatus() {
  String user = "N/A";

  if (isLoggedIn1) {
    user = "Nguyen Van A";
  } else if (isLoggedIn2) {
    user = "Nguyen Van B";
  }
  return user;
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

//---Cảnh báo va chạm---
//Hàm đọc cảm biến
void TaskReadSensor(void* pvParameters) {
  while (true) {
    mpu6050.update();
    float accelX = mpu6050.getAccX();
    float accelY = mpu6050.getAccY();
    float accelZ = mpu6050.getAccZ();
    float totalAcceleration = sqrt(accelX * accelX + accelY * accelY + accelZ * accelZ);

    if (totalAcceleration >= LIGHT_COLLISION_THRESHOLD && totalAcceleration < MODERATE_COLLISION_THRESHOLD) {
      sendCollisionData("Light Collision", totalAcceleration);
    } else if (totalAcceleration >= MODERATE_COLLISION_THRESHOLD && totalAcceleration < SEVERE_COLLISION_THRESHOLD) {
      sendCollisionData("Moderate Collision", totalAcceleration);
    } else if (totalAcceleration >= SEVERE_COLLISION_THRESHOLD) {
      sendCollisionData("Severe Collision", totalAcceleration);
    } else {
      sendCollisionData("No Collision", totalAcceleration);
    }

    vTaskDelay(50 / portTICK_PERIOD_MS);
  }
}

//Hàm phát âm thanh cảnh báo
void TaskHandleBuzzer(void* pvParameters) {
  while (true) {
    float accelX = mpu6050.getAccX();
    float accelY = mpu6050.getAccY();
    float accelZ = mpu6050.getAccZ();
    float totalAcceleration = sqrt(accelX * accelX + accelY * accelY + accelZ * accelZ);
    if (xSemaphoreTake(buzzerMutex, portMAX_DELAY)) {  // Lấy mutex
      if (totalAcceleration >= LIGHT_COLLISION_THRESHOLD && totalAcceleration < MODERATE_COLLISION_THRESHOLD) {
        playBuzzerTone(5, 100, 1000);
      } else if (totalAcceleration >= MODERATE_COLLISION_THRESHOLD && totalAcceleration < SEVERE_COLLISION_THRESHOLD) {
        playBuzzerTone(5, 100, 1500);
      } else if (totalAcceleration >= SEVERE_COLLISION_THRESHOLD) {
        playBuzzerTone(10, 100, 2000);
      } else {
        ledcWriteTone(BUZZER_CHANNEL, 0);
      }
      xSemaphoreGive(buzzerMutex);  // Giải phóng mutex
    }
    vTaskDelay(200 / portTICK_PERIOD_MS);
  }
}

void playBuzzerTone(int repeats, int duration, int frequency) {
  for (int i = 0; i < repeats; i++) {
    ledcWriteTone(BUZZER_CHANNEL, frequency);
    vTaskDelay(duration / portTICK_PERIOD_MS);
    ledcWriteTone(BUZZER_CHANNEL, 0);
    vTaskDelay(duration / portTICK_PERIOD_MS);
  }
}

//---GPS---
void readGPSTask(void* parameter) {
  static int lastGPSChars = 0;
  static unsigned long lastGPSTime = 0;

  for (;;) {
    while (gpsSerial.available() > 0) gps.encode(gpsSerial.read());

    int currentGPSChars = gps.charsProcessed();

    if (currentGPSChars != lastGPSChars) {
      lastGPSChars = currentGPSChars;
      lastGPSTime = millis();

      printGPSData();
    } else if (millis() - lastGPSTime > 5000) {
      Serial.println("No GPS data received: check wiring");
    }

    vTaskDelay(pdMS_TO_TICKS(1000));
  }
}

void printGPSData() {
  if (gps.location.isValid() && gps.date.isValid() && gps.time.isValid() && gps.satellites.isValid() && gps.hdop.isValid()) {
    int hour = gps.time.hour() + 7;  // Cộng 7 giờ cho múi giờ
    int day = gps.date.day();
    int month = gps.date.month();
    int year = gps.date.year();

    // Kiểm tra nếu giờ vượt quá 24 để tăng ngày
    if (hour >= 24) {
      hour -= 24;  // Trừ đi 24 giờ để giờ quay về khoảng [0, 23]
      day++;       // Tăng thêm 1 ngày

      // Kiểm tra ngày vượt quá số ngày trong tháng
      if ((month == 1 || month == 3 || month == 5 || month == 7 || month == 8 || month == 10 || month == 12) && day > 31) {
        day = 1;
        month++;
      } else if ((month == 4 || month == 6 || month == 9 || month == 11) && day > 30) {
        day = 1;
        month++;
      } else if (month == 2) {  // Xử lý tháng 2 và năm nhuận
        bool isLeapYear = (year % 4 == 0 && year % 100 != 0) || (year % 400 == 0);
        if (day > (isLeapYear ? 29 : 28)) {
          day = 1;
          month++;
        }
      }

      // Kiểm tra nếu tháng vượt quá 12 để tăng năm
      if (month > 12) {
        month = 1;
        year++;
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
    Serial.println("No valid GPS data available.");
  }
}

//---SIM + SMS---
void setupSIM() {
  simSerial.begin(115200, SERIAL_8N1, SIM_RX, SIM_TX);
  vTaskDelay(pdMS_TO_TICKS(100));
}

void sendSMS(const char* phoneNumber, const char* message) {
  // simSerial.println("AT+CFUN=1,1");
  // simSerial.println("AT+RST");
  simSerial.println("AT+CMGF=1");
  vTaskDelay(pdMS_TO_TICKS(100));

  simSerial.print("AT+CMGS=\"");
  simSerial.print(phoneNumber);
  simSerial.println("\"");

  vTaskDelay(pdMS_TO_TICKS(100));

  simSerial.print(message);
  simSerial.write(26);
  vTaskDelay(pdMS_TO_TICKS(2000));

  // Serial.println("Response: " + simSerial.readString());
  vTaskDelay(pdMS_TO_TICKS(3000));
}

void listenForMessages(void* pvParameters) {
  String incomingMessage;
  String senderNumber;

  for (;;) {
    if (simSerial.available()) {
      incomingMessage = simSerial.readString();
      int start = incomingMessage.indexOf("+CMT: \"") + 7;
      int end = incomingMessage.indexOf("\"", start);
      if (start != -1 && end != -1) senderNumber = incomingMessage.substring(start, end);

      Serial.println(incomingMessage);

      if (incomingMessage.indexOf("report") != -1) {
        if (!senderNumber.isEmpty()) {
          sendSMS(senderNumber.c_str(), generateReportMessage().c_str());
          sendSMS(senderNumber.c_str(), generateAlertMessage().c_str());
        }
      } else if (incomingMessage.indexOf("unlock") != -1) {
        digitalWrite(RELAY_KEY, LOW);
        Serial.println("Đã mở khóa xe");
        if (!senderNumber.isEmpty()) sendSMS(senderNumber.c_str(), "Da mo khoa xe");
      } else if (incomingMessage.indexOf("lock") != -1) {
        digitalWrite(RELAY_KEY, HIGH);
        Serial.println("Đã khóa xe");
        if (!senderNumber.isEmpty()) sendSMS(senderNumber.c_str(), "Da khoa xe!");
      }
    }
    vTaskDelay(pdMS_TO_TICKS(200));
  }
}

void sendSleepSMS() {
  if (sleepGucWarningFlag) {
    String message = "!!!PHAT HIEN NGU GUC!!!\n" + generateAlertMessage();
    sendSMS(alertphoneNumber, message.c_str());
    sendSMS(alertphoneNumber, generateReportMessage().c_str());
    // sleepGucWarningFlag = false;  // Đặt lại cờ sau khi gửi tin nhắn
  }
}

void sendCollisionSMS() {
  if (collisionFlag) {
    String message = "!!!PHAT HIEN VA CHAM!!!\n" + generateAlertMessage();
    sendSMS(alertphoneNumber, message.c_str());
    sendSMS(alertphoneNumber, generateReportMessage().c_str());
    // collisionFlag = false;  // Đặt lại cờ sau khi gửi tin nhắn
  }
}

String sendATCommand(const char* command, unsigned long timeout) {
  simSerial.println(command);
  unsigned long startTime = millis();
  while (millis() - startTime < timeout) {
    if (simSerial.available()) {
      return simSerial.readStringUntil('\n');
    }
  }
  return "";
}

String getGSMStatus() {
  simSerial.println("AT+CSQ");
  vTaskDelay(pdMS_TO_TICKS(100));

  if (simSerial.available()) {
    String response = simSerial.readString();
    int signalStrengthIndex = response.indexOf("+CSQ: ");
    if (signalStrengthIndex != -1) {
      String signalStrength = response.substring(signalStrengthIndex + 6, signalStrengthIndex + 8);
      return (signalStrength == "99") ? "Không có tín hiệu GSM" : signalStrength;
    }
  }
  return "No GSM signals.";
}

String generateStoleDetectMessage() {
  String stoleDetectMessage = "!!!TRUY CAP TRAI PHEP!!!\n" + generateReportMessage();
  sendSMS(alertphoneNumber, stoleDetectMessage.c_str());
}

String batteryWarningMessage() {
  String batteryLowMessage = "!!!LOW BATTERY-PLEASE CHARGE!!!\n";
  sendSMS(alertphoneNumber, batteryLowMessage.c_str());
}

String overspeedingMessage() {
  String message = "!!!QUA TOC DO - VUI LONG GIAM TOC!!!\n" + generateAlertMessage();
  sendSMS(alertphoneNumber, message.c_str());
  sendSMS(alertphoneNumber, generateReportMessage().c_str());
}

String generateReportMessage() {
  String reportMessage = "- Status: ";
  if (isLoggedIn1) {
    reportMessage += "Logged In\n";
    reportMessage += "- User: " + printLoginStatus() + "\n";  // Thêm thông tin user
  } else if (isLoggedIn2) {
    reportMessage += "Logged In\n";
    reportMessage += "- User: " + printLoginStatus() + "\n";  // Thêm thông tin user
  } else {
    reportMessage += "Logged Out\n";
    reportMessage += "- User: " + printLoginStatus() + "\n";
  }
  reportMessage += "- Key status : \n";
  if (gps.location.isValid()) {
    reportMessage += "- Location: https://www.google.com/maps?q=" + String(gps.location.lat(), 6) + "," + String(gps.location.lng(), 6) + "\n";
    reportMessage += "- GPS satellites: " + String(gps.satellites.value()) + "\n";
    reportMessage += "- HDOP: " + String(gps.hdop.hdop(), 1) + "\n";
  } else {
    reportMessage += "- No GPS signals!!!\n";
  }
  reportMessage += "- GSM: " + getGSMStatus() + "\n";
  reportMessage += "- Input Voltage: " + String(voltageInput, 2) + "V\n";  // Sử dụng biến toàn cục;
  reportMessage += "- Battery: " + String(bat_percentage) + "%\n";
  reportMessage += "- Connection: \n";
  return reportMessage;
}

String generateAlertMessage() {
  return "- Speed: \n- Drive times: \n- Overspeeding: \n- Sleepy times: " + String(sleepGucCount) + "\n- Impact: " + String(collisionCount);
}

//---Đọc giá trị nguồn đầu vào
void inputVoltageReading() {
  sensorValue = analogRead(analogInPin);
  voltageInput = (((sensorValue * 3.3) / 4096) * 5 + calibration);  //multiply by two as voltage divider network is 100K & 100K Resistor
  Serial.print("\t Output Voltage = ");
  Serial.print(voltageInput);
  vTaskDelay(10000 / portTICK_PERIOD_MS);
}

void batteryRead() {
  Serial.println(bat_percentage);
  vTaskDelay(10000 / portTICK_PERIOD_MS);  // Delay để giảm tần suất đọc
}