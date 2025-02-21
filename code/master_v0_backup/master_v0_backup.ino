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
#include <SD.h>

//Khai báo buzzer
#define BUZZER_PIN 27
#define BUZZER_CHANNEL 0
#define PWM_FREQ 5000
#define PWM_RESOLUTION 8

// Relay công tắc xe
#define RELAY_KEY 12
#define KEY_INPUT 35
int keyState = 0;
bool vehiclestolen = false;  // Xe bị mất cắp
bool vehiclestatus = false;  // Trạng thái bật/tắt xe
bool alertActive = false;

// Chân CS của module SD
#define LED_SD 2
#define SD_CS 5
unsigned long ledIntervalSD = 200;
File logFile;
// Khai báo handle cho task
TaskHandle_t taskSDHandle;

//---Khai báo RFID---
#define SS_PIN 21
#define RST_PIN 22
#define RELAY_START 14
#define LED_RFID 13
MFRC522 rfid(SS_PIN, RST_PIN);
struct User {
  byte uid[4];                     // UID của thẻ
  String name;                     // Tên người dùng
  bool isLoggedIn;                 // Trạng thái đăng nhập
  int overspeedCount;              // Đếm số lần quá tốc độ
  int sleepGucCount;               // Đếm số lần ngủ gục
  int collisionCount;              // Đếm số lần va chạm
  unsigned long totalDrivingTime;  // Thời gian lái xe (tính bằng giây)
};

// Danh sách người dùng
User users[] = {
  { { 0x03, 0xCD, 0x7A, 0x29 }, "Tran Nguyen Khanh Hoang", false, 0, 0, 0, 0 },
  { { 0x93, 0xE7, 0x46, 0x16 }, "Vo Ngoc Bao Han", false, 0, 0, 0, 0 },
  { { 0x35, 0xEA, 0x96, 0x5C }, "Ngo Nguyen Thanh Truc", false, 0, 0, 0, 0 },
  { { 0x79, 0xD7, 0x77, 0x00 }, "Van Dinh Hung", false, 0, 0, 0, 0 },
  { { 0x9D, 0xD4, 0x36, 0x21 }, "Chill guy", false, 0, 0, 0, 0 }
};

//Trạng thái user
String userName = "N/A";           // Mặc định là chưa đăng nhập
String loginStatus = "Dang xuat";  // Mặc định trạng thái là đăng xuất

bool anyLoggedIn = false;
unsigned long ledIntervalRFID = 200;
byte lastLoggedUID[4] = { 0 };  // Lưu UID của thẻ đã đăng nhập
const int userCount = sizeof(users) / sizeof(users[0]);

// Khai báo đối tượng TinyGPSPlus cho NEO 6MV2
TinyGPSPlus gps;
HardwareSerial gpsSerial(1);
static const uint32_t GPSBaud = 9600;
float currentLat = 0.0;
float currentLon = 0.0;
int currentSatellites = 0;
float currentHDOP = 0.0;
String currentDate = "N/A";  // Lưu ngày
String currentTime = "N/A";  // Lưu giờ

//Khai báo UART SIM A7680c
HardwareSerial simSerial(2);  // UART2
#define SIM_TX 32
#define SIM_RX 33
#define LED_GSM 4
#define RX_BUFFER_SIZE 1024  // Kích thước bộ đệm RX (tùy chỉnh)
int GSMlevel = -1;
unsigned long ledIntervalGSM = 200;
String incomingMessage;
String senderNumber;
bool sendSMSFlag = false;

//---Khai báo các biến---
const char* alertphoneNumber = "+84942750136";  //SĐT khẩn cấp

// Cấu hình cảm biến Hall và các tham số
int sensorValue = 0;
const int hallPin = 39;            // Chân kết nối với cảm biến Hall (chân analog)
const int numMagnets = 9;          // Số nam châm trên bánh xe
const int analogThreshold = 4000;  // Ngưỡng tín hiệu analog từ cảm biến Hall
float wheelRadius = 4;             // Bán kính bánh xe (0.28m)
float magnetRadius = 0.075;        // Bán kính vị trí đặt nam châm từ tâm bánh xe (m)
float speed = 0.0;                 // Vận tốc xe (km/h)
unsigned long pulseInterval = 0;   // Thời gian giữa hai xung (microseconds)
unsigned long maxIdleTime = 2000000;
unsigned long lastPulseTime = 0;  // Thời gian nhận xung trước đó
bool overspeedFlag = false;
int overspeedCount = 0;
unsigned long totalDrivingTime = 0;  // Tổng thời gian lái xe (tính bằng giây)
bool isDriving = false;              // Trạng thái xe đang di chuyển
unsigned long overspeedStartTime = 0;

String drivingTimes = "N/A";

//reset thông số khi qua 0h
bool resetDone = false;

//Ngưỡng gia tốc va chạm
const float LIGHT_COLLISION_THRESHOLD = 2;
const float MODERATE_COLLISION_THRESHOLD = 2.3;
const float SEVERE_COLLISION_THRESHOLD = 2.5;

// Cờ trạng thái cảnh báo
bool sleepGucWarningFlag = false;
bool collisionFlag = false;

// Biến đếm
int sleepGucCount = 0;
int collisionCount = 0;

//Khai báo biến trạng thái Nguồn
int analogInPin = 36;      // Analog input pin
int voltageValue;          // Analog Output of Sensor
float calibration = -2.7;  // Check Battery voltage using multimeter & add/subtract the value
float voltageInput = 0.0;
//Khai báo biến trạng thái pin
int bat_percentage;

//---Khai báo ESP NOW----
uint8_t broadcastAddress[] = { 0x2C, 0xBC, 0xBB, 0x0C, 0x50, 0x7C };   //MAC của điểm đến
uint8_t broadcastAddress1[] = { 0xD8, 0xBC, 0x38, 0xFB, 0xB1, 0x90 };  //MAC của điểm đến
bool ESPNOWsended = false;

//Kiểu dữ liệu gửi/nhận
typedef struct struct_message {
  bool sleepGucWarning;
  char message[32];
  float acceleration;
  char collision_type[20];
  int bat_percent;
  bool overspeedWarning;
} struct_message;

struct_message myData;

esp_now_peer_info_t peerInfo;
esp_now_peer_info_t peerInfo1;

//Khai báo semaphore
SemaphoreHandle_t buzzerMutex;
SemaphoreHandle_t xSerialMutex;
// SemaphoreHandle_t pulseSemaphore;

//Khai báo cảm biến gia tốc
MPU6050 mpu6050(Wire);
float totalAcceleration = 0.0;

//Hàm nhận dữ liệu
void OnDataRecv(const uint8_t* mac, const uint8_t* incomingData, int len) {
  memcpy(&myData, incomingData, sizeof(myData));
  //   if (myData.sleepGucWarning && !sleepGucWarningFlag) {
  //     sleepGucWarningFlag = true;
  //     sleepGucCount++;  // Tăng đếm khi cảnh báo ngủ gục xảy ra
  //     sendSleepSMS();
  // } else if (!myData.sleepGucWarning && sleepGucWarningFlag) {
  //     sleepGucWarningFlag = false;
  //     Serial.println("Ngừng cảnh báo ngủ gục.");
  // }
  if (myData.sleepGucWarning) {
    if (!sleepGucWarningFlag) {
      sleepGucWarningFlag = true;
      sleepGucCount++;  // Tăng đếm khi cảnh báo ngủ gục xảy ra
      appendLog(currentDate, currentTime, userName, loginStatus,
                currentLat, currentLon, currentSatellites, currentHDOP, GSMlevel,
                speed, drivingTimes, voltageInput, bat_percentage, overspeedCount,
                sleepGucCount, collisionCount, "");
      sendSleepSMS();
    }
    Serial.println("Cảnh báo: Ngủ gục!");
    vTaskDelay(1000 / portTICK_PERIOD_MS);
  } else {
    if (sleepGucWarningFlag) {
      sleepGucWarningFlag = false;
      Serial.println("Ngừng cảnh báo ngủ gục.");
    }
  }

  bat_percentage = myData.bat_percent;
}

//---Hàm gửi dữ liệu---
void OnDataSent(const uint8_t* mac_addr, esp_now_send_status_t status) {
  ESPNOWsended = (status == ESP_NOW_SEND_SUCCESS);
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
        appendLog(currentDate, currentTime, userName, loginStatus,
                  currentLat, currentLon, currentSatellites, currentHDOP, GSMlevel,
                  speed, drivingTimes, voltageInput, bat_percentage, overspeedCount,
                  sleepGucCount, collisionCount, "");
        sendCollisionSMS();
        emergencyCall(alertphoneNumber);
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

void sendoverspeedWarning(bool overspeedWarning) {
  myData.overspeedWarning = overspeedWarning;
  esp_now_send(broadcastAddress, (uint8_t*)&myData, sizeof(myData));
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

  memcpy(peerInfo1.peer_addr, broadcastAddress, 6);
  peerInfo1.channel = 0;
  peerInfo1.encrypt = false;
  esp_now_add_peer(&peerInfo1);
}

//Hàm test còi
void testBuzzer() {
  digitalWrite(LED_RFID, HIGH);
  digitalWrite(LED_SD, HIGH);
  digitalWrite(LED_GSM, HIGH);
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
String sendATCommand(const char* command, unsigned long timeout = 5000);
void sendSMS(const char* phoneNumber, const char* message);
void listenForMessages(void* pvParameters);
void readGPSTask(void* parameter);
void printGPSData();

void playBuzzerTone(int repeats, int duration, int frequency) {
  for (int i = 0; i < repeats; i++) {
    ledcWriteTone(BUZZER_CHANNEL, frequency);
    vTaskDelay(duration / portTICK_PERIOD_MS);
    ledcWriteTone(BUZZER_CHANNEL, 0);
    vTaskDelay(duration / portTICK_PERIOD_MS);
  }
}


// Khởi tạo SD card
bool initializeSD() {
  // Tắt CS của RFID để tránh xung đột
  digitalWrite(SS_PIN, HIGH);

  // Bắt đầu giao tiếp với SD
  if (!SD.begin(SD_CS)) {
    Serial.println("Không thể khởi tạo thẻ SD!");
    sendSMS(alertphoneNumber, "!!microSD card error!!");
    return false;
  }
  Serial.println("SD card đã sẵn sàng.");
  return true;
}

bool isSDInserted() {
  // Thử mở file và kiểm tra
  File testFile = SD.open("/test.tmp", FILE_WRITE);
  if (testFile) {
    testFile.close();
    SD.remove("/test.tmp");
    return true;
  }
  return false;
}

void setup() {
  Serial.begin(115200);

  initializeESPNow();  //ESP NOW

  gpsSerial.begin(GPSBaud, SERIAL_8N1, 16, 17);  //GPS

  setupSIM();  //SIM
  xSerialMutex = xSemaphoreCreateMutex();
  pinMode(LED_GSM, OUTPUT);
  // Serial.println(sendATCommand("AT"));
  // Serial.println(sendATCommand("AT+CMGD=1,4"));
  // Serial.println(sendATCommand("AT+CNMI=2,2,0,0,0"));

  //Relay công tắc xe
  pinMode(RELAY_KEY, OUTPUT);
  pinMode(KEY_INPUT, INPUT);
  digitalWrite(RELAY_KEY, LOW);

  //Relay khởi động xe
  pinMode(RELAY_START, OUTPUT);
  digitalWrite(RELAY_START, HIGH);
  pinMode(LED_RFID, OUTPUT);
  pinMode(BUZZER_PIN, OUTPUT);

  // Thiết lập chân cảm biến Hall
  pinMode(hallPin, INPUT);

  // Khởi tạo chân CS cho cả SD và RFID
  pinMode(LED_SD, OUTPUT);
  pinMode(SD_CS, OUTPUT);
  pinMode(SS_PIN, OUTPUT);
  // Đảm bảo tất cả CS ban đầu đều tắt
  digitalWrite(SD_CS, HIGH);
  digitalWrite(SS_PIN, LOW);

  Serial.println("Khởi động SD card...");
  if (initializeSD()) {
    initializeLogFile();
  }

  //Buzzer
  pinMode(BUZZER_PIN, OUTPUT);
  ledcSetup(BUZZER_CHANNEL, PWM_FREQ, PWM_RESOLUTION);
  ledcAttachPin(BUZZER_PIN, BUZZER_CHANNEL);
  testBuzzer();
  buzzerMutex = xSemaphoreCreateMutex();
  if (buzzerMutex == NULL) {
    Serial.println("Không thể tạo semaphore");
    while (true)
      ;
  }

  //RFID
  SPI.begin();
  rfid.PCD_Init();
  Serial.println("Đang chờ thẻ RFID...");

  //Cảm biến va chạm
  Wire.begin(25, 26);
  Serial.println("Đang khởi tạo MPU6050...");
  mpu6050.begin();
  mpu6050.calcGyroOffsets(true);
  Serial.println("MPU6050 đã sẵn sàng!");

  xTaskCreatePinnedToCore(taskSDLogger, "SD Logger", 5000, NULL, 3, &taskSDHandle, 0);
  xTaskCreatePinnedToCore(taskRFID, "RFID Task", 8000, NULL, 4, NULL, 0);
  xTaskCreatePinnedToCore(taskRFIDLED, "LED RFID Task", 1024, NULL, 1, NULL, 0);
  xTaskCreatePinnedToCore(taskGSMLED, "LED GSM Task", 1024, NULL, 1, NULL, 0);
  xTaskCreatePinnedToCore(taskSDLED, "LED SD Task", 1024, NULL, 1, NULL, 0);
  xTaskCreatePinnedToCore(buzzerstoleTask, "buzzerstoleTask", 2048, NULL, 4, NULL, 0);
  xTaskCreatePinnedToCore(buzzerCollisionTask, "buzzerCollisionTask", 2048, NULL, 4, NULL, 0);
  xTaskCreatePinnedToCore(buzzerOverspeedTask, "buzzerOverspeedTask", 2048, NULL, 4, NULL, 0);
  xTaskCreatePinnedToCore(readGPSTask, "Read GPS Task", 4096, NULL, 2, NULL, 0);
  xTaskCreatePinnedToCore(sendOverspeedWarningTask, "OverspeedWarningTask", 4000, NULL, 4, NULL, 0);
  xTaskCreatePinnedToCore(calculateSpeedTask, "Calculate Speed", 8000, NULL, 4, NULL, 0);
  xTaskCreatePinnedToCore(listenForMessages, "Listen for Messages", 6000, NULL, 4, NULL, 1);
  xTaskCreatePinnedToCore(TaskReadSensor, "TaskReadSensor", 6000, NULL, 4, NULL, 1);
  xTaskCreatePinnedToCore(drivingTimerTask, "DrivingTimerTask", 2048, NULL, 2, NULL, 1);
  xTaskCreatePinnedToCore(vehicleStatusTask, "Vehicle Status Task", 4096, NULL, 3, NULL, 1);
  xTaskCreatePinnedToCore(readBatTask, "readBatTask", 2048, NULL, 2, NULL, 1);
  xTaskCreatePinnedToCore(readinputVoltageTask, "readinputVoltageTask", 2048, NULL, 1, NULL, 1);
  xTaskCreatePinnedToCore(resetTask, "ResetTask", 1000, NULL, 1, NULL, 1);
  xTaskCreatePinnedToCore(debugprintTask, "debugprintTask", 2000, NULL, 1, NULL, 1);
  xTaskCreatePinnedToCore(taskCheckGSM, "TaskCheckGSM", 2048, NULL, 1, NULL, 1);
}

void loop() {
  // GSMlevel = getGSMStatus();  // Gọi hàm để kiểm tra tín hiệu GSM
  // vTaskDelay(10000 / portTICK_PERIOD_MS);
}

void debugprintTask(void* pvParameters) {
  while (true) {
    Serial.print("   User: ");
    Serial.print(userName);
    Serial.print("   Key: ");
    Serial.print(vehiclestatus);
    Serial.print("   KeyALG: ");
    Serial.print(keyState);
    Serial.print("   Speed: ");
    Serial.print(speed);
    Serial.print("   SpeedALG: ");
    Serial.print(sensorValue);
    Serial.print("   Driving Time: ");
    Serial.println(drivingTimes);
    Serial.print("   overspeedCount: ");
    Serial.print(overspeedCount);
    Serial.print("   sleepGucCount: ");
    Serial.print(sleepGucCount);
    Serial.print("   collisionCount: ");
    Serial.println(collisionCount);
    Serial.print("   GSM: ");
    Serial.print(GSMlevel);
    Serial.print("   ESPNOW: ");
    Serial.print(ESPNOWsended);
    Serial.print("    Battery: ");
    Serial.print(bat_percentage);
    Serial.print("    Input Voltage = ");
    Serial.println(voltageInput);
    Serial.println(myData.sleepGucWarning);
    vTaskDelay(1000 / portTICK_PERIOD_MS);
  }
}

void resetTask(void* pvParameters) {
  while (true) {
    // bool resetDone = false;
    if (currentTime == "00:00:00" && !resetDone) {
      resetparameter();
      resetDone = true;
      Serial.println("Các biến đếm đã được reset!");
    } else if (currentTime != "00:00:00") {
      resetDone = false;  // Reset cờ để sẵn sàng cho ngày mới
    }
    vTaskDelay(1000 / portTICK_PERIOD_MS);  // Chờ 1 giây để tránh reset lặp lại
  }
  vTaskDelay(500 / portTICK_PERIOD_MS);  // Kiểm tra thường xuyên hơn
}

void resetparameter() {
  collisionCount = 0;
  sleepGucCount = 0;
  overspeedCount = 0;
  totalDrivingTime = 0;
  drivingTimes = "N/A";
}

// Task điều khiển buzzer
void buzzerstoleTask(void* parameter) {
  while (true) {
    if (alertActive) {
      if (xSemaphoreTake(buzzerMutex, portMAX_DELAY)) {
        // Phát tín hiệu cảnh báo

        playBuzzerTone(3, 200, 5000);  // Ví dụ: 3 lần bíp, mỗi lần 200ms, tần số 1000Hz
        xSemaphoreGive(buzzerMutex);
      }
    }
    vTaskDelay(200 / portTICK_PERIOD_MS);  // Chu kỳ kiểm tra task
  }
}

// Task kiểm tra trạng thái xe và phát hiện mất cắp
void vehicleStatusTask(void* parameter) {
  unsigned long alertStartTime = 0;  // Thời gian bắt đầu cảnh báo

  while (true) {
    // Đọc trạng thái chìa khóa
    keyState = analogRead(KEY_INPUT);
    vehiclestatus = (keyState > 2400);  // Chìa khóa bật nếu giá trị > 4000

    // Kiểm tra điều kiện cảnh báo
    if ((!vehiclestatus && !anyLoggedIn && speed > 5.0) || (vehiclestatus && !anyLoggedIn && speed > 5.0)) {
      if (!alertActive) {
        alertStartTime = millis();  // Bắt đầu đếm thời gian cảnh báo
        alertActive = true;
        appendLog(currentDate, currentTime, userName, loginStatus,
                  currentLat, currentLon, currentSatellites, currentHDOP, GSMlevel,
                  speed, drivingTimes, voltageInput, bat_percentage, overspeedCount,
                  sleepGucCount, collisionCount, "xxxx");
        Serial.print("Cảnh báo: Phát hiện khả năng mất cắp xe!    ");
        Serial.println(keyState);
      }
    } else {
      alertActive = false;
    }

    // Kiểm tra thời gian cảnh báo
    if (alertActive && (millis() - alertStartTime > 10000)) {  // Sau 10 giây
      vehiclestolen = true;                                    // Ghi nhận xe bị mất cắp
      alertActive = true;
      digitalWrite(RELAY_KEY, HIGH);
      digitalWrite(RELAY_START, HIGH);
      Serial.print("Xe bị mất cắp!   ");
      Serial.println(keyState);
      generateStoleDetectMessage();
    }
    vTaskDelay(500 / portTICK_PERIOD_MS);  // Chu kỳ kiểm tra task
  }
}

void sendOverspeedWarningTask(void* parameter) {
  bool overspeedSMS = false;  // Cờ theo dõi trạng thái gửi tin nhắn
  while (true) {
    if (speed >= 55) {
      if (!overspeedFlag) {
        overspeedFlag = true;
        overspeedStartTime = millis();
        sendoverspeedWarning(true);  // Hiển thị cảnh báo quá tốc
        overspeedSMS = false;        // Cho phép gửi tin nhắn mới nếu chưa gửi
      }

      // Gửi tin nhắn nếu quá 10 giây và chưa gửi tin nhắn
      if (!overspeedSMS && (millis() - overspeedStartTime >= 10000)) {
        overspeedCount++;
        appendLog(currentDate, currentTime, userName, loginStatus,
                  currentLat, currentLon, currentSatellites, currentHDOP, GSMlevel,
                  speed, drivingTimes, voltageInput, bat_percentage, overspeedCount,
                  sleepGucCount, collisionCount, "");
        sendOverspeedingSMS();  // Gửi SMS cảnh báo quá tốc độ
        Serial.println("Overspeed alert sent.");
        overspeedSMS = true;    // Đặt cờ để tránh gửi lại tin nhắn
      }
    }

    // Reset trạng thái nếu tốc độ giảm xuống mức an toàn
    if (speed < 50) {
      overspeedFlag = false;
      overspeedStartTime = 0;
      sendoverspeedWarning(false);  // Tắt cảnh báo quá tốc
      overspeedSMS = false;         // Reset cờ để cho phép gửi tin nhắn lại
    }

    vTaskDelay(500 / portTICK_PERIOD_MS);  // Kiểm tra trạng thái mỗi 500ms
  }
}



void buzzerOverspeedTask(void* pvParameters) {
  while (true) {
    if (overspeedFlag) {
      if (xSemaphoreTake(buzzerMutex, portMAX_DELAY)) {
        playBuzzerTone(1, 100, 2000);  // Phát cảnh báo quá tốc độ (5 lần bíp, 100ms mỗi lần)
        playBuzzerTone(1, 100, 1700);
        xSemaphoreGive(buzzerMutex);  // Giải phóng semaphore
      }
    }
    // Kiểm tra nếu semaphore được cấp phát
    vTaskDelay(200 / portTICK_PERIOD_MS);  // Chu kỳ kiểm tra buzzer
  }
}

// Task tính toán tốc độ
void calculateSpeedTask(void* parameter) {
  while (true) {
    unsigned long currentTime = micros();  // Lấy thời gian hiện tại (microseconds)
    sensorValue = analogRead(hallPin);

    if (sensorValue < analogThreshold) {            // Phát hiện xung
      pulseInterval = currentTime - lastPulseTime;  // Tính thời gian giữa hai xung
      // Tính tốc độ
      float distancePerPulse = 2 * PI * magnetRadius / numMagnets;   // Quãng đường giữa hai xung (m)
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

// Task quản lý thời gian lái xe
void drivingTimerTask(void* pvParameters) {
  while (true) {
    if (speed > 0) {     // Xe đang di chuyển
      if (!isDriving) {  // Bắt đầu tính thời gian
        isDriving = true;
      }
      totalDrivingTime += 1;  // Tính bằng giây
    } else if (isDriving) {   // Xe dừng lại
      // Cộng dồn thời gian lái trước đó
      isDriving = false;
    }

    // Cập nhật chuỗi thời gian lái xe
    if (totalDrivingTime > 0) {
      int hours = totalDrivingTime / 3600;
      int minutes = (totalDrivingTime % 3600) / 60;
      int seconds = totalDrivingTime % 60;

      char buffer[9];  // Chuỗi định dạng hh:mm:ss
      snprintf(buffer, sizeof(buffer), "%02d:%02d:%02d", hours, minutes, seconds);
      drivingTimes = String(buffer);
    }
    vTaskDelay(1000 / portTICK_PERIOD_MS);  // Cập nhật mỗi giây
  }
}

void initializeLogFile() {
  // Kiểm tra nếu file log.csv chưa tồn tại
  if (!SD.exists("/log.csv")) {
    Serial.println("File log.csv không tồn tại, tạo file mới...");
    File logFile = SD.open("/log.csv", FILE_WRITE);
    if (logFile) {
      // Ghi tiêu đề (header) vào file mới
      logFile.println(
        "Thoi gian,,Nguoi dung,Trang thai,Vi tri,,,,GSM,Van toc,Thoi gian lai xe lien tuc,Dien ap ngo vao,Pin (slave),Canh bao,,,");
      logFile.println(
        "Ngay,Gio,,Dang nhap,Lat,Lon,So ve tinh,HDOP,,,,,,Qua toc do,Ngu guc,Va cham,Mat cap");
      logFile.close();
      Serial.println("Đã tạo file log.csv và ghi tiêu đề.");
    } else {
      Serial.println("Lỗi: Không thể tạo file log.csv!");
    }
  } else {
    Serial.println("File log.csv đã tồn tại. Sẽ ghi tiếp dữ liệu.");
  }
}

void reinitializeSD() {
  SD.end();  // Giải phóng module SD
  delay(100);

  digitalWrite(SD_CS, HIGH);  // Đảm bảo tắt module SD
  delay(100);
  digitalWrite(SD_CS, LOW);  // Bật module SD lại

  if (SD.begin(SD_CS)) {
    Serial.println("Thẻ SD đã được tái khởi động thành công.");
    if (!SD.exists("/log.csv")) {
      Serial.println("File log.csv không tồn tại, tạo file mới...");
      File logFile = SD.open("/log.csv", FILE_WRITE);
      if (logFile) {
        // Ghi tiêu đề (header) vào file mới
        logFile.println(
          "Thoi gian,,Nguoi dung,Trang thai,Vi tri,,,,GSM,Van toc,Thoi gian lai xe lien tuc,Dien ap ngo vao,Pin (slave),Canh bao,,,");
        logFile.println(
          "Ngay,Gio,,Dang nhap,Lat,Lon,So ve tinh,HDOP,,,,,,Qua toc do,Ngu guc,Va cham,Mat cap");
        logFile.close();
        Serial.println("Đã tạo file log.csv và ghi tiêu đề.");
      } else {
        Serial.println("Lỗi: Không thể tạo file log.csv!");
      }
    } else {
      Serial.println("File log.csv đã tồn tại. Sẽ ghi tiếp dữ liệu.");
    }
  } else {
    Serial.println("Không thể tái khởi động thẻ SD.");
  }
}

// Hàm thêm một dòng dữ liệu vào file log.csv
void appendLog(
  String Ngay,
  String Gio,
  String nguoiDungA,
  String trangThai,
  float lat,
  float lon,
  int soVeTinh,
  float hdop,
  int GSM,
  float vanToc,
  String thoiGianLaiXeLienTuc,
  float dienApNgoVao,
  int pinSlave,
  int Quatocdo,
  int Nguguc,
  int Vacham,
  String Matcap) {
  logFile = SD.open("/log.csv", FILE_APPEND);

  if (logFile) {
    logFile.print(Ngay);
    logFile.print(",");
    logFile.print(Gio);
    logFile.print(",");
    logFile.print(nguoiDungA);
    logFile.print(",");
    logFile.print(trangThai);
    logFile.print(",");
    logFile.print(lat, 6);
    logFile.print(",");
    logFile.print(lon, 6);
    logFile.print(",");
    logFile.print(soVeTinh);
    logFile.print(",");
    logFile.print(hdop, 2);
    logFile.print(",");
    logFile.print(GSM);
    logFile.print(",");
    logFile.print(vanToc, 1);
    logFile.print(",");
    logFile.print(thoiGianLaiXeLienTuc);
    logFile.print(",");
    logFile.print(dienApNgoVao, 2);
    logFile.print(",");
    logFile.print(pinSlave);
    logFile.print(",");
    logFile.print(Quatocdo);
    logFile.print(",");
    logFile.print(Nguguc);
    logFile.print(",");
    logFile.print(Vacham);
    logFile.print(",");
    logFile.println(Matcap);
    logFile.close();
    Serial.println("Dữ liệu đã được ghi vào log.csv.");
  } else {
    Serial.println("Không thể ghi vào file log.csv!");
  }
}

// Task ghi dữ liệu log
void taskSDLogger(void* parameter) {
  for (;;) {
    // Kích hoạt SD
    digitalWrite(SS_PIN, HIGH);  // Tắt RFID
    digitalWrite(SD_CS, LOW);    // Bật SD

    if (!isSDInserted()) {
      ledIntervalSD = 200;
      Serial.println("Thẻ SD không được phát hiện. Vui lòng kiểm tra!");
      reinitializeSD();  // Thử khởi động lại nếu cần
    } else {
      ledIntervalSD = 1000;
      for (int i = 0; i < userCount; i++) {
        if (users[i].isLoggedIn) {
          userName = users[i].name;   // Lấy tên người dùng đang đăng nhập
          loginStatus = "Dang nhap";  // Đặt trạng thái là đăng nhập
          break;
        }
      }
      // Ghi dữ liệu giả lập vào file log.csv
      appendLog(
        currentDate, currentTime, userName, loginStatus, currentLat, currentLon, currentSatellites,
        currentHDOP, GSMlevel, speed, drivingTimes, voltageInput, bat_percentage, overspeedCount,
        sleepGucCount, collisionCount, "");
    }
    digitalWrite(SD_CS, HIGH);  // Bật SD
    digitalWrite(SS_PIN, LOW);  // Bật RFID
    vTaskDelay(pdMS_TO_TICKS(5000));
  }
}

void taskSDLED(void* parameter) {
  for (;;) {
    digitalWrite(LED_SD, !digitalRead(LED_SD));
    vTaskDelay(ledIntervalSD / portTICK_PERIOD_MS);
  }
}

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
      int userIndex = findUserIndexByUID(rfid.uid.uidByte);
      if (userIndex != -1) {
        // Kiểm tra vận tốc trước khi thay đổi trạng thái đăng nhập/đăng xuất
        if (speed != 0) {
          Serial.println("Access denied. Speed must be 0 to change login state.");
          rfid.PICC_HaltA();  // Dừng RFID nếu không thay đổi trạng thái
          vTaskDelay(200 / portTICK_PERIOD_MS);
          continue;  // Không xử lý thêm nếu xe đang di chuyển
        }
        // Kiểm tra nếu có người dùng khác đã đăng nhập
        if (anyLoggedIn && memcmp(rfid.uid.uidByte, lastLoggedUID, 4) != 0) {
          Serial.println("Access denied. Another card is already logged in.");
          rfid.PICC_HaltA();  // Dừng RFID nếu có thẻ khác đã đăng nhập
          vTaskDelay(200 / portTICK_PERIOD_MS);
          continue;  // Không xử lý thêm nếu có thẻ khác đã đăng nhập
        }
        // Nếu không có vấn đề gì thì gọi hàm handleLoginState
        handleLoginState(&users[userIndex]);
      } else {
        Serial.println("Access denied. Unknown card.");
      }
      rfid.PICC_HaltA();  // Dừng RFID sau khi xử lý
    }
    vTaskDelay(500 / portTICK_PERIOD_MS);
  }
}

// Task điều khiển nháy LED
void taskRFIDLED(void* parameter) {
  for (;;) {
    digitalWrite(LED_RFID, !digitalRead(LED_RFID));
    vTaskDelay(ledIntervalRFID / portTICK_PERIOD_MS);
  }
}

// Hàm kiểm tra UID có trùng với người dùng
int findUserIndexByUID(byte* scannedUID) {
  for (int i = 0; i < sizeof(users) / sizeof(users[0]); i++) {
    if (memcmp(scannedUID, users[i].uid, 4) == 0) {
      return i;
    }
  }
  return -1;  // Không tìm thấy
}

// Hàm đổi trạng thái đăng nhập/đăng xuất và kích hoạt Buzzer
void handleLoginState(User* user) {
  if (xSemaphoreTake(buzzerMutex, portMAX_DELAY)) {
    if (user->isLoggedIn && memcmp(user->uid, lastLoggedUID, 4) == 0) {
      user->isLoggedIn = false;
      anyLoggedIn = false;
      userName = "N/A";
      loginStatus = "Dang xuat";
      // Lưu giá trị biến đếm khi đăng xuất
      user->overspeedCount = overspeedCount;
      user->sleepGucCount = sleepGucCount;
      user->collisionCount = collisionCount;
      user->totalDrivingTime = totalDrivingTime;
      Serial.print("Logout successful.   ");
      Serial.println(user->name);
      digitalWrite(RELAY_START, HIGH);
      resetparameter();
      ledIntervalRFID = 200;
      playBuzzerTone(1, 1000, 4000);
      memset(lastLoggedUID, 0, 4);  // Xóa UID đã lưu khi đăng xuất
      appendLog(currentDate, currentTime, user->name, "Dang nhap", currentLat, currentLon,
                currentSatellites, currentHDOP, GSMlevel, speed, drivingTimes, voltageInput, bat_percentage,
                overspeedCount, sleepGucCount, collisionCount, "");
      sendSMS(alertphoneNumber, generateReportMessage().c_str());
        vTaskDelay(pdMS_TO_TICKS(2000));
      sendSMS(alertphoneNumber, generateAlertMessage().c_str());
    } else if (!user->isLoggedIn && memcmp(lastLoggedUID, user->uid, 4) != 0) {
      user->isLoggedIn = true;
      anyLoggedIn = true;
      Serial.print("Login successful.   ");
      Serial.println(user->name);
      // Khôi phục giá trị biến đếm khi đăng nhập
      overspeedCount = user->overspeedCount;
      sleepGucCount = user->sleepGucCount;
      collisionCount = user->collisionCount;
      totalDrivingTime = user->totalDrivingTime;
      digitalWrite(RELAY_START, LOW);
      digitalWrite(RELAY_KEY, LOW);
      ledIntervalRFID = 1000;
      playBuzzerTone(3, 100, 4000);
      memcpy(lastLoggedUID, user->uid, 4);  // Lưu UID của thẻ khi đăng nhập
      appendLog(currentDate, currentTime, user->name, "Dang nhap", currentLat, currentLon,
                currentSatellites, currentHDOP, GSMlevel, speed, drivingTimes, voltageInput, bat_percentage,
                overspeedCount, sleepGucCount, collisionCount, "");
      sendSMS(alertphoneNumber, generateReportMessage().c_str());
        vTaskDelay(pdMS_TO_TICKS(2000));
      sendSMS(alertphoneNumber, generateAlertMessage().c_str());
    }
    xSemaphoreGive(buzzerMutex);
  }
}

//---Cảnh báo va chạm---
//Hàm đọc cảm biến
void TaskReadSensor(void* pvParameters) {
  while (true) {
    mpu6050.update();
    float accelX = mpu6050.getAccX();
    float accelY = mpu6050.getAccY();
    float accelZ = mpu6050.getAccZ();
    totalAcceleration = sqrt(accelX * accelX + accelY * accelY + accelZ * accelZ);

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
void buzzerCollisionTask(void* pvParameters) {
  while (true) {
    if (xSemaphoreTake(buzzerMutex, portMAX_DELAY)) {
      if (totalAcceleration >= LIGHT_COLLISION_THRESHOLD && totalAcceleration < MODERATE_COLLISION_THRESHOLD) {
        playBuzzerTone(5, 100, 1000);
      } else if (totalAcceleration >= MODERATE_COLLISION_THRESHOLD && totalAcceleration < SEVERE_COLLISION_THRESHOLD) {
        playBuzzerTone(5, 100, 1500);
      } else if (totalAcceleration >= SEVERE_COLLISION_THRESHOLD) {
        playBuzzerTone(10, 100, 2000);
      }
      xSemaphoreGive(buzzerMutex);
    }
    vTaskDelay(200 / portTICK_PERIOD_MS);
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

    // Lấy dữ liệu từ GPS
    currentLat = gps.location.lat();
    currentLon = gps.location.lng();
    currentSatellites = gps.satellites.value();
    currentHDOP = gps.hdop.hdop();

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

    // Lưu ngày và giờ GPS
    char dateBuffer[15];
    snprintf(dateBuffer, sizeof(dateBuffer), "%04d/%02d/%02d", year, month, day);
    currentDate = String(dateBuffer);

    char timeBuffer[15];
    snprintf(timeBuffer, sizeof(timeBuffer), "%02d:%02d:%02d", hour, gps.time.minute(), gps.time.second());
    currentTime = String(timeBuffer);

    Serial.printf("%-15.6f %-15.6f %-15s %-15s %-05d %-10.1f\n",
                  currentLat, currentLon, currentDate.c_str(), currentTime.c_str(), currentSatellites, currentHDOP);
  } else {
    Serial.println("No valid GPS data available.");
  }
}

//---SIM + SMS---
void setupSIM() {
  simSerial.begin(115200, SERIAL_8N1, SIM_RX, SIM_TX);
  simSerial.setRxBufferSize(RX_BUFFER_SIZE);
  vTaskDelay(pdMS_TO_TICKS(100));
  // Serial.println(sendATCommand("AT+IPREX=115200"));
  Serial.println(sendATCommand("AT"));
  Serial.println(sendATCommand("ATE0"));
  Serial.println(sendATCommand("AT+CNMI=2,2,0,0,0"));
  Serial.println(sendATCommand("AT+CMEE=2"));
  Serial.println(sendATCommand("AT+CMGF=1"));
  Serial.println(sendATCommand("AT+CMGD=1,4"));
  Serial.println(sendATCommand("AT+CREG?"));
  Serial.println(sendATCommand("AT+COPS?"));
  Serial.println(sendATCommand("AT+CPMS=?"));
  Serial.println(sendATCommand("AT+CPMS?"));
  Serial.println(sendATCommand("AT+CPMS=\"ME\",\"ME\",\"ME\""));
  vTaskDelay(pdMS_TO_TICKS(100));
}


void emergencyCall(const char* phoneNumber) {
  Serial.println("Dialing " + String(phoneNumber));
  simSerial.print("ATD");
  simSerial.print(phoneNumber);
  simSerial.println(";");  // Lệnh quay số
  vTaskDelay(pdMS_TO_TICKS(20000));
  simSerial.println("ATH");  // Lệnh kết thúc cuộc gọi
}

void sendSMS(const char* phoneNumber, const char* message) {
  // In ra Serial tin nhắn gửi đi
  Serial.println("================================");
  Serial.print("Sending SMS to: ");
  Serial.println(phoneNumber);
  Serial.println("================================");
  Serial.println(message);
  Serial.println("================================");

  simSerial.println("AT+CMGF=1");
  vTaskDelay(pdMS_TO_TICKS(100));
  // simSerial.println("AT+CSCS=\"GSM\"");
  // vTaskDelay(pdMS_TO_TICKS(100));
  simSerial.print("AT+CMGS=\"");
  simSerial.print(phoneNumber);
  vTaskDelay(pdMS_TO_TICKS(500));
  simSerial.println("\"");
  vTaskDelay(pdMS_TO_TICKS(300));
  simSerial.print(message);
  vTaskDelay(pdMS_TO_TICKS(200));
  simSerial.write(26);
  // vTaskDelay(pdMS_TO_TICKS(2000));
  // simSerial.print("AT+CPMS?");
  vTaskDelay(pdMS_TO_TICKS(1000));
} 

void listenForMessages(void* pvParameters) {
  // String incomingMessage;
  // String senderNumber;
  // bool sendSMSFlag = false;

  for (;;) {
    // Đọc tin nhắn nếu có
    if (simSerial.available()) {
      incomingMessage = simSerial.readString();
      int start = incomingMessage.indexOf("+CMT: \"");
      if (start != -1) {
        start += 7;  // Bỏ qua phần "+CMT: \""
        int end = incomingMessage.indexOf("\"", start);
        if (end != -1) {
          senderNumber = incomingMessage.substring(start, end);
        }
      }
      Serial.println(incomingMessage);  // In tin nhắn để debug
      incomingMessage.toLowerCase();

      // Kiểm tra các lệnh trong tin nhắn
      if (incomingMessage.indexOf("report") != -1) {
        sendSMSFlag = true;
      } else if (incomingMessage.indexOf("unlock") != -1) {
        vehiclestolen = false;
        digitalWrite(RELAY_KEY, LOW);
        Serial.println("Đã mở khóa xe");
        sendSMSFlag = true;
      } else if (incomingMessage.indexOf("lock") != -1) {
        if ((alertActive) || (anyLoggedIn && speed < 5.0) || speed < 5.0) {
          digitalWrite(RELAY_KEY, HIGH);
          digitalWrite(RELAY_START, HIGH);
          Serial.println("Đã khóa xe");
          sendSMSFlag = true;
        }
      }

      // Kiểm tra tín hiệu GSM
      int signalStrengthIndex = incomingMessage.indexOf("+csq: ");
      if (signalStrengthIndex != -1) {
        String signalStrength = incomingMessage.substring(signalStrengthIndex + 6, signalStrengthIndex + 8);
        GSMlevel = signalStrength.toInt();
        if (signalStrength == "99" || GSMlevel == -1) {
          ledIntervalGSM = 200;
        } else {
          ledIntervalGSM = 1000;
        }
      }

      // Nếu cần, gửi tin nhắn
      if (sendSMSFlag && !senderNumber.isEmpty()) {
        if (incomingMessage.indexOf("report") != -1) {
          sendSMS(senderNumber.c_str(), generateReportMessage().c_str());
          sendSMS(senderNumber.c_str(), generateAlertMessage().c_str());
        } else if (incomingMessage.indexOf("unlock") != -1) {
          sendSMS(senderNumber.c_str(), "Vehicle unlocked");
        } else if (incomingMessage.indexOf("lock") != -1) {
          sendSMS(senderNumber.c_str(), "Vehicle locked");
        }
        sendSMSFlag = false;  // Reset cờ gửi SMS
      }
    }

    vTaskDelay(pdMS_TO_TICKS(200));  // Delay giữa các lần đọc tin nhắn
  }
}


void sendSleepSMS() {
  if (sleepGucWarningFlag) {
    String message = "!SLEEP DETECTED!\n" + generateAlertMessage();
    sendSMS(alertphoneNumber, message.c_str());
    sendSMS(alertphoneNumber, generateReportMessage().c_str());
    // sleepGucWarningFlag = false;  // Đặt lại cờ sau khi gửi tin nhắn
  }
}

void sendCollisionSMS() {
  if (collisionFlag) {
    String message = "!IMPACT DETECTED!\n" + generateAlertMessage();
    sendSMS(alertphoneNumber, message.c_str());
    sendSMS(alertphoneNumber, generateReportMessage().c_str());
    // collisionFlag = false;  // Đặt lại cờ sau khi gửi tin nhắn
  }
}

void sendOverspeedingSMS() {
  if (overspeedFlag) {
    String message = "!OVERSPEEDING!\n" + generateAlertMessage();
    sendSMS(alertphoneNumber, message.c_str());
    sendSMS(alertphoneNumber, generateReportMessage().c_str());
  }
}

void generateStoleDetectMessage() {
  if (vehiclestolen) {
    String stoleDetectMessage = "!UNKNOWN ACCESS!\n" + generateReportMessage();
    sendSMS(alertphoneNumber, stoleDetectMessage.c_str());
    sendSMS(alertphoneNumber, generateAlertMessage().c_str());
  }
}

String sendATCommand(const char* command, unsigned long timeout) {
  simSerial.println(command);
  // unsigned long startTime = millis();
  // while (millis() - startTime < timeout) {
  //   if (simSerial.available()) {
  //     return simSerial.readStringUntil('\n');
  //   }
  // }
  return "";
}

void taskCheckGSM(void* parameter) {
  while (true) {
    if (!collisionFlag || !sleepGucWarningFlag || !overspeedFlag || !vehiclestolen || !sendSMSFlag) {
      vTaskDelay(pdMS_TO_TICKS(2000));
      simSerial.println("AT+CSQ");
      vTaskDelay(pdMS_TO_TICKS(10000));
    }
  }
}

void taskGSMLED(void* parameter) {
  for (;;) {
    digitalWrite(LED_GSM, !digitalRead(LED_GSM));
    vTaskDelay(ledIntervalGSM / portTICK_PERIOD_MS);
  }
}



String generateReportMessage() {
  String reportMessage = "- Status: ";
  for (int i = 0; i < userCount; i++) {
    if (users[i].isLoggedIn) {
      // anyLoggedIn = true;
      reportMessage += "LogIn\n";
      reportMessage += "- User: " + users[i].name + "\n";
      break;
    }
  }
  if (!anyLoggedIn) {
    reportMessage += "LogOut\n";
    reportMessage += "- User: N/A\n";
  }
  // if (!vehiclestatus) {
  //   reportMessage += "- Key: OFF\n";
  // } else {
  //   reportMessage += "- Key: ON\n";
  // }
  if (gps.location.isValid()) {
    reportMessage += "- Location: https://www.google.com/maps?q=" + String(gps.location.lat(), 6) + "," + String(gps.location.lng(), 6) + "\n";
    reportMessage += "- Satellites: " + String(gps.satellites.value()) + "\n";
    reportMessage += "- HDOP: " + String(gps.hdop.hdop(), 1) + "\n";
  } else {
    reportMessage += "- GPS: No signals\n";
  }
  reportMessage += "- GSM: " + String(GSMlevel) + "\n";
  return reportMessage;
}

String generateAlertMessage() {
  String alertmessage;
  if (!vehiclestatus) {
    alertmessage += "- Key: OFF\n";
  } else {
    alertmessage += "- Key: ON\n";
  }
  alertmessage += "- Power: " + String(voltageInput, 2) + "V\n";
  alertmessage += "- Battery: " + String(bat_percentage) + "%\n";
  alertmessage += "- Connection: " + String(ESPNOWsended) + "\n";
  alertmessage += "- Speed: " + String(speed, 1) + " km/h\n";
  alertmessage += "- Drive times: " + drivingTimes + "\n";
  alertmessage += "- Overspeed: " + String(overspeedCount) + "\n";
  alertmessage += "- Sleepy: " + String(sleepGucCount) + "\n";
  alertmessage += "- Impact: " + String(collisionCount) + "\n";
  return alertmessage;
}

//---Đọc giá trị nguồn đầu vào
void readinputVoltageTask(void* pvParameters) {
  while (true) {
    voltageValue = analogRead(analogInPin);
    voltageInput = (((voltageValue * 3.3) / 4096) * 5.7 + calibration);  //multiply by two as voltage divider network is 100K & 100K Resistor
    vTaskDelay(10000 / portTICK_PERIOD_MS);
  }
}

void readBatTask(void* pvParameters) {
  static bool lowBatterySent = false;
  while (true) {
    if (ESPNOWsended) {
      // Kiểm tra phần trăm pin và gửi cảnh báo khi cần
      if (bat_percentage < 20 && !lowBatterySent) {
        String batlowMessage = "!LOW BATTERY!\n" + generateAlertMessage();
        sendSMS(alertphoneNumber, batlowMessage.c_str());
        lowBatterySent = true;
      } else if (bat_percentage >= 20 && lowBatterySent) {
        lowBatterySent = false;  // Reset cảnh báo khi pin được sạc
      }
    }
    vTaskDelay(5000 / portTICK_PERIOD_MS);
  }
}