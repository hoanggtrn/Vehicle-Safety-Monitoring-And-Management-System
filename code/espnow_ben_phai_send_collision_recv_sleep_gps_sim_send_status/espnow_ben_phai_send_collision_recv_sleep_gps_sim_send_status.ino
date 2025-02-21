#include <TinyGPSPlus.h>
#include <Arduino.h>
#include <HardwareSerial.h>
#include <MPU6050_tockn.h>
#include <Wire.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <esp_now.h>
#include <WiFi.h>

#define BUZZER_PIN 5
#define BUZZER_CHANNEL 0
#define PWM_FREQ 5000
#define PWM_RESOLUTION 8

const char* alertphoneNumber = "+84942750136";

const float LIGHT_COLLISION_THRESHOLD = 1.7;
const float MODERATE_COLLISION_THRESHOLD = 2;
const float SEVERE_COLLISION_THRESHOLD = 2.2;

uint8_t broadcastAddress[] = { 0xD8, 0xBC, 0x38, 0xFB, 0xB1, 0x90 };

typedef struct struct_message {
  bool sleepGucWarning;
  char message[32];
  float acceleration;
  char collision_type[20];
} struct_message;

struct_message myData;
MPU6050 mpu6050(Wire);
esp_now_peer_info_t peerInfo;

// Cờ trạng thái
bool sleepGucWarningFlag = false;
bool collisionFlag = false;

// Biến đếm
int sleepGucCount = 0;
int collisionCount = 0;

void OnDataRecv(const uint8_t *mac, const uint8_t *incomingData, int len) {
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

  // if (strcmp(myData.collision_type, "No Collision") != 0) {
  //   if (!collisionFlag) {
  //     collisionFlag = true;
  //     collisionCount++;  // Tăng đếm khi xảy ra va chạm
  //   }
  //   Serial.println(myData.message);  // In thông báo va chạm
  // } else {
  //   if (collisionFlag) {
  //     collisionFlag = false;
  //     Serial.println("Ngừng cảnh báo va chạm.");
  //   }
  // }
}

void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  // Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Delivery Success" : "Delivery Fail");
  // vTaskDelay(1000 / portTICK_PERIOD_MS);
}

void playBuzzerTone(int repeats, int duration, int frequency) {
  for (int i = 0; i < repeats; i++) {
    ledcWriteTone(BUZZER_CHANNEL, frequency);
    vTaskDelay(duration / portTICK_PERIOD_MS);
    ledcWriteTone(BUZZER_CHANNEL, 0);
    vTaskDelay(duration / portTICK_PERIOD_MS);
  }
}

void sendCollisionData(const char* collision_type, float acceleration) {
  static String previousCollisionType = "No Collision";  // Lưu trạng thái va chạm trước đó

  // Chỉ gửi cảnh báo khi trạng thái va chạm thay đổi
  if (previousCollisionType != collision_type) {
    strcpy(myData.collision_type, collision_type);
    strcpy(myData.message, strcmp(collision_type, "No Collision") == 0 ? "No Collision" : "Collision Detected!");
    myData.acceleration = acceleration;
    esp_now_send(broadcastAddress, (uint8_t *)&myData, sizeof(myData));
    
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

void TaskReadSensor(void *pvParameters) {
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


void TaskHandleBuzzer(void *pvParameters) {
  while (true) {
    float accelX = mpu6050.getAccX();
    float accelY = mpu6050.getAccY();
    float accelZ = mpu6050.getAccZ();
    float totalAcceleration = sqrt(accelX * accelX + accelY * accelY + accelZ * accelZ);

    if (totalAcceleration >= LIGHT_COLLISION_THRESHOLD && totalAcceleration < MODERATE_COLLISION_THRESHOLD) {
      playBuzzerTone(5, 100, 1000);
    } else if (totalAcceleration >= MODERATE_COLLISION_THRESHOLD && totalAcceleration < SEVERE_COLLISION_THRESHOLD) {
      playBuzzerTone(5, 100, 1500);
    } else if (totalAcceleration >= SEVERE_COLLISION_THRESHOLD) {
      playBuzzerTone(10, 100, 2000);
    } else {
      ledcWriteTone(BUZZER_CHANNEL, 0);
    }

    vTaskDelay(200 / portTICK_PERIOD_MS);
  }
}

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

void testBuzzer() {
  playBuzzerTone(5, 100, 1000);
}

// Khai báo đối tượng TinyGPSPlus và UART cho SIM7680C
TinyGPSPlus gps;
HardwareSerial simSerial(2);  // UART2
#define SIM_TX 32
#define SIM_RX 33

// Cấu hình GPS
HardwareSerial gpsSerial(1);           
static const uint32_t GPSBaud = 9600;  

// Relay công tắc xe
#define RELAY_SWITCH 4

// Prototype các hàm
void setupSIM();
String sendATCommand(const char* command, unsigned long timeout = 15000);
void sendSMS(const char* phoneNumber, const char* message);
void listenForMessages(void* pvParameters);
void readGPSTask(void* parameter);
void printGPSData();

void setup() {
  Serial.begin(115200);  
  pinMode(RELAY_SWITCH, OUTPUT);
  digitalWrite(RELAY_SWITCH, HIGH);
  setupSIM();

  Wire.begin(27, 26);
  pinMode(BUZZER_PIN, OUTPUT);
  ledcSetup(BUZZER_CHANNEL, PWM_FREQ, PWM_RESOLUTION);
  ledcAttachPin(BUZZER_PIN, BUZZER_CHANNEL);
   testBuzzer();
  Serial.println("Đang khởi tạo MPU6050...");
  mpu6050.begin();
  mpu6050.calcGyroOffsets(true);
  Serial.println("MPU6050 đã sẵn sàng!");

  initializeESPNow();
  
  gpsSerial.begin(GPSBaud, SERIAL_8N1, 16, 17);  

  // In tiêu đề
  Serial.printf("%-15s %-15s %-15s %-10s %-10s\n", "Latitude", "Longitude", "Time", "Satellites", "HDOP");
  Serial.println(F("-----------------------------------------------------------"));

  xTaskCreatePinnedToCore(readGPSTask, "Read GPS Task", 2048, NULL, 1, NULL, 1);
  xTaskCreate(listenForMessages, "Listen for Messages", 8000, NULL, 1, NULL);
  xTaskCreatePinnedToCore(TaskReadSensor, "TaskReadSensor", 2048, NULL, 1, NULL, 0);
  xTaskCreatePinnedToCore(TaskHandleBuzzer, "TaskHandleBuzzer", 2048, NULL, 1, NULL, 1);

  Serial.println(sendATCommand("AT"));
  Serial.println(sendATCommand("AT+CMGD=1,4"));

// Serial.println(sendATCommand("  AT&F"));
}

void loop() {}

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
    int hour = (gps.time.hour() + 7) % 24;
    Serial.printf("%-15.6f %-15.6f %02d/%02d/%-10d %02d:%02d:%-05d %-05d %-10.1f\n",
                  gps.location.lat(),
                  gps.location.lng(),
                  gps.date.day(),
                  gps.date.month(),
                  gps.date.year(),
                  hour,
                  gps.time.minute(),
                  gps.time.second(),
                  gps.satellites.value(),
                  gps.hdop.hdop());
  } else {
    Serial.println("No valid GPS data available.");
  }
}

void setupSIM() {
  simSerial.begin(115200, SERIAL_8N1, SIM_RX, SIM_TX);  
  vTaskDelay(pdMS_TO_TICKS(100));
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
  return "Không thể lấy tín hiệu GSM.";
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
        digitalWrite(RELAY_SWITCH, LOW);
        Serial.println("Đã mở khóa xe");
        if (!senderNumber.isEmpty()) sendSMS(senderNumber.c_str(), "Da mo khoa xe");
      } else if (incomingMessage.indexOf("lock") != -1) {
        digitalWrite(RELAY_SWITCH, HIGH);
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
    // sleepGucWarningFlag = false;  // Đặt lại cờ sau khi gửi tin nhắn
  }
}

void sendCollisionSMS() {
  if (collisionFlag) {
    String message = "!!!PHAT HIEN VA CHAM!!!\n" + generateAlertMessage();
    sendSMS(alertphoneNumber, message.c_str());
    // collisionFlag = false;  // Đặt lại cờ sau khi gửi tin nhắn
  }
}

String generateReportMessage() {
  String reportMessage = "- Status: \n- User: \n- Key status: \n";
  if (gps.location.isValid()) {
    reportMessage += "- Location: https://www.google.com/maps?q=" + String(gps.location.lat(), 6) + "," + String(gps.location.lng(), 6) + "\n";
    reportMessage += "- GPS satellites: " + String(gps.satellites.value()) + "\n";
    reportMessage += "- HDOP: " + String(gps.hdop.hdop(), 1) + "\n";
  } else {
    reportMessage += "- No GPS signals!!!\n";
  }
  reportMessage += "- GSM: " + getGSMStatus() + "\n";
  return reportMessage;
}

String generateAlertMessage() {
  return "- Speed: \n- Drive times: \n- Overspeeding: \n- Sleepy times: " + String(sleepGucCount) + "\n- Impact: " + String(collisionCount);
}
