#include <MPU6050_tockn.h>
#include <Wire.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <esp_now.h>
#include <WiFi.h>
#include <SimpleKalmanFilter.h>

//Khai báo buzzer
#define BUZZER_PIN 27
#define BUZZER_CHANNEL 0
#define PWM_FREQ 5000
#define PWM_RESOLUTION 8
#define LED_PIN 12  // Chân LED báo kết nối ESP-NOW
#define ALERT_LED_PIN 14

//Khai báo ngưỡng phát hiện ngủ gục
const int alertAngleThreshold = 15;
const unsigned long sleepGucDuration = 3000;
bool isHeadNghieng = false;
unsigned long startNghiengTime = 0;

//Khai báo biến trạng thái Nguồn
int analogInPin = 36;   // Analog input pin
int sensorValue;        // Analog Output of Sensor
float calibration = 0;  // Check Battery voltage using multimeter & add/subtract the value
float voltage = 0.0;
int bat_percentage = 20;

//Khai báo cảnh báo quá tốc độ
bool overspeedFlag = false;
unsigned long overspeedStartTime = 0;

//Khai báo cảm biến gia tốc
MPU6050 mpu(Wire);
// Cấu hình bộ lọc Kalman cho X và Y
SimpleKalmanFilter kalmanFilterX(2, 2, 0.01);
SimpleKalmanFilter kalmanFilterY(2, 2, 0.01);

//---Khai báo ESP NOW----
uint8_t broadcastAddress[] = { 0xD8, 0xBC, 0x38, 0xFB, 0xB1, 0x90 };  // MAC ESP32 receiver

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

//Khai báo semaphore
SemaphoreHandle_t buzzerMutex;

//Hàm gửi dữ liệu
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  // Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Delivery Success" : "Delivery Fail");
  // digitalWrite(LED_PIN, status == ESP_NOW_SEND_SUCCESS ? HIGH : LOW);  // Bật LED nếu gửi thành công
}

//---Hàm nhận dữ liệu---
void OnDataRecv(const uint8_t *mac, const uint8_t *incomingData, int len) {
  memcpy(&myData, incomingData, sizeof(myData));
  // Serial.print("Received message: ");
  // Serial.println(myData.message);
  // Serial.print("Acceleration: ");
  // Serial.println(myData.acceleration);
  // Serial.print("Collision Type: ");
  // Serial.println(myData.collision_type);
  digitalWrite(LED_PIN, HIGH);  // Bật LED khi nhận dữ liệu

  myData.overspeedWarning;
}

void sendSleepGucWarning(bool warning) {
  myData.sleepGucWarning = warning;
  esp_now_send(broadcastAddress, (uint8_t *)&myData, sizeof(myData));
}
void sendBatteryInfo(int bat_percent) {
  myData.bat_percent = bat_percent;
  esp_now_send(broadcastAddress, (uint8_t *)&myData, sizeof(myData));
  vTaskDelay(4000 / portTICK_PERIOD_MS);  // Delay để giảm tần suất đọc
}

void initializeESPNow() {
  WiFi.mode(WIFI_STA);
  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }
  esp_now_register_send_cb(OnDataSent);
  memcpy(peerInfo.peer_addr, broadcastAddress, 6);
  peerInfo.channel = 0;
  peerInfo.encrypt = false;
  esp_now_add_peer(&peerInfo);
  esp_now_register_recv_cb(OnDataRecv);
}

void setup() {
  Serial.begin(115200);

  initializeESPNow();

  //Buzzer
  pinMode(ALERT_LED_PIN, OUTPUT);
  pinMode(BUZZER_PIN, OUTPUT);
  ledcSetup(BUZZER_CHANNEL, PWM_FREQ, PWM_RESOLUTION);
  ledcAttachPin(BUZZER_PIN, BUZZER_CHANNEL);
  ledcAttachPin(ALERT_LED_PIN, BUZZER_CHANNEL);
  testBuzzer();

  //Led báo trạng thái ESP NOW
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);  // Tắt LED lúc đầu


  buzzerMutex = xSemaphoreCreateMutex();

  //Cảm biến ngủ gục
  Wire.begin(32, 33);
  mpu.begin();
  mpu.calcGyroOffsets(true);
  Serial.println("Bắt đầu theo dõi trạng thái ngủ gục...");

  xTaskCreatePinnedToCore(checkNghiengTask, "Check Nghieng", 8000, NULL, 4, NULL, 0);
  xTaskCreatePinnedToCore(buzzerControlTask, "Buzzer Control", 5000, NULL, 3, NULL, 1);
  xTaskCreate(TaskHandleBuzzer, "TaskHandleBuzzer", 5000, NULL, 3, NULL);
  xTaskCreatePinnedToCore(readBatteryTask, "Read Battery", 5000, NULL, 1, NULL, 1);  // Task mới cho pin
  xTaskCreate(buzzerOverspeedTask, "buzzerOverspeedTask", 5000, NULL, 3, NULL);
  xTaskCreate(buzzerBatTask, "buzzerBatTask", 5000, NULL, 3, NULL);
}

void loop() {}

void checkNghiengTask(void *pvParameters) {
  while (true) {
    mpu.update();
    float angleX = kalmanFilterX.updateEstimate(mpu.getAngleX());
    float angleY = kalmanFilterY.updateEstimate(mpu.getAngleY());
    // In giá trị góc ra Serial Monitor
    Serial.print("Góc X: ");
    Serial.print(angleX);
    Serial.print(" | Góc Y: ");
    Serial.print(angleY);
    Serial.print("   Received message: ");
    Serial.print(myData.message);
    Serial.print("    Acceleration: ");
    Serial.print(myData.acceleration);
    Serial.print("    Collision Type: ");
    Serial.print(myData.collision_type);
    Serial.print("    Ovs: ");
    Serial.print(myData.overspeedWarning);
    Serial.print("    Sleep: ");
    Serial.print(myData.sleepGucWarning);
    Serial.print("    Battery: ");
    Serial.print(bat_percentage);
    Serial.print("    Voltage: ");
    Serial.print(voltage);
    Serial.print("    GPIOVP: ");
    Serial.println(sensorValue);

    if (abs(angleX) > alertAngleThreshold || abs(angleY) > alertAngleThreshold) {
      if (!isHeadNghieng) {
        startNghiengTime = millis();
        isHeadNghieng = true;
      }

      if (millis() - startNghiengTime >= sleepGucDuration) {
        sendSleepGucWarning(true);
      }
    } else {
      isHeadNghieng = false;
      sendSleepGucWarning(false);
    }

    vTaskDelay(50 / portTICK_PERIOD_MS);
  }
}

void buzzerControlTask(void *pvParameters) {
  while (true) {
    if (xSemaphoreTake(buzzerMutex, portMAX_DELAY)) {  // Lấy mutex
      if (isHeadNghieng && (millis() - startNghiengTime >= sleepGucDuration)) {
        for (int i = 0; i < 3; i++) {
          ledcWriteTone(BUZZER_CHANNEL, 1500);
          vTaskDelay(100 / portTICK_PERIOD_MS);
          ledcWriteTone(BUZZER_CHANNEL, 0);
          vTaskDelay(50 / portTICK_PERIOD_MS);
        }
      } else {
        // ledcWriteTone(BUZZER_CHANNEL, 0);
      }
      xSemaphoreGive(buzzerMutex);  // Giải phóng mutex
    }
    vTaskDelay(100 / portTICK_PERIOD_MS);
  }
}

// Function to control buzzer based on collision type
void playBuzzerTone(int repeatCount, int toneFrequency, int beepDelay, int pauseDelay) {
  for (int i = 0; i < repeatCount; i++) {
    ledcWriteTone(BUZZER_CHANNEL, toneFrequency);
    vTaskDelay(beepDelay / portTICK_PERIOD_MS);
    ledcWriteTone(BUZZER_CHANNEL, 0);
    vTaskDelay(beepDelay / portTICK_PERIOD_MS);
  }
  vTaskDelay(pauseDelay / portTICK_PERIOD_MS);
}

// Task to handle received data and control buzzer
void TaskHandleBuzzer(void *pvParameters) {
  while (true) {
    if (xSemaphoreTake(buzzerMutex, portMAX_DELAY)) {  // Lấy mutex
      if (myData.acceleration > 0) {
        if (strcmp(myData.collision_type, "Light Collision") == 0) {
          playBuzzerTone(5, 1000, 100, 200);  // 5 beeps, 100ms delay
        } else if (strcmp(myData.collision_type, "Moderate Collision") == 0) {
          playBuzzerTone(5, 1500, 100, 200);  // 5 beeps, 100ms delay
        } else if (strcmp(myData.collision_type, "Severe Collision") == 0) {
          playBuzzerTone(10, 2000, 100, 200);  // Continuous beep
        }
      } else if (strcmp(myData.collision_type, "No Collision") == 0) {
        // ledcWriteTone(BUZZER_CHANNEL, 0);  // Stop buzzer if no collision
      }
      xSemaphoreGive(buzzerMutex);  // Giải phóng mutex
    }
    vTaskDelay(200 / portTICK_PERIOD_MS);  // Polling interval
  }
}

void buzzerOverspeedTask(void *pvParameters) {
  while (true) {

    if (myData.overspeedWarning) {
      // overspeedStartTime = millis();
      if (xSemaphoreTake(buzzerMutex, portMAX_DELAY)) {  // Lấy mutex
                                                         // if (millis() - overspeedStartTime <= 10000) {
        playBuzzerTone(5, 2100, 100, 200);               // 5 beeps, 100ms delay
        // }
        xSemaphoreGive(buzzerMutex);
      }
    }

    vTaskDelay(100 / portTICK_PERIOD_MS);  // Kiểm tra trạng thái mỗi giây
  }
}

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

void readBatteryTask(void *pvParameters) {
  while (true) {
    sensorValue = analogRead(analogInPin);
    voltage = (((sensorValue * 3.3) / 4096) * 2.65 - 0.22);
    bat_percentage = mapfloat(voltage, 2.8, 4.2, 0, 100);

    if (bat_percentage > 100) {
      bat_percentage = 100;
    }
    if (bat_percentage < 0) {
      bat_percentage = 1;
    }



    sendBatteryInfo(bat_percentage);
    vTaskDelay(1000 / portTICK_PERIOD_MS);  // Delay để giảm tần suất đọc
  }
}

float mapfloat(float x, float in_min, float in_max, float out_min, float out_max) {
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

void buzzerBatTask(void *pvParameters) {
  while (true) {
    if (bat_percentage < 20) {
      if (xSemaphoreTake(buzzerMutex, portMAX_DELAY)) {
        playBuzzerTone(1, 2000, 100, 0);  // Phát cảnh báo quá tốc độ (5 lần bíp, 100ms mỗi lần)
        playBuzzerTone(1, 1950, 90, 0);
        playBuzzerTone(1, 1900, 90, 0);
        playBuzzerTone(1, 1800, 80, 0);
        xSemaphoreGive(buzzerMutex);  // Giải phóng semaphore
      }
    }
    // Kiểm tra nếu semaphore được cấp phát
    vTaskDelay(300 / portTICK_PERIOD_MS);  // Chu kỳ kiểm tra buzzer
  }
}
