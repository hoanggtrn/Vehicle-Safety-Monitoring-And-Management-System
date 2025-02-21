#include <MPU6050_tockn.h>
#include <Wire.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <esp_now.h>
#include <WiFi.h>
#include <SimpleKalmanFilter.h>

#define BUZZER_PIN 5
#define BUZZER_CHANNEL 0
#define PWM_FREQ 5000
#define PWM_RESOLUTION 8
#define LED_PIN 2  // Chân LED báo kết nối ESP-NOW

const int alertAngleThreshold = 15;
const unsigned long sleepGucDuration = 3000;

MPU6050 mpu(Wire);
bool isHeadNghieng = false;
unsigned long startNghiengTime = 0;

uint8_t broadcastAddress[] = { 0xD8, 0xBC, 0x38, 0xFB, 0x73, 0x44 };  // MAC ESP32 receiver

// Cấu hình bộ lọc Kalman cho X và Y
SimpleKalmanFilter kalmanFilterX(2, 2, 0.01);
SimpleKalmanFilter kalmanFilterY(2, 2, 0.01);

typedef struct struct_message {
  bool sleepGucWarning;
  char message[32];
  float acceleration;
  char collision_type[20];
} struct_message;

SemaphoreHandle_t buzzerMutex;

struct_message myData;
esp_now_peer_info_t peerInfo;

void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  // Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Delivery Success" : "Delivery Fail");
  digitalWrite(LED_PIN, status == ESP_NOW_SEND_SUCCESS ? HIGH : LOW);  // Bật LED nếu gửi thành công
}

// Callback function executed when data is received
void OnDataRecv(const uint8_t *mac, const uint8_t *incomingData, int len) {
  memcpy(&myData, incomingData, sizeof(myData));
  Serial.print("Received message: ");
  Serial.println(myData.message);
  Serial.print("Acceleration: ");
  Serial.println(myData.acceleration);
  Serial.print("Collision Type: ");
  Serial.println(myData.collision_type);
  digitalWrite(LED_PIN, HIGH);  // Bật LED khi nhận dữ liệu
}

void sendSleepGucWarning(bool warning) {
  myData.sleepGucWarning = warning;
  esp_now_send(broadcastAddress, (uint8_t *)&myData, sizeof(myData));
}

void setup() {
  Serial.begin(115200);
  Wire.begin(27, 26);
  pinMode(BUZZER_PIN, OUTPUT);
  ledcSetup(BUZZER_CHANNEL, PWM_FREQ, PWM_RESOLUTION);
  ledcAttachPin(BUZZER_PIN, BUZZER_CHANNEL);
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);  // Tắt LED lúc đầu

  testBuzzer();
    buzzerMutex = xSemaphoreCreateMutex();
  
  mpu.begin();
  mpu.calcGyroOffsets(true);
  Serial.println("Bắt đầu theo dõi trạng thái ngủ gục...");

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

  // xTaskCreatePinnedToCore(readSensorTask, "Read Sensor", 2048, NULL, 1, NULL, 0);
  xTaskCreatePinnedToCore(checkNghiengTask, "Check Nghieng", 2048, NULL, 1, NULL, 1);
  xTaskCreatePinnedToCore(buzzerControlTask, "Buzzer Control", 2048, NULL, 1, NULL, 1);
  xTaskCreate(TaskHandleBuzzer, "TaskHandleBuzzer", 2048, NULL, 1, NULL);
}

void loop() {}

// void readSensorTask(void *pvParameters) {
//   while (true) {
//     mpu.update();
//     vTaskDelay(100 / portTICK_PERIOD_MS);
//   }
// }

void checkNghiengTask(void *pvParameters) {
  while (true) {
    mpu.update();
    float angleX = kalmanFilterX.updateEstimate(mpu.getAngleX());
    float angleY = kalmanFilterY.updateEstimate(mpu.getAngleY());
    // In giá trị góc ra Serial Monitor
    Serial.print("Góc X: ");
    Serial.print(angleX);
    Serial.print(" | Góc Y: ");
    Serial.println(angleY);

    if (abs(angleX) > alertAngleThreshold || abs(angleY) > alertAngleThreshold) {
      if (!isHeadNghieng) {
        startNghiengTime = millis();
        isHeadNghieng = true;
      }

      if (millis() - startNghiengTime >= sleepGucDuration) {
        sendSleepGucWarning(true);
        // for (int i = 0; i < 3; i++) {
        //   ledcWriteTone(BUZZER_CHANNEL, 1500);
        //   vTaskDelay(100 / portTICK_PERIOD_MS);
        //   ledcWriteTone(BUZZER_CHANNEL, 0);
        //   vTaskDelay(50 / portTICK_PERIOD_MS);
        // }
      }
    } else {
      isHeadNghieng = false;
      sendSleepGucWarning(false);
    }

    vTaskDelay(100 / portTICK_PERIOD_MS);
  }
}

void buzzerControlTask(void *pvParameters) {
  while (true) {
    if (xSemaphoreTake(buzzerMutex, portMAX_DELAY)) { // Lấy mutex
      if (isHeadNghieng && (millis() - startNghiengTime >= sleepGucDuration)) {
        for (int i = 0; i < 3; i++) {
          ledcWriteTone(BUZZER_CHANNEL, 1500);
          vTaskDelay(100 / portTICK_PERIOD_MS);
          ledcWriteTone(BUZZER_CHANNEL, 0);
          vTaskDelay(50 / portTICK_PERIOD_MS);
        }
      } else {
        ledcWriteTone(BUZZER_CHANNEL, 0);
      }
      xSemaphoreGive(buzzerMutex); // Giải phóng mutex
    }
    vTaskDelay(10 / portTICK_PERIOD_MS);
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
    if (xSemaphoreTake(buzzerMutex, portMAX_DELAY)) { // Lấy mutex
      if (myData.acceleration > 0) {
        if (strcmp(myData.collision_type, "Light Collision") == 0) {
          playBuzzerTone(5, 1000, 100, 200);  // 5 beeps, 100ms delay
        } else if (strcmp(myData.collision_type, "Moderate Collision") == 0) {
          playBuzzerTone(5, 1500, 100, 200);  // 5 beeps, 100ms delay
        } else if (strcmp(myData.collision_type, "Severe Collision") == 0) {
          playBuzzerTone(10, 2000, 100, 200);  // Continuous beep
        }
      } else if (strcmp(myData.collision_type, "No Collision") == 0) {
        ledcWriteTone(BUZZER_CHANNEL, 0);  // Stop buzzer if no collision
      }
      xSemaphoreGive(buzzerMutex); // Giải phóng mutex
    }
    vTaskDelay(200 / portTICK_PERIOD_MS);  // Polling interval
  }
}

void testBuzzer() {
  playBuzzerTone(5, 1000, 100, 200);  // 5 beeps, 100ms delay
}
