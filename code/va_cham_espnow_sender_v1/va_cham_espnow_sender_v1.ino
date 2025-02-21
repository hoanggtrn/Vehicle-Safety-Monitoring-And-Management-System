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

const float LIGHT_COLLISION_THRESHOLD = 1.7;
const float MODERATE_COLLISION_THRESHOLD = 2;
const float SEVERE_COLLISION_THRESHOLD = 2.2;

uint8_t broadcastAddress[] = { 0xD8, 0xBC, 0x38, 0xFB, 0xB1, 0x90 };

typedef struct struct_message {
  char message[32];
  float acceleration;
  char collision_type[20];
} struct_message;

struct_message myData;
MPU6050 mpu6050(Wire);
esp_now_peer_info_t peerInfo;

void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Delivery Success" : "Delivery Fail");
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
  strcpy(myData.collision_type, collision_type);
  strcpy(myData.message, "Collision Detected!");
  myData.acceleration = acceleration;
  esp_now_send(broadcastAddress, (uint8_t *)&myData, sizeof(myData));
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
      strcpy(myData.collision_type, "No Collision");
      strcpy(myData.message, "No Collision");
      myData.acceleration = totalAcceleration;
      esp_now_send(broadcastAddress, (uint8_t *)&myData, sizeof(myData));
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
      playBuzzerTone(2, 100, 1000);
    } else if (totalAcceleration >= MODERATE_COLLISION_THRESHOLD && totalAcceleration < SEVERE_COLLISION_THRESHOLD) {
      playBuzzerTone(3, 100, 1000);
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
  memcpy(peerInfo.peer_addr, broadcastAddress, 6);
  peerInfo.channel = 0;
  peerInfo.encrypt = false;
  esp_now_add_peer(&peerInfo);
}

void setup() {
  Serial.begin(115200);
  Wire.begin(27, 26);
  pinMode(BUZZER_PIN, OUTPUT);
  ledcSetup(BUZZER_CHANNEL, PWM_FREQ, PWM_RESOLUTION);
  ledcAttachPin(BUZZER_PIN, BUZZER_CHANNEL);

  Serial.println("Đang khởi tạo MPU6050...");
  mpu6050.begin();
  mpu6050.calcGyroOffsets(true);
  Serial.println("MPU6050 đã sẵn sàng!");

  initializeESPNow();

  xTaskCreatePinnedToCore(TaskReadSensor, "TaskReadSensor", 2048, NULL, 1, NULL, 0);
  xTaskCreatePinnedToCore(TaskHandleBuzzer, "TaskHandleBuzzer", 2048, NULL, 1, NULL, 1);
}

void loop() {}
