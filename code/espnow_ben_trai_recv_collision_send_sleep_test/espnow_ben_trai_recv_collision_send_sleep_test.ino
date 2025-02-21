#include <esp_now.h>
#include <WiFi.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <MPU6050_tockn.h>
#include <Wire.h>

// Define pin for Buzzer
#define BUZZER_PIN 5
#define BUZZER_CHANNEL 0
#define PWM_FREQ 5000
#define PWM_RESOLUTION 8

const int alertAngleThreshold = 15;
const unsigned long sleepGucDuration = 3000;

MPU6050 mpu(Wire);
bool isHeadNghieng = false;
unsigned long startNghiengTime = 0;

uint8_t broadcastAddress[] = { 0xD8, 0xBC, 0x38, 0xFB, 0x73, 0x44 }; // MAC ESP32 receiver

// Define a data structure
typedef struct struct_message {
  char message[32];
  float acceleration;
  char collision_type[20];
  bool sleepGucWarning;
} struct_message;

struct_message myData;
esp_now_peer_info_t peerInfo;

void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Delivery Success" : "Delivery Fail");
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

  mpu.begin();
  mpu.calcGyroOffsets(true);
  Serial.println("Bắt đầu theo dõi trạng thái ngủ gục...");

  WiFi.mode(WIFI_STA);
  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }

  // Register callback function for receiving data
  esp_now_register_recv_cb(OnDataRecv);
  memcpy(peerInfo.peer_addr, broadcastAddress, 6);
  peerInfo.channel = 0;
  peerInfo.encrypt = false;
  esp_now_add_peer(&peerInfo);

  // Create tasks
  xTaskCreate(TaskHandleBuzzer, "TaskHandleBuzzer", 2048, NULL, 1, NULL);
  xTaskCreatePinnedToCore(readSensorTask, "Read Sensor", 2048, NULL, 1, NULL, 0);
  xTaskCreatePinnedToCore(checkNghiengTask, "Check Nghieng", 2048, NULL, 1, NULL, 1);
  xTaskCreatePinnedToCore(buzzerControlTask, "Buzzer Control", 2048, NULL, 1, NULL, 1);
}

void loop() {
  // Empty main loop
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
}

// Function to control buzzer based on collision type
void playCollisionAlert(int repeatCount, int toneFrequency, int beepDelay, int pauseDelay) {
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
    if (myData.acceleration > 0) {
      if (strcmp(myData.collision_type, "Light Collision") == 0) {
        playCollisionAlert(5, 1000, 100, 500);  // 5 beeps, 100ms delay
      } else if (strcmp(myData.collision_type, "Moderate Collision") == 0) {
        playCollisionAlert(5, 1000, 100, 500);  // 5 beeps, 100ms delay
      } else if (strcmp(myData.collision_type, "Severe Collision") == 0) {
        playCollisionAlert(10, 2000, 100, 1000); // Continuous beep
      }
    } else if (strcmp(myData.collision_type, "No Collision") == 0) {
      ledcWriteTone(BUZZER_CHANNEL, 0);  // Stop buzzer if no collision
    }
    vTaskDelay(200 / portTICK_PERIOD_MS);  // Polling interval
  }
}

void readSensorTask(void *pvParameters) {
  while (true) {
    mpu.update();
    vTaskDelay(100 / portTICK_PERIOD_MS);
  }
}

void checkNghiengTask(void *pvParameters) {
  while (true) {
    float angleX = mpu.getAngleX();
    float angleY = mpu.getAngleY();

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

    vTaskDelay(100 / portTICK_PERIOD_MS);
  }
}

void buzzerControlTask(void *pvParameters) {
  while (true) {
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

    vTaskDelay(10 / portTICK_PERIOD_MS);
  }
}
