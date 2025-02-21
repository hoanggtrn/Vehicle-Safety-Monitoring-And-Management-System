#include <esp_now.h>
#include <WiFi.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

// Define a data structure
typedef struct struct_message {
  char message[32];
  float acceleration;
  char collision_type[20];
} struct_message;

struct_message myData;

// Define pin for Buzzer
#define BUZZER_PIN 5
#define BUZZER_CHANNEL 0
#define PWM_FREQ 5000
#define PWM_RESOLUTION 8

void setup() {
  Serial.begin(115200);
  WiFi.mode(WIFI_STA);

  pinMode(BUZZER_PIN, OUTPUT);
  ledcSetup(BUZZER_CHANNEL, PWM_FREQ, PWM_RESOLUTION);
  ledcAttachPin(BUZZER_PIN, BUZZER_CHANNEL);

  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }

  // Register callback function for receiving data
  esp_now_register_recv_cb(OnDataRecv);

  // Create tasks
  xTaskCreate(TaskHandleBuzzer, "TaskHandleBuzzer", 2048, NULL, 1, NULL);

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
