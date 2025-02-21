#include <esp_now.h>
#include <WiFi.h>

typedef struct struct_message {
  bool sleepGucWarning;
} struct_message;

struct_message myData;

void OnDataRecv(const uint8_t * mac, const uint8_t *incomingData, int len) {
  memcpy(&myData, incomingData, sizeof(myData));
  
  if (myData.sleepGucWarning) {
    Serial.println("Cảnh báo: Ngủ gục!");
  } else {
    Serial.println("Trạng thái bình thường.");
  }
}

void setup() {
  Serial.begin(115200);
  WiFi.mode(WIFI_STA);
  
  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }
  esp_now_register_recv_cb(OnDataRecv);
}

void loop() {}
