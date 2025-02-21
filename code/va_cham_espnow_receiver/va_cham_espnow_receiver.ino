// Include Libraries
#include <esp_now.h>
#include <WiFi.h>

// Define a data structure
typedef struct struct_message {
  char message[32];
  float acceleration;      // Gia tốc
  char collision_type[20]; // Loại va chạm
} struct_message;

// Create a structured object
struct_message myData;

// Define pin for Buzzer
#define BUZZER_PIN 5
#define BUZZER_CHANNEL 0
#define PWM_FREQ 5000   // Tần số PWM
#define PWM_RESOLUTION 8  // Độ phân giải 8 bit

void setup() {
  // Set up Serial Monitor
  Serial.begin(115200);
  
  // Set ESP32 as a Wi-Fi Station
  WiFi.mode(WIFI_STA);

  // Initialize Buzzer pin
  pinMode(BUZZER_PIN, OUTPUT); // Thiết lập chân Buzzer là đầu ra
  ledcSetup(BUZZER_CHANNEL, PWM_FREQ, PWM_RESOLUTION); // Thiết lập kênh PWM
  ledcAttachPin(BUZZER_PIN, BUZZER_CHANNEL); // Gán chân Buzzer cho kênh PWM

  // Initialize ESP-NOW
  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }
  
  // Register callback function
  esp_now_register_recv_cb(OnDataRecv);
}
 
void loop() {
  // Loop chính không làm gì
}

// Callback function executed when data is received
void OnDataRecv(const uint8_t * mac, const uint8_t *incomingData, int len) {
  memcpy(&myData, incomingData, sizeof(myData));
  
  // Print received data to Serial Monitor
  Serial.print("Received message: ");
  Serial.println(myData.message);
  Serial.print("Acceleration: ");
  Serial.println(myData.acceleration);
  Serial.print("Collision Type: ");
  Serial.println(myData.collision_type);
  
  // Process collision alert and activate buzzer
  if (myData.acceleration > 0) { // Kiểm tra nếu có va chạm
    if (strcmp(myData.collision_type, "Light Collision") == 0) { // Va chạm nhẹ
      for (int i = 0; i < 5; i++) {             // Lặp lại 2 lần
        for (int i = 0; i < 2; i++) {           // Lặp lại 2 lần
          ledcWriteTone(BUZZER_CHANNEL, 1000);  // Phát âm thanh với tần số 1000Hz
          delay(100);                           // Kêu trong 200 ms
          ledcWriteTone(BUZZER_CHANNEL, 0);     // Dừng âm thanh
          delay(100);                           // Nghỉ trong 200 ms
        }
        delay(500);  // Nghỉ thêm 0.5 giây trước khi lặp lại
      }
    } else if (strcmp(myData.collision_type, "Moderate Collision") == 0) { // Va chạm vừa
      for (int i = 0; i < 5; i++) {             // Lặp lại 2 lần
        for (int i = 0; i < 3; i++) {           // Lặp lại 2 lần
          ledcWriteTone(BUZZER_CHANNEL, 1000);  // Phát âm thanh với tần số 1000Hz
          delay(100);                           // Kêu trong 200 ms
          ledcWriteTone(BUZZER_CHANNEL, 0);     // Dừng âm thanh
          delay(100);                           // Nghỉ trong 200 ms
        }
        delay(500);  // Nghỉ thêm 0.5 giây trước khi lặp lại
      }
    } else if (strcmp(myData.collision_type, "Severe Collision") == 0) { // Va chạm nặng
      for (int i = 0; i < 10; i++) {           // Lặp lại 2 lần
          ledcWriteTone(BUZZER_CHANNEL, 2000);  // Phát âm thanh với tần số 1000Hz
          delay(100);                           // Kêu trong 200 ms
          ledcWriteTone(BUZZER_CHANNEL, 0);     // Dừng âm thanh
          delay(100);                           // Nghỉ trong 200 ms
        }
      // delay(5000);
    }
  } else if (strcmp(myData.collision_type, "No Collision") == 0) { // Không va chạm
    ledcWriteTone(BUZZER_CHANNEL, 0); // Dừng âm thanh
  }
}
