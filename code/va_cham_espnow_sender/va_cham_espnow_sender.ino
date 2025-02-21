#include <MPU6050_tockn.h>
#include <Wire.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <esp_now.h>
#include <WiFi.h>

// Cấu hình Buzzer
#define BUZZER_PIN 5
#define BUZZER_CHANNEL 0
#define PWM_FREQ 5000     // Tần số PWM
#define PWM_RESOLUTION 8  // Độ phân giải 8 bit

// Ngưỡng gia tốc cho các loại va chạm
const float LIGHT_COLLISION_THRESHOLD = 1.7;   // 2 m/s² cho va chạm nhẹ
const float MODERATE_COLLISION_THRESHOLD = 2;  // 2.5 m/s² cho va chạm trung bình
const float SEVERE_COLLISION_THRESHOLD = 2.2;  // 3 m/s² cho va chạm nặng

// MAC Address của responder
uint8_t broadcastAddress[] = { 0xD8, 0xBC, 0x38, 0xFB, 0x73, 0x44 };

// Định nghĩa cấu trúc dữ liệu
typedef struct struct_message {
  char message[32];
  float acceleration;
  char collision_type[20];
} struct_message;

// Tạo một đối tượng cấu trúc
struct_message myData;

// Khai báo MPU6050
MPU6050 mpu6050(Wire);
// Đăng ký peer
esp_now_peer_info_t peerInfo;
// Callback function khi gửi dữ liệu
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  Serial.print("\r\nLast Packet Send Status:\t");
  Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Delivery Success" : "Delivery Fail");
}

void setup() {
  Serial.begin(115200);
  Wire.begin(27, 26);  // Sử dụng SDA = GPIO27, SCL = GPIO26

  pinMode(BUZZER_PIN, OUTPUT);                          // Thiết lập chân Buzzer là đầu ra
  ledcSetup(BUZZER_CHANNEL, PWM_FREQ, PWM_RESOLUTION);  // Thiết lập kênh PWM
  ledcAttachPin(BUZZER_PIN, BUZZER_CHANNEL);            // Gán chân Buzzer cho kênh PWM

  Serial.println("Đang khởi tạo MPU6050...");
  mpu6050.begin();
  mpu6050.calcGyroOffsets(true);
  Serial.println("MPU6050 đã sẵn sàng!");


  // Thiết lập WiFi và ESP-NOW
  WiFi.mode(WIFI_STA);
  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }
  esp_now_register_send_cb(OnDataSent);


  memcpy(peerInfo.peer_addr, broadcastAddress, 6);
  peerInfo.channel = 0;
  peerInfo.encrypt = false;
  if (esp_now_add_peer(&peerInfo) != ESP_OK) {
    Serial.println("Failed to add peer");
    return;
  }

  // Tạo các tác vụ
  xTaskCreatePinnedToCore(TaskReadSensor, "TaskReadSensor", 2048, NULL, 1, NULL, 0);
  xTaskCreatePinnedToCore(TaskHandleBuzzer, "TaskHandleBuzzer", 2048, NULL, 1, NULL, 1);
}

void loop() {
  // Loop chính không làm gì, các tác vụ sẽ thực hiện công việc của chúng
}

// Tác vụ đọc dữ liệu cảm biến
void TaskReadSensor(void *pvParameters) {
  while (true) {
    mpu6050.update();

    // Đọc giá trị gia tốc
    float accelX = mpu6050.getAccX();
    float accelY = mpu6050.getAccY();
    float accelZ = mpu6050.getAccZ();

    // Tính gia tốc tổng hợp
    float totalAcceleration = sqrt(accelX * accelX + accelY * accelY + accelZ * accelZ);
    Serial.print("Gia tốc: ");
    Serial.print(totalAcceleration);

    // Xác định trạng thái va chạm
    if (totalAcceleration >= LIGHT_COLLISION_THRESHOLD && totalAcceleration < MODERATE_COLLISION_THRESHOLD) {
      Serial.println(" - Trạng thái: Va chạm nhẹ");
      strcpy(myData.collision_type, "Light Collision");
      strcpy(myData.message, "Collision Detected!");
    } else if (totalAcceleration >= MODERATE_COLLISION_THRESHOLD && totalAcceleration < SEVERE_COLLISION_THRESHOLD) {
      Serial.println(" - Trạng thái: Va chạm vừa");
      strcpy(myData.collision_type, "Moderate Collision");
      strcpy(myData.message, "Collision Detected!");
    } else if (totalAcceleration >= SEVERE_COLLISION_THRESHOLD) {
      Serial.println(" - Trạng thái: Va chạm nặng");
      strcpy(myData.collision_type, "Severe Collision");
      strcpy(myData.message, "Collision Detected!");
    } else {
      Serial.println(" - Trạng thái: Không có va chạm");
      strcpy(myData.collision_type, "No Collision");
      strcpy(myData.message, "No Collision");
    }

    // Gửi dữ liệu qua ESP-NOW
    myData.acceleration = totalAcceleration;  // Lưu gia tốc vào cấu trúc
    esp_err_t result = esp_now_send(broadcastAddress, (uint8_t *)&myData, sizeof(myData));
    if (result == ESP_OK) {
      Serial.println("Sending confirmed");
    } else {
      Serial.println("Sending error");
    }

    delay(50);  // Thêm khoảng dừng giữa các lần đọc dữ liệu
  }
}

// Tác vụ xử lý buzzer
void TaskHandleBuzzer(void *pvParameters) {
  while (true) {
    // Kiểm tra va chạm và phát âm thanh
    float accelX = mpu6050.getAccX();
    float accelY = mpu6050.getAccY();
    float accelZ = mpu6050.getAccZ();
    float totalAcceleration = sqrt(accelX * accelX + accelY * accelY + accelZ * accelZ);

    // Kiểm tra va chạm
    if (totalAcceleration >= LIGHT_COLLISION_THRESHOLD && totalAcceleration < MODERATE_COLLISION_THRESHOLD) {
      // Va chạm nhẹ: Buzzer kêu 2 lần, nghỉ 0.5 giây, lặp lại
      for (int i = 0; i < 5; i++) {             // Lặp lại 2 lần
        for (int i = 0; i < 2; i++) {           // Lặp lại 2 lần
          ledcWriteTone(BUZZER_CHANNEL, 1000);  // Phát âm thanh với tần số 1000Hz
          delay(100);                           // Kêu trong 200 ms
          ledcWriteTone(BUZZER_CHANNEL, 0);     // Dừng âm thanh
          delay(100);                           // Nghỉ trong 200 ms
        }
        delay(500);  // Nghỉ thêm 0.5 giây trước khi lặp lại
      }

    } else if (totalAcceleration >= MODERATE_COLLISION_THRESHOLD && totalAcceleration < SEVERE_COLLISION_THRESHOLD) {
      // Va chạm trung bình: Buzzer kêu 3 lần, nghỉ 0.5 giây, lặp lại
      for (int i = 0; i < 5; i++) {             // Lặp lại 2 lần
        for (int i = 0; i < 3; i++) {           // Lặp lại 2 lần
          ledcWriteTone(BUZZER_CHANNEL, 1000);  // Phát âm thanh với tần số 1000Hz
          delay(100);                           // Kêu trong 200 ms
          ledcWriteTone(BUZZER_CHANNEL, 0);     // Dừng âm thanh
          delay(100);                           // Nghỉ trong 200 ms
        }
        delay(500);  // Nghỉ thêm 0.5 giây trước khi lặp lại
      }
    } else if (totalAcceleration >= SEVERE_COLLISION_THRESHOLD) {
      // Va chạm nặng: Buzzer kêu liên tục
      for (int i = 0; i < 10; i++) {           // Lặp lại 2 lần
          ledcWriteTone(BUZZER_CHANNEL, 2000);  // Phát âm thanh với tần số 1000Hz
          delay(100);                           // Kêu trong 200 ms
          ledcWriteTone(BUZZER_CHANNEL, 0);     // Dừng âm thanh
          delay(100);                           // Nghỉ trong 200 ms
        }
      // delay(5000);
    } else {
      ledcWriteTone(BUZZER_CHANNEL, 0);  // Tắt âm thanh nếu không có va chạm
    }

    delay(200);  // Thêm khoảng dừng giữa các lần xử lý
  }
}
