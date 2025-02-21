#include <MPU6050_tockn.h>
#include <Wire.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

MPU6050 mpu6050(Wire);

// Định nghĩa chân cho Buzzer và kênh LEDC
#define BUZZER_PIN 5
#define BUZZER_CHANNEL 0
#define PWM_FREQ 5000   // Tần số PWM
#define PWM_RESOLUTION 8  // Độ phân giải 8 bit

// Ngưỡng gia tốc cho các loại va chạm
const float LIGHT_COLLISION_THRESHOLD = 1.7; // 2 m/s² cho va chạm nhẹ
const float MODERATE_COLLISION_THRESHOLD = 2; // 2.5 m/s² cho va chạm trung bình
const float SEVERE_COLLISION_THRESHOLD = 2.2; // 3 m/s² cho va chạm nặng

void setup() {
  Serial.begin(115200);
  Wire.begin(27, 26);   // Sử dụng SDA = GPIO27, SCL = GPIO26

  pinMode(BUZZER_PIN, OUTPUT); // Thiết lập chân Buzzer là đầu ra
  ledcSetup(BUZZER_CHANNEL, PWM_FREQ, PWM_RESOLUTION); // Thiết lập kênh PWM
  ledcAttachPin(BUZZER_PIN, BUZZER_CHANNEL); // Gán chân Buzzer cho kênh PWM

  Serial.println("Đang khởi tạo MPU6050...");
  mpu6050.begin();
  mpu6050.calcGyroOffsets(true); 
  Serial.println("MPU6050 đã sẵn sàng!");

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

    // Hiển thị gia tốc
    Serial.print("Gia tốc: ");
    Serial.print(totalAcceleration);
    
    // Xác định trạng thái va chạm
    if (totalAcceleration >= LIGHT_COLLISION_THRESHOLD && totalAcceleration < MODERATE_COLLISION_THRESHOLD) {
      Serial.println(" - Trạng thái: Va chạm nhẹ");
    } 
    else if (totalAcceleration >= MODERATE_COLLISION_THRESHOLD && totalAcceleration < SEVERE_COLLISION_THRESHOLD) {
      Serial.println(" - Trạng thái: Va chạm vừa");
    } 
    else if (totalAcceleration >= SEVERE_COLLISION_THRESHOLD) {
      Serial.println(" - Trạng thái: Va chạm nặng");
    } 
    else {
      Serial.println(" - Trạng thái: Không có va chạm");
    }

    delay(100); // Thêm khoảng dừng giữa các lần đọc dữ liệu
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
      for (int i = 0; i < 5; i++) { // Lặp lại 5 lần
        ledcWriteTone(BUZZER_CHANNEL, 1000); // Phát âm thanh với tần số 1000Hz
        delay(200); // Kêu trong 200 ms
        ledcWriteTone(BUZZER_CHANNEL, 0); // Dừng âm thanh
        delay(200); // Nghỉ trong 200 ms
      }
      delay(500); // Nghỉ thêm 0.5 giây trước khi lặp lại
    } 
    else if (totalAcceleration >= MODERATE_COLLISION_THRESHOLD && totalAcceleration < SEVERE_COLLISION_THRESHOLD) {
      // Va chạm trung bình: Buzzer kêu 3 lần, nghỉ 0.5 giây, lặp lại
      for (int i = 0; i < 5; i++) { // Lặp lại 5 lần
        for (int j = 0; j < 3; j++) { // Kêu 3 lần
          ledcWriteTone(BUZZER_CHANNEL, 1000); // Phát âm thanh với tần số 1000Hz
          delay(200); // Kêu trong 200 ms
          ledcWriteTone(BUZZER_CHANNEL, 0); // Dừng âm thanh
          delay(200); // Nghỉ trong 200 ms
        }
        delay(500); // Nghỉ thêm 0.5 giây trước khi lặp lại
      }
    } 
    else if (totalAcceleration >= SEVERE_COLLISION_THRESHOLD) {
      // Va chạm nặng: Buzzer kêu liên tục
      ledcWriteTone(BUZZER_CHANNEL, 1000); // Phát âm thanh liên tục với tần số 1000Hz
    } 
    else {
      ledcWriteTone(BUZZER_CHANNEL, 0); // Tắt âm thanh nếu không có va chạm
    }

    delay(200);  // Thêm khoảng dừng giữa các lần xử lý
  }
}

