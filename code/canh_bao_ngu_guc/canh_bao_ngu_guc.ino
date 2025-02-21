#include <Wire.h>
#include <MPU6050_tockn.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

#define BUZZER_PIN 5
#define BUZZER_CHANNEL 0
#define PWM_FREQ 5000
#define PWM_RESOLUTION 8

MPU6050 mpu(Wire);
const int alertAngleThreshold = 15; // Ngưỡng góc nghiêng cảnh báo
const unsigned long sleepGucDuration = 3000; // Thời gian nghiêng đầu liên tục để cảnh báo

float angleX = 0, angleY = 0;
bool isHeadNghieng = false;
unsigned long startNghiengTime = 0; // Khai báo biến toàn cục để theo dõi thời gian nghiêng

void setup() {
  pinMode(BUZZER_PIN, OUTPUT);
  ledcSetup(BUZZER_CHANNEL, PWM_FREQ, PWM_RESOLUTION);
  ledcAttachPin(BUZZER_PIN, BUZZER_CHANNEL);

  Serial.begin(115200);
  Wire.begin(27, 26);
  mpu.begin();
  mpu.calcGyroOffsets(true);
  Serial.println("Bắt đầu theo dõi trạng thái ngủ gục...");

  // Tạo các task
  xTaskCreate(readSensorTask, "Read Sensor", 2048, NULL, 1, NULL);
  xTaskCreate(checkNghiengTask, "Check Nghieng", 2048, NULL, 1, NULL);
  xTaskCreate(buzzerControlTask, "Buzzer Control", 2048, NULL, 1, NULL);
}

void loop() {
  // Không sử dụng loop vì FreeRTOS sẽ quản lý các task
}

void readSensorTask(void *pvParameters) {
  while (true) {
    mpu.update(); // Cập nhật giá trị cảm biến

    // Tính góc nghiêng
    angleX = mpu.getAngleX(); // Thay đổi từ getGyroX() sang getAngleX() 
    angleY = mpu.getAngleY(); // Thay đổi từ getGyroX() sang getAngleY() 

    // In giá trị góc ra Serial Monitor
    Serial.print("Góc X: ");
    Serial.print(angleX);
    Serial.print(" | Góc Y: ");
    Serial.println(angleY);

    vTaskDelay(100 / portTICK_PERIOD_MS); // Độ trễ giữa các lần đọc
  }
}

void checkNghiengTask(void *pvParameters) {
  while (true) {
    // Kiểm tra góc nghiêng
    if (abs(angleX) > alertAngleThreshold || abs(angleY) > alertAngleThreshold) {
      if (!isHeadNghieng) {
        startNghiengTime = millis(); // Lưu thời gian bắt đầu nghiêng
        isHeadNghieng = true; // Đánh dấu trạng thái là nghiêng
        Serial.println("Góc nghiêng phát hiện!");
      }

      // Kiểm tra thời gian nghiêng liên tục
      if (millis() - startNghiengTime >= sleepGucDuration) {
        Serial.println("Ngủ gục - Cảnh báo!");
        vTaskDelay(1000 / portTICK_PERIOD_MS); // Delay thêm một giây trước khi phát lại âm thanh
      } else {
        Serial.println("Đang theo dõi nghiêng...");
      }
    } else {
      if (isHeadNghieng) {
        // Nếu trước đó là nghiêng và bây giờ không còn nghiêng
        Serial.print("Thời gian nghiêng: ");
        Serial.print((millis() - startNghiengTime) / 1000.0); // In thời gian nghiêng ra giây
        Serial.println(" giây.");
      }
      isHeadNghieng = false; // Đặt lại trạng thái khi đầu không còn nghiêng
      Serial.println("Bình thường");
    }

    vTaskDelay(100 / portTICK_PERIOD_MS); // Độ trễ giữa các lần kiểm tra
  }
}

void buzzerControlTask(void *pvParameters) {
  while (true) {
    if (isHeadNghieng && (millis() - startNghiengTime >= sleepGucDuration)) {
      // Phát âm thanh cảnh báo
      for (int i = 0; i < 3; i++) { // Lặp lại 3 lần
        ledcWriteTone(BUZZER_CHANNEL, 1500); // Phát âm thanh với tần số 1000Hz
        vTaskDelay(100 / portTICK_PERIOD_MS); // Kêu trong 200 ms
        ledcWriteTone(BUZZER_CHANNEL, 0); // Dừng âm thanh
        vTaskDelay(50 / portTICK_PERIOD_MS); // Nghỉ trong 200 ms
      }
    } else {
      ledcWriteTone(BUZZER_CHANNEL, 0); // Dừng âm thanh
    }

    vTaskDelay(10 / portTICK_PERIOD_MS); // Độ trễ nhỏ giữa các lần kiểm tra
  }
}

// thêm gia tốc vào để kiểm tra, thêm điều kiện để mpu nằm dọc
