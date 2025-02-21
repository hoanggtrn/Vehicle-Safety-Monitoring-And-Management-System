const int hallPin = 39;           // Chân kết nối với cảm biến Hall (chân analog)
volatile int pulseCount = 0;      // Biến đếm số xung
unsigned long lastPulseTime = 0;  // Thời gian nhận xung trước đó
float speed = 0.0;                // Vận tốc xe (km/h)
const int numMagnets = 9;         // Số nam châm trên bánh xe
float wheelRadius = 0.3;          // Bán kính bánh xe (m)
float magnetRadius = 0.15;        // Bán kính vị trí đặt nam châm từ tâm bánh xe (m)
const int analogThreshold = 4000; // Ngưỡng tín hiệu analog từ cảm biến Hall

void setup() {
  Serial.begin(115200);
  pinMode(hallPin, INPUT); // Thiết lập chân cảm biến Hall là đầu vào (analog)
}

void loop() {
  // Đọc tín hiệu từ cảm biến Hall
  int sensorValue = analogRead(hallPin); // Đọc giá trị analog từ cảm biến Hall

  // Kiểm tra nếu giá trị tín hiệu vượt qua ngưỡng (tín hiệu có sự thay đổi từ mức cao xuống thấp khi nam châm đi qua)
  if (sensorValue < analogThreshold) {  // Tín hiệu xuống thấp (0V) khi nam châm gần
    // Nếu có xung (nam châm gần), tính toán tốc độ
    unsigned long currentTime = millis();
    unsigned long deltaTime = currentTime - lastPulseTime;  // Thời gian giữa hai xung

    if (deltaTime > 0) {
      // Tính tốc độ tại vị trí nam châm (m/s)
      float magnetVelocity = (2 * PI * magnetRadius) / (deltaTime / 1000.0); 

      // Chuyển đổi vận tốc tại vị trí nam châm thành vận tốc xe thực tế
      speed = (magnetVelocity * (wheelRadius / magnetRadius)) * 3.6;  // km/h

      // In tốc độ


      // Cập nhật lại thời gian nhận xung
      lastPulseTime = currentTime;

      // Đợi tín hiệu trở lại mức cao trước khi tiếp tục
      while (analogRead(hallPin) < analogThreshold);  // Chờ đến khi tín hiệu quay lại mức cao (nam châm đã rời xa)
    }
  }
      Serial.print("Speed: ");
      Serial.print(speed);
      Serial.print(" km/h");
      Serial.print("   sensorValue: ");
    Serial.println(sensorValue);
    delay(50);
}
