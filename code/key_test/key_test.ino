#define RELAY_KEY 12   // Chân điều khiển relay chìa khóa
#define RELAY_START 14 // Chân điều khiển relay đề xe
#define KEY_INPUT 35   // Chân nhận tín hiệu từ chìa khóa

void setup() {
  // Khởi tạo chân GPIO
  pinMode(RELAY_KEY, OUTPUT); 
  pinMode(RELAY_START, OUTPUT); 
  pinMode(KEY_INPUT, INPUT); 

  // Đặt trạng thái ban đầu cho các relay
  digitalWrite(RELAY_KEY, LOW);  // Relay chìa khóa ban đầu LOW
  digitalWrite(RELAY_START, HIGH); // Relay đề xe ban đầu HIGH

  // Khởi động Serial để debug
  Serial.begin(115200);
}

void loop() {
  // Đọc trạng thái chìa khóa
  int keyState = analogRead(KEY_INPUT);

  // Kiểm tra trạng thái
  if (keyState > 4000) { 
    // Chìa khóa đang bật
    Serial.print("Xe đang bật");
    Serial.println(keyState);
    // digitalWrite(RELAY_KEY_PIN, HIGH);  // Bật relay chìa khóa
    // digitalWrite(RELAY_START_PIN, LOW); // Bật relay đề xe
  } else {
    // Chìa khóa đang tắt
    Serial.println("Xe đang tắt");
    Serial.println(keyState);
    // digitalWrite(RELAY_KEY_PIN, LOW);  // Tắt relay chìa khóa
    // digitalWrite(RELAY_START_PIN, HIGH); // Tắt relay đề xe
  }

  delay(500); // Giảm tải CPU
}
