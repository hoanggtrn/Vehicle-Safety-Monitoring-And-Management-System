#include <TinyGPSPlus.h>
#include <Arduino.h>
#include <HardwareSerial.h>

// Khai báo đối tượng TinyGPSPlus và UART cho SIM7680C
TinyGPSPlus gps;
HardwareSerial simSerial(2);  // UART2
#define SIM_TX 32
#define SIM_RX 33

// Cấu hình GPS
HardwareSerial gpsSerial(1);           
static const uint32_t GPSBaud = 9600;  

// Relay công tắc xe
#define RELAY_SWITCH 4

// Prototype các hàm
void setupSIM();
String sendATCommand(const char* command, unsigned long timeout = 15000);
void sendSMS(const char* phoneNumber, const char* message);
void listenForMessages(void* pvParameters);
void readGPSTask(void* parameter);
void printGPSData();

void setup() {
  Serial.begin(115200);  
  pinMode(RELAY_SWITCH, OUTPUT);
  digitalWrite(RELAY_SWITCH, HIGH);
  setupSIM();
  
  gpsSerial.begin(GPSBaud, SERIAL_8N1, 16, 17);  

  // In tiêu đề
  Serial.printf("%-15s %-15s %-15s %-10s %-10s\n", "Latitude", "Longitude", "Time", "Satellites", "HDOP");
  Serial.println(F("-----------------------------------------------------------"));

  xTaskCreatePinnedToCore(readGPSTask, "Read GPS Task", 2048, NULL, 1, NULL, 1);
  xTaskCreate(listenForMessages, "Listen for Messages", 8000, NULL, 1, NULL);

  Serial.println(sendATCommand("AT"));
}

void loop() {}

void readGPSTask(void* parameter) {
  static int lastGPSChars = 0;  
  static unsigned long lastGPSTime = 0; 

  for (;;) {
    while (gpsSerial.available() > 0) gps.encode(gpsSerial.read());

    int currentGPSChars = gps.charsProcessed();

    if (currentGPSChars != lastGPSChars) {
      lastGPSChars = currentGPSChars;
      lastGPSTime = millis(); 

      printGPSData();
    } else if (millis() - lastGPSTime > 5000) {
      Serial.println("No GPS data received: check wiring");
    }

    vTaskDelay(pdMS_TO_TICKS(1000));
  }
}

void printGPSData() {
  if (gps.location.isValid() && gps.date.isValid() && gps.time.isValid() && gps.satellites.isValid() && gps.hdop.isValid()) {
    int hour = (gps.time.hour() + 7) % 24;
    Serial.printf("%-15.6f %-15.6f %02d/%02d/%-10d %02d:%02d:%-05d %-05d %-10.1f\n",
                  gps.location.lat(),
                  gps.location.lng(),
                  gps.date.day(),
                  gps.date.month(),
                  gps.date.year(),
                  hour,
                  gps.time.minute(),
                  gps.time.second(),
                  gps.satellites.value(),
                  gps.hdop.hdop());
  } else {
    Serial.println("No valid GPS data available.");
  }
}

void setupSIM() {
  simSerial.begin(115200, SERIAL_8N1, SIM_RX, SIM_TX);  
  vTaskDelay(pdMS_TO_TICKS(100));
}

String sendATCommand(const char* command, unsigned long timeout) {
  simSerial.println(command);
  unsigned long startTime = millis();
  while (millis() - startTime < timeout) {
    if (simSerial.available()) {
      return simSerial.readStringUntil('\n');
    }
  }
  return "";
}

void sendSMS(const char* phoneNumber, const char* message) {
  simSerial.println("AT+CMGF=1");
  vTaskDelay(pdMS_TO_TICKS(100));  

  simSerial.print("AT+CMGS=\"");
  simSerial.print(phoneNumber);  
  simSerial.println("\"");

  vTaskDelay(pdMS_TO_TICKS(100));  

  simSerial.print(message);
  simSerial.write(26);  
  vTaskDelay(pdMS_TO_TICKS(2000));  

  Serial.println("Response: " + simSerial.readString());  
}

String getGSMStatus() {
  simSerial.println("AT+CSQ");
  vTaskDelay(pdMS_TO_TICKS(100));

  if (simSerial.available()) {
    String response = simSerial.readString();
    int signalStrengthIndex = response.indexOf("+CSQ: ");
    if (signalStrengthIndex != -1) {
      String signalStrength = response.substring(signalStrengthIndex + 6, signalStrengthIndex + 8);
      return (signalStrength == "99") ? "Không có tín hiệu GSM" : signalStrength;
    }
  }
  return "Không thể lấy tín hiệu GSM.";
}

void listenForMessages(void* pvParameters) {
  String incomingMessage;
  String senderNumber;

  for (;;) {
    if (simSerial.available()) {
      incomingMessage = simSerial.readString();
      int start = incomingMessage.indexOf("+CMT: \"") + 7;
      int end = incomingMessage.indexOf("\"", start);
      if (start != -1 && end != -1) senderNumber = incomingMessage.substring(start, end);

      Serial.println(incomingMessage);

      if (incomingMessage.indexOf("report") != -1) {
        if (!senderNumber.isEmpty()) {
          sendSMS(senderNumber.c_str(), generateReportMessage().c_str());
          sendSMS(senderNumber.c_str(), generateAlertMessage().c_str());
        }
      } else if (incomingMessage.indexOf("unlock") != -1) {
        digitalWrite(RELAY_SWITCH, LOW);
        Serial.println("Đã mở khóa xe");
        if (!senderNumber.isEmpty()) sendSMS(senderNumber.c_str(), "Da mo khoa xe");
      } else if (incomingMessage.indexOf("lock") != -1) {
        digitalWrite(RELAY_SWITCH, HIGH);
        Serial.println("Đã khóa xe");
        if (!senderNumber.isEmpty()) sendSMS(senderNumber.c_str(), "Da khoa xe!");
      }
    }
    vTaskDelay(pdMS_TO_TICKS(200));
  }
}

String generateReportMessage() {
  String reportMessage = "- Status: \n- User: \n- Key status: \n";
  if (gps.location.isValid()) {
    reportMessage += "- Location: https://www.google.com/maps?q=" + String(gps.location.lat(), 6) + "," + String(gps.location.lng(), 6) + "\n";
    reportMessage += "- GPS satellites: " + String(gps.satellites.value()) + "\n";
    reportMessage += "- HDOP: " + String(gps.hdop.hdop(), 1) + "\n";
  } else {
    reportMessage += "- No GPS signals!!!\n";
  }
  reportMessage += "- GSM: " + getGSMStatus() + "\n";
  return reportMessage;
}

String generateAlertMessage() {
  return "- Speed: \n- Drive times: \n- Overspeeding: \n- Sleepy times: \n- Impact: \n";
}
