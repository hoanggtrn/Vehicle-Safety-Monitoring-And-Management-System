#include <SPI.h>
#include <SD.h>

// Chân CS của module SD
#define SD_CS 5

File logFile;

void setup() {
  Serial.begin(115200);
  while (!Serial) {
    delay(10); // Chờ Serial kết nối
  }

  Serial.println("Khởi động SD card...");

  // Khởi tạo SD card
  if (!SD.begin(SD_CS)) {
    Serial.println("Không thể khởi tạo SD card!");
    while (true);
  }
  Serial.println("SD card đã sẵn sàng.");

  // Tạo file log.csv
  createLogFile();
}

void loop() {
  // Thêm dữ liệu vào file log.csv (giả lập ví dụ)
  appendLog(
    "2024-11-27 12:00:00",
    "Nguyen Van A",
    "Nguyen Van B",
    "Dang nhap",
    "Dang xuat",
    "10.762622",
    "106.660172",
    7,
    1.5,
    40.5,
    "12:30",
    12.3,
    85,
    "Qua toc do"
  );
  
  delay(5000); // Giả lập thêm dữ liệu mỗi 5 giây
}

// Hàm tạo file log với tiêu đề
void createLogFile() {
  logFile = SD.open("/log.csv", FILE_WRITE);

  if (logFile) {
    logFile.println(
      "Thoi gian,Nguoi dung,,Trang thai,,Vi tri,,,,GSM,Van toc,Thoi gian lai xe lien tuc,\"Dien ap ngo vao\",Pin (slave),Canh bao,,,");
    logFile.println(
      ",Nguyen Van A,Nguyen Van B,Dang nhap,Dang xuat,Lat,Lon,So ve tinh,HDOP,,,,,,Qua toc do,Ngu guc,Va cham,Mat cap");
    logFile.close();
    Serial.println("Tạo file log.csv thành công.");
  } else {
    Serial.println("Không thể tạo file log.csv!");
  }
}

// Hàm thêm một dòng dữ liệu vào file log.csv
void appendLog(
  String thoiGian,
  String nguoiDungA,
  String nguoiDungB,
  String trangThaiA,
  String trangThaiB,
  String lat,
  String lon,
  int soVeTinh,
  float hdop,
  float vanToc,
  String thoiGianLaiXeLienTuc,
  float dienApNgoVao,
  int pinSlave,
  String canhBao
) {
  logFile = SD.open("/log.csv", FILE_APPEND);

  if (logFile) {
    logFile.print(thoiGian);
    logFile.print(",");
    logFile.print(nguoiDungA);
    logFile.print(",");
    logFile.print(nguoiDungB);
    logFile.print(",");
    logFile.print(trangThaiA);
    logFile.print(",");
    logFile.print(trangThaiB);
    logFile.print(",");
    logFile.print(lat);
    logFile.print(",");
    logFile.print(lon);
    logFile.print(",");
    logFile.print(soVeTinh);
    logFile.print(",");
    logFile.print(hdop, 2);
    logFile.print(",");
    logFile.print(vanToc, 1);
    logFile.print(",");
    logFile.print(thoiGianLaiXeLienTuc);
    logFile.print(",");
    logFile.print(dienApNgoVao, 2);
    logFile.print(",");
    logFile.print(pinSlave);
    logFile.print(",");
    logFile.println(canhBao);
    logFile.close();
    Serial.println("Dữ liệu đã được ghi vào log.csv.");
  } else {
    Serial.println("Không thể ghi vào file log.csv!");
  }
}
