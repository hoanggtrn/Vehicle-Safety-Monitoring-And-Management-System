
# Hệ Thống Quản Lý và Giám Sát An Toàn Cho Người Lái Xe

Đồ án tốt nghiệp - Trần Nguyễn Khánh Hoàng  
Khoa Điện - Điện Tử, Trường Đại học Sư phạm Kỹ thuật TP.HCM  
Thời gian: 12/2024

## 📌 Giới thiệu

Hệ thống được thiết kế nhằm hỗ trợ giám sát và nâng cao an toàn cho người điều khiển xe máy thông qua các tính năng theo dõi tình trạng hoạt động và phát hiện nguy hiểm trong quá trình tham gia giao thông. Hệ thống bao gồm bộ trung tâm và bộ cảnh báo ngủ gục với khả năng giao tiếp không dây, xử lý tín hiệu và gửi cảnh báo qua SMS.

## 🎯 Mục tiêu

- Giám sát tốc độ, vị trí GPS, điện áp hệ thống, trạng thái pin.
- Cảnh báo khi phát hiện các tình huống khẩn cấp: va chạm, ngủ gục, vượt quá tốc độ, mất trộm xe.
- Điều khiển khóa/mở khóa từ xa qua tin nhắn SMS.
- Lưu trữ thông tin hoạt động vào thẻ nhớ microSD.
- Giao tiếp không dây giữa các module thông qua ESP-NOW.

## ⚙️ Thành phần chính

- **Vi điều khiển chính:** ESP32-WROOM32
- **Cảm biến:**
  - MPU6050 (gia tốc)
  - Hall sensor (tốc độ)
  - Cảm biến điện áp
- **Module ngoại vi:**
  - SIM A7680C (GSM/GPS)
  - RC522 (RFID)
  - MicroSD module
  - Relay điều khiển khóa xe
- **Giao tiếp:** ESP-NOW, UART, I2C, SPI

## 🛠 Cấu trúc hệ thống

- Bộ trung tâm: xử lý chính, thu thập dữ liệu, gửi cảnh báo SMS.
- Bộ cảnh báo ngủ gục: cảm biến trạng thái đầu, truyền thông với bộ trung tâm.
- Vỏ hộp thiết kế riêng cho từng module.
- PCB tùy chỉnh theo sơ đồ nguyên lý.

  ### 🔧 Hình ảnh thiết kế và thực tế

**PCB Thiết Kế (2D/3D):**  
![PCB Design](https://github.com/hoanggtrn/Vehicle-Safety-Monitoring-And-Management-System/blob/4c5854178aebe95d2d50cff45d7e387fc7b77728/Thi%E1%BA%BFt%20k%E1%BA%BF/B%E1%BB%99%20trung%20t%C3%A2m/M%E1%BA%A1ch%20in/PCB_botrungtam(front).png)
![PCB Design](https://github.com/hoanggtrn/Vehicle-Safety-Monitoring-And-Management-System/blob/4c5854178aebe95d2d50cff45d7e387fc7b77728/Thi%E1%BA%BFt%20k%E1%BA%BF/B%E1%BB%99%20c%E1%BA%A3nh%20b%C3%A1o%20ng%E1%BB%A7%20g%E1%BB%A5c/M%E1%BA%A1ch%20in/PCB_bocanhbaonguguc(front).png)

**Thiết bị thực tế lắp ráp:**  
![PCB Real](https://github.com/hoanggtrn/Vehicle-Safety-Monitoring-And-Management-System/blob/b891f5cc6a181db0935d6e91fd1c9045bce5793c/Thi%E1%BA%BFt%20k%E1%BA%BF/H%C3%ACnh%20%E1%BA%A3nh%20s%E1%BA%A3n%20ph%E1%BA%A9m/P1011317.JPG)
![PCB Real](https://github.com/hoanggtrn/Vehicle-Safety-Monitoring-And-Management-System/blob/f8ecf1a9dfae4f387d521779a2ef8ad7bbb4991f/Thi%E1%BA%BFt%20k%E1%BA%BF/H%C3%ACnh%20%E1%BA%A3nh%20s%E1%BA%A3n%20ph%E1%BA%A9m/P1011246.JPG)
## 📊 Kết quả đạt được

- Hệ thống hoạt động ổn định trong các thử nghiệm thực tế.
- Độ chính xác cao trong việc phát hiện va chạm và vượt tốc.
- Có khả năng mở rộng, nâng cấp thêm nhiều chức năng.
- Gửi và nhận tin nhắn phản hồi chính xác trong thời gian thực.

## 🔍 Giới hạn đề tài

- Chỉ giám sát qua SMS, chưa có kết nối internet/thời gian thực.
- Danh sách người dùng cố định.
- Cảnh báo dựa trên ngưỡng đã thiết lập sẵn.
- Tốc độ giới hạn cứng ở 55 km/h.

## 🚀 Hướng phát triển tương lai

- Thêm giao diện web/app theo dõi trạng thái thời gian thực.
- Tự động học và điều chỉnh ngưỡng cảnh báo thông minh.
- Mở rộng hệ thống nhận dạng người lái xe bằng sinh trắc học.

## 📄 Tài liệu liên quan

Xem toàn bộ báo cáo đồ án tại file `01-TranNguyenKhanhHoang.DATN (final).pdf`.
