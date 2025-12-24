# 🏎️ Mô phỏng Xe Tự Hành Dò Đường (ROS 2 Jazzy & Gazebo Harmonic)

Dự án hiện thực hóa thuật toán xe tự lái bám làn đường (Lane Following) và nhận diện vạch đích (Finish Line) sử dụng hệ điều hành Robot **ROS 2** và trình mô phỏng **Gazebo Sim**.

## ✨ Tính năng nổi bật
- **Cơ chế lái Ackermann:** Mô phỏng động học ô tô thực tế với plugin `gz-sim-ackermann-steering-system`.
- **Hệ thống Perception (Thị giác):** Sử dụng OpenCV với bộ lọc HSV và kỹ thuật ROI (Vùng quan tâm) để tách vạch vàng/đỏ ổn định dưới mọi điều kiện ánh sáng.
- **Bộ điều khiển PD (Proportional-Derivative):** Tinh chỉnh hệ số $K_p$ và $K_d$ giúp xe vận hành ổn định ở tốc độ cao mà không bị hiện tượng rung lắc (Rắn bò).
- **Điều tốc thích nghi (Adaptive Speed Control):**
  - **Chạy thẳng:** Tự tin đẩy vận tốc lên **3.5 m/s**.
  - **Vào cua:** Tự động hạ về **2.5 m/s** khi phát hiện độ lệch làn vượt ngưỡng 15px.
  - **Về đích:** Rà phanh về **2.2 m/s** và dừng khựng chính xác khi mũi xe chạm vạch đỏ.
- **Dashboard Giám sát chuyên nghiệp:** 4 màn hình OpenCV thời gian thực (Telemetry, Mask Vàng, Mask Đỏ, Vùng quét ROI).
- **Tính năng Restart nhanh:** Nhấn phím **'R'** trên Dashboard để Teleport xe về vị trí xuất phát ngay lập tức mà không cần reset thế giới.

## 🛠️ Yêu cầu hệ thống
- **Hệ điều hành:** Ubuntu 24.04 (Noble) hoặc 22.04 (Jammy)
- **ROS 2 Version:** Jazzy Jalisco / Humble Hawksbill
- **Simulator:** Gazebo Sim Harmonic
- **Thư viện:** OpenCV, CvBridge, Xacro

## 🚀 Cài đặt và Khởi chạy

### 1. Clone và Build dự án
```bash
cd ~/ros2_ws/src
git clone https://github.com/CAMLC25/Self-driving-car-ROS2.git
cd ..
colcon build --symlink-install --packages-select my_self_driving_bot
source install/setup.bash