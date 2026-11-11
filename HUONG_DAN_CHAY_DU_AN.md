# HƯỚNG DẪN THỰC HIỆN VÀ CHẠY THỬ NGHIỆM DỰ ÁN
## ĐỀ TÀI: THIẾT KẾ UAV TRẠM CHUYỂN TIẾP SÓNG (FLYING RELAY NODE)
**Hệ thống đồng mô phỏng 3 công cụ: PX4 Autopilot SITL + Gazebo 3D + ns-3 Ad-hoc Network**

---

### I. CẤU TRÚC THƯ MỤC DỰ ÁN
Toàn bộ mã nguồn và tệp cấu hình dự án được lưu tại:
`D:\LT Mạng\UAV_Relay_Project\`

```text
UAV_Relay_Project/
├── ns3_sim/
│   ├── uav_relay_aodv.cc         # Mã nguồn mô phỏng mạng Wi-Fi 802.11n + AODV bằng C++
│   └── run_ns3.sh                # Script tự động tìm ns-3, biên dịch và chạy mô phỏng
├── px4_gazebo/
│   ├── launch_sitl_multi.sh      # Script khởi chạy 2 drone trong Gazebo (SITL Multi-Vehicle)
│   └── px4_multi_vehicle.py      # Bộ điều khiển bay MAVLink UDP (Cổng 14540 và 14541)
├── co_simulation/
│   ├── relay_network_bridge.py   # Cầu nối đồng bộ thời gian thực PX4 -> ns-3 (Forwarding Engine)
│   ├── live_visualizer.py        # Giao diện Radar 2D động hiển thị quá trình chuyển tiếp sóng
│   └── plot_results.py           # Bộ tạo 4 đồ thị đánh giá hiệu năng đạt chuẩn báo cáo
├── results/                      # Thư mục lưu kết quả mô phỏng (CSV, đồ thị PNG, PCAP, NetAnim XML)
│   └── uav_relay_performance_charts.png
├── run_full_demo.sh              # Menu điều khiển chạy toàn bộ hệ thống trên Linux/Ubuntu
├── run_demo_windows.bat          # Menu chạy thử nghiệm nhanh trực tiếp trên Windows
├── HUONG_DAN_CHAY_DU_AN.md       # Tài liệu hướng dẫn thực hành từng bước (File này)
└── BAO_CAO_THIET_KE_DU_AN.md      # Báo cáo kỹ thuật chi tiết nộp giảng viên
```

---

### II. TRUY CẬP DỰ ÁN TỪ MÁY ẢO UBUNTU TRÊN VMWARE

Thư mục `D:\LT Mạng` trên máy thật Windows đã được kết nối chia sẻ (VMware Shared Folders) sang Ubuntu.

1. Khởi động máy ảo Ubuntu 22.04 trên VMware Workstation.
2. Mở cửa sổ **Terminal** trong Ubuntu (`Ctrl + Alt + T`).
3. Truy cập vào thư mục dự án bằng lệnh:
```bash
cd /mnt/hgfs/LT_Mang/UAV_Relay_Project
ls -la
```
*(Nếu chưa thấy `/mnt/hgfs/LT_Mang`, bạn chạy lệnh gắn ổ đĩa chia sẻ: `sudo vmhgfs-fuse .host:/ /mnt/hgfs -o allow_other`)*.

---

### III. CÁC BƯỚC CHẠY THỬ NGHIỆM CHI TIẾT

#### BƯỚC 1: Chạy mô phỏng mạng ns-3 (Wi-Fi 802.11n & AODV Multi-hop)
Mô phỏng này chứng minh chính xác logic mạng của đề tài:
- **Node 0 (GCS)** ở tọa độ (0, 0, 0).
- **Node 1 (Relay UAV)** ở (250m, 0, 30m).
- **Node 2 (Recon UAV)** bay từ 0m đến 500m (v = 10 m/s).
- Tầm phát sóng Wi-Fi là 300m. Khi Node 2 vượt quá 300m, sóng trực tiếp từ GCS đứt, giao thức **AODV** tự động tìm đường và chuyển tiếp gói tin qua Node 1!

**Thực hiện:**
```bash
cd /mnt/hgfs/LT_Mang/UAV_Relay_Project/ns3_sim
chmod +x run_ns3.sh
./run_ns3.sh
```

**Quan sát trên màn hình:**
- Từ $T = 0s \rightarrow 30s$ (cự ly $< 300m$): Chế độ `DIRECT (1-hop)` - Gói tin đi thẳng GCS $\rightarrow$ Recon.
- Từ $T = 30s \rightarrow 50s$ (cự ly $300m \rightarrow 500m$): Tự động chuyển `RELAY via UAV 1 (2-hop)` - Gói tin bắc cầu GCS $\rightarrow$ UAV 1 $\rightarrow$ UAV 2.
- Kết thúc mô phỏng, FlowMonitor sẽ in bảng thống kê: PDR đạt **~98-99%** (nếu không có Relay node thì sau 300m PDR sẽ rơi về 0%).
- Tệp hoạt họa `uav_relay_animation.xml` và các tệp bắt gói tin Wireshark `uav-relay-node-*.pcap` được tạo tự động trong thư mục `results/`.

*(Xem hoạt họa NetAnim nếu có: `cd ~/ns-allinone-3.*/netanim* && ./NetAnim` rồi mở file `uav_relay_animation.xml`)*.

---

#### BƯỚC 2: Khởi chạy 2 Drone trên PX4 SITL & Môi trường 3D Gazebo
Mô phỏng động lực học bay vật lý thực tế của 2 chiếc Drone Iris Quadrotor:
- Drone 1 (Relay): mở cổng MAVLink UDP `14540`.
- Drone 2 (Recon): mở cổng MAVLink UDP `14541`.

**Thực hiện:**
Mở Terminal mới trên Ubuntu:
```bash
cd /mnt/hgfs/LT_Mang/UAV_Relay_Project/px4_gazebo
chmod +x launch_sitl_multi.sh
./launch_sitl_multi.sh
```
Gazebo sẽ mở ra một thế giới 3D với 2 chiếc drone iris đặt cạnh nhau trên mặt đất.

---

#### BƯỚC 3: Chạy kịch bản điều khiển bay tự động (Autonomous Mission)
Bộ điều khiển bay viết bằng Python kết nối đồng thời tới 2 cổng MAVLink:
- Gửi lệnh cất cánh cho cả 2 drone lên độ cao 30m.
- Ra lệnh cho UAV 1 bay tới tọa độ trung điểm `(x=250m, y=0, z=30m)` và lơ lửng giữ vị trí (Relay Station).
- Ra lệnh cho UAV 2 bay thẳng theo trục X tới mốc `(x=500m, y=0, z=30m)` với tốc độ 10 m/s.
- Liên tục đọc tọa độ thực $(x_1, y_1, z_1)$ và $(x_2, y_2, z_2)$ xuất ra màn hình và file `results/px4_flight_telemetry.csv`.

**Thực hiện:**
Mở Terminal thứ 3 trên Ubuntu:
```bash
cd /mnt/hgfs/LT_Mang/UAV_Relay_Project/px4_gazebo
python3 px4_multi_vehicle.py
```
*(Nếu muốn chạy thử nghiệm kịch bản bay độc lập mà không cần mở Gazebo 3D để tiết kiệm RAM, bạn chỉ cần thêm cờ `--mock`: `python3 px4_multi_vehicle.py --mock`)*.

---

#### BƯỚC 4: Chạy Cầu nối Đồng mô phỏng (Co-Simulation Bridge) & Radar Visualizer
Module này đóng vai trò cầu nối: lấy tọa độ bay thực tế từ Gazebo/PX4 và đồng bộ vào mạng không dây để kiểm thử chuyển tiếp gói tin theo thời gian thực:

```bash
cd /mnt/hgfs/LT_Mang/UAV_Relay_Project/co_simulation
python3 live_visualizer.py
```
Một cửa sổ Radar 2D tuyệt đẹp sẽ hiện lên:
- Vùng tròn xanh lá: Vùng phủ sóng trực tiếp của Trạm mặt đất (300m).
- Vùng tròn xanh dương: Vùng phủ sóng của UAV 1 Relay (mở rộng vùng sóng lên tới 550m).
- Khi UAV 2 bay vượt qua 300m, đường truyền trực tiếp bị đánh dấu X đỏ (đứt liên lạc), ngay lập tức đường truyền bắc cầu qua UAV 1 sáng lên và duy trì gói tin liên tục!

---

#### BƯỚC 5: Xuất 4 Đồ thị Báo cáo Hiệu năng Khoa học
Chạy lệnh sau (có thể chạy trên Ubuntu hoặc trực tiếp trên Windows):
```bash
cd /mnt/hgfs/LT_Mang/UAV_Relay_Project/co_simulation
python3 plot_results.py
```
File đồ thị chất lượng cao sẽ được xuất ra tại:
`results/uav_relay_performance_charts.png` gồm 4 đồ thị chuẩn mực:
1. Bản đồ bố trí không gian 2D và bán kính phủ sóng vô tuyến.
2. Đồ thị khoảng cách tương đối giữa các node theo thời gian & ranh giới 300m.
3. Đồ thị số chặng định tuyến AODV (1-hop $\rightarrow$ 2-hop) theo vị trí drone.
4. Đồ thị tỷ lệ nhận gói PDR (so sánh có relay vs không có relay) & độ trễ mạng.

---

### IV. CHẠY NHANH BẰNG MENU TỔNG HỢP (ALL-IN-ONE)

Để thuận tiện nhất khi báo cáo trước giảng viên, bạn chỉ cần chạy:
- **Trên Linux/Ubuntu:**
  ```bash
  cd /mnt/hgfs/LT_Mang/UAV_Relay_Project
  chmod +x run_full_demo.sh
  ./run_full_demo.sh
  ```
- **Trên Windows:**
  Nhấp đúp chuột vào file: `D:\LT Mạng\UAV_Relay_Project\run_demo_windows.bat`.
