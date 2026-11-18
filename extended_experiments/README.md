# HƯỚNG DẪN THỰC HIỆN THỰC NGHIỆM MÔ PHỎNG BỔ SUNG (NS-3)

Thư mục này chứa toàn bộ mã nguồn, kịch bản tự động hóa và công cụ phân tích phục vụ cho 2 phần thực nghiệm mở rộng theo yêu cầu đánh giá:
1. **Phân tích độ nhạy của hệ thống theo bán kính cắt sóng vô tuyến $R_{cutoff}$ ($250\text{ m}, 300\text{ m}, 350\text{ m}$)**
2. **Đo kiểm đối chứng giữa giao thức định tuyến phản ứng AODV (RFC 3561) và định tuyến chủ động OLSR (RFC 3626)**

---

## 1. Cấu trúc thư mục

```
extended_experiments/
├── uav_relay_olsr.cc              # Mã nguồn mô phỏng ns-3 với giao thức OLSR
├── run_extended_experiments.sh     # Script tự động biên dịch và chạy 3 bài test
├── plot_and_compare_extended.py   # Script trích xuất số liệu và vẽ đồ thị đối sánh
├── README.md                      # Hướng dẫn chi tiết các bước thực hiện
└── results/                       # Thư mục lưu kết quả CSV, PCAP và biểu đồ
    ├── ns3_baseline_r300_metrics.csv
    ├── ns3_sensitivity_r250_metrics.csv
    ├── ns3_sensitivity_r350_metrics.csv
    ├── ns3_olsr_metrics.csv
    ├── extended_sensitivity_r_cutoff.png
    └── extended_protocol_aodv_vs_olsr.png
```

---

## 2. Hướng dẫn chạy thực nghiệm trong máy ảo VMware Ubuntu

### Bước 1: Mở Terminal trong Ubuntu và truy cập thư mục dự án
Nếu bạn dùng tính năng VMware Shared Folders (`/mnt/hgfs/`):
```bash
cd "/mnt/hgfs/LT Mạng/UAV_Relay_Project/extended_experiments"
# Hoặc đường dẫn tương ứng trên máy ảo của bạn
```

### Bước 2: Cấp quyền thực thi và khởi chạy script tự động
```bash
chmod +x run_extended_experiments.sh
./run_extended_experiments.sh
```

Script sẽ tự động:
1. Tìm kiếm thư mục cài đặt `ns-3` trên máy ảo.
2. Sao chép `uav_relay_aodv.cc` và `uav_relay_olsr.cc` vào thư mục `scratch/` của ns-3.
3. Thực thi lần lượt 3 kịch bản mô phỏng:
   - **Kịch bản 1:** Độ nhạy $R_{cutoff} = 250\text{ m}$ (AODV).
   - **Kịch bản 2:** Độ nhạy $R_{cutoff} = 350\text{ m}$ (AODV).
   - **Kịch bản 3:** Giao thức định tuyến OLSR ($R_{cutoff} = 300\text{ m}$).
4. Tự động thu thập toàn bộ các file `.csv` kết quả và gói bắt `.pcap` lưu về thư mục `results/`.

*(Thời gian thực thi toàn bộ 3 kịch bản: khoảng 30–60 giây).*

---

## 3. Trích xuất số liệu và tạo đồ thị đối sánh

Sau khi chạy xong, bạn có thể thực thi script phân tích ngay trên Windows hoặc trên Ubuntu:

```bash
python plot_and_compare_extended.py
```

Lệnh này sẽ:
1. In bảng tổng hợp số liệu đo kiểm thực tế (Số gói Tx/Rx, PDR %, Độ trễ trung bình, Mốc thời gian chuyển tuyến).
2. Tự động xuất 2 tệp đồ thị độ phân giải cao (300 DPI):
   - `results/extended_sensitivity_r_cutoff.png`: Đồ thị so sánh PDR và trễ theo $R_{cutoff}$.
   - `results/extended_protocol_aodv_vs_olsr.png`: Đồ thị so sánh hiệu năng AODV vs OLSR.

---

## 4. Hướng dẫn chụp ảnh minh chứng cho báo cáo / thuyết minh

Để có minh chứng trực quan, thuyết phục nhất cho báo cáo, bạn hãy chụp 2 ảnh màn hình sau:

1. **Ảnh 1: Cửa sổ Terminal hoàn thành 3 kịch bản:**
   - Chụp màn hình Terminal khi lệnh `./run_extended_experiments.sh` in ra các dòng `[✓] Hoan thanh thi nghiem...` và bảng kết quả FlowMonitor.
2. **Ảnh 2: Tệp bắt gói Wireshark của OLSR:**
   - Mở file bắt gói `results/pcap_olsr/uav-relay-olsr-0-0.pcap` bằng Wireshark.
   - Nhập bộ lọc `olsr` trên thanh tìm kiếm để thấy rõ các bản tin `HELLO` và `TC` định kỳ được gửi trên cổng UDP 698.
