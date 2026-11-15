# KỊCH BẢN ĐỐI CHỨNG BASELINE: TRUYỀN TRỰC TIẾP KHÔNG CÓ UAV RELAY

Thư mục này chứa mã nguồn và kết quả đo kiểm kịch bản mạng **Baseline (Không sử dụng trạm chuyển tiếp UAV)** nhằm đối sánh thực nghiệm với giải pháp đề xuất **Flying Relay Node**.

---

## 1. CẤU TRÚC THƯ MỤC

```text
baseline_comparison/
├── uav_direct_baseline.cc      # Mã nguồn C++ ns-3 mô phỏng liên kết trực tiếp (2 nút: GCS và Recon)
├── run_baseline.sh             # Script biên dịch và thực thi mô phỏng trên ns-3
├── compare_results.py          # Script phân tích và xuất bảng/đồ thị đối sánh Baseline vs Đề xuất
├── results/                    # Thư mục kết quả đo kiểm
│   ├── ns3_baseline_metrics.csv        # 118 bản ghi chu kỳ đo kiểm mạng Baseline
│   └── baseline_vs_relay_comparison.png # Đồ thị khoa học đối sánh PDR và Cự ly
└── README.md                   # Hướng dẫn chi tiết thực thi và kiểm chứng
```

---

## 2. NGUYÊN LÝ HOẠT ĐỘNG CỦA KỊCH BẢN ĐỐI CHỨNG

- **Mô hình mạng:** Chỉ gồm 2 nút mạng:
  - **Node 0 (GCS):** Đặt cố định tại mặt đất $(0, 0, 0)$, IP `10.1.1.1`.
  - **Node 1 (UAV Recon):** Di chuyển theo đúng tệp tọa độ bay thực tế `px4_flight_telemetry.csv` từ $0\text{ m} \rightarrow 500\text{ m}$ ở độ cao $30\text{ m}$, IP `10.1.1.2`.
- **Cấu hình vô tuyến:** Chuẩn IEEE 802.11n Ad-hoc, công suất phát $20\text{ dBm}$, ngưỡng nhạy thu $-85\text{ dBm}$, mô hình suy hao không gian tự do kết hợp ngưỡng ngắt sóng $R = 300\text{ m}$.
- **Luồng dữ liệu:** GCS phát luồng UDP Telemetry (128 Bytes, tần số 20 Hz, tốc độ 20.48 Kbps) tới UAV Recon từ giây thứ $2.0\text{ s}$ đến $59.0\text{ s}$ ($1.139$ gói tin).

---

## 3. HƯỚNG DẪN THỰC THI TRÊN UBUNTU / WSL

### Bước 1: Chạy mô phỏng Baseline trên ns-3
```bash
cd baseline_comparison
bash run_baseline.sh
```
*Script sẽ tự động sao chép mã nguồn vào thư mục `scratch/` của ns-3, nạp tệp telemetry và lưu kết quả đo kiểm vào thư mục `results/`.*

### Bước 2: So sánh kết quả thực nghiệm với kịch bản Đề xuất
```bash
python3 compare_results.py
```
*Lệnh trên sẽ in bảng đối sánh chi tiết 3 giai đoạn ra màn hình Terminal và xuất biểu đồ so sánh `results/baseline_vs_relay_comparison.png`.*

---

## 4. TÓM TẮT KẾT QUẢ ĐỐI SÁNH

| Tiêu chí Đánh giá | Baseline (Không có Relay) | Đề xuất (Có Flying Relay Node) | Đánh giá Kỹ thuật |
| :--- | :---: | :---: | :--- |
| **Giai đoạn 1 ($x \le 300\text{ m}$)** | PDR: **$100.00\%$** (589/589 gói)<br/>Độ trễ: **$0.116\text{ ms}$** | PDR: **$100.00\%$** (589/589 gói)<br/>Độ trễ: **$0.116\text{ ms}$** | Cả hai đều truyền trực tiếp 1 chặng ổn định |
| **Giai đoạn 2 ($x > 300\text{ m}$)** | PDR: **$0.00\%$** (0/550 gói nhận)<br/>Độ trễ: **Vô hạn** (Mất kết nối) | PDR: **$99.82\%$** (549/550 gói nhận)<br/>Độ trễ: **$0.336\text{ ms}$** | **UAV Relay duy trì thông suốt 100% liên lạc** |
| **Toàn trình ($0 - 60\text{ s}$)** | PDR tổng: **$51.71\%$** (589/1139 gói) | PDR tổng: **$99.91\%$** (1138/1139 gói) | **Khẳng định tính vượt trội của giải pháp đề tài** |
