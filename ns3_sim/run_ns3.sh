#!/usr/bin/env bash
set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(dirname "$SCRIPT_DIR")"
RESULTS_DIR="$PROJECT_ROOT/results"
mkdir -p "$RESULTS_DIR"

echo "=========================================================="
echo "    CHẠY MÔ PHỎNG NS-3: THIẾT KẾ UAV TRẠM CHUYỂN TIẾP SÓNG"
echo "=========================================================="

# Tìm thư mục cài đặt ns-3 trong hệ thống
NS3_DIR=""
POSSIBLE_PATHS=(
    "$HOME/ns-allinone-3."*"/ns-3."*
    "$HOME/ns-3."*
    "$HOME/ns-3-dev"
    "/opt/ns-3."*
    "/usr/local/ns-3."*
    "$HOME/tarballs/ns-allinone-3."*"/ns-3."*
    "/mnt/hgfs/LT_Mang/ns-allinone-3."*"/ns-3."*
)

for pattern in "${POSSIBLE_PATHS[@]}"; do
    for dir in $pattern; do
        if [ -d "$dir" ] && ([ -f "$dir/ns3" ] || [ -f "$dir/waf" ]); then
            NS3_DIR="$dir"
            break 2
        fi
    done
done

if [ -z "$NS3_DIR" ]; then
    echo "[!] Không tìm thấy thư mục cài đặt ns-3 tự động."
    echo -n "Vui lòng nhập đường dẫn thư mục ns-3 của bạn (ví dụ: /home/khoale/ns-allinone-3.41/ns-3.41): "
    read -r NS3_DIR
fi

if [ ! -d "$NS3_DIR" ]; then
    echo "[LỖI] Thư mục '$NS3_DIR' không tồn tại!"
    exit 1
fi

echo "[+] Đã xác định thư mục ns-3: $NS3_DIR"

# Đồng bộ mã nguồn và tệp telemetry vào scratch
echo "[+] Sao chép uav_relay_aodv.cc vào $NS3_DIR/scratch/..."
cp "$SCRIPT_DIR/uav_relay_aodv.cc" "$NS3_DIR/scratch/"
if [ -f "$RESULTS_DIR/px4_flight_telemetry.csv" ]; then
    echo "[+] Đồng bộ tệp telemetry PX4 vào $NS3_DIR/..."
    cp -f "$RESULTS_DIR/px4_flight_telemetry.csv" "$NS3_DIR/"
fi

# Biên dịch và thực thi mô phỏng ns-3
cd "$NS3_DIR"
echo "[+] Tiến hành biên dịch và chạy mô phỏng..."

if [ -f "./ns3" ]; then
    ./ns3 run scratch/uav_relay_aodv
elif [ -f "./waf" ]; then
    ./waf --run scratch/uav_relay_aodv
else
    echo "[LỖI] Không tìm thấy trình chạy ./ns3 hoặc ./waf!"
    exit 1
fi

# Sao chép tệp kết quả về thư mục results
echo "[+] Thu thập kết quả mô phỏng..."
cp -f ns3_uav_relay_metrics.csv "$RESULTS_DIR/" 2>/dev/null || true
cp -f uav_relay_animation.xml "$RESULTS_DIR/" 2>/dev/null || true
cp -f uav-relay-node-*.pcap "$RESULTS_DIR/" 2>/dev/null || true

echo "=========================================================="
echo "[THÀNH CÔNG] Mô phỏng NS-3 đã hoàn tất!"
echo "Các tệp kết quả đã được lưu tại: $RESULTS_DIR"
echo " - File số liệu đồ thị:   ns3_uav_relay_metrics.csv"
echo " - File hoạt họa NetAnim: uav_relay_animation.xml"
echo " - File bắt gói tin PCAP: uav-relay-node-*.pcap"
echo "=========================================================="
