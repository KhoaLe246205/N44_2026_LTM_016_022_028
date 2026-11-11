#!/usr/bin/env bash
set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$SCRIPT_DIR"

clear
echo "========================================================================"
echo "    ĐỀ TÀI 2: THIẾT KẾ UAV TRẠM CHUYỂN TIẾP SÓNG (FLYING RELAY NODE)    "
echo "        HỆ THỐNG ĐỒNG MÔ PHỎNG: PX4 SITL - GAZEBO - NS-3 AD-HOC        "
echo "========================================================================"
echo "1. Chạy Mô phỏng Mạng ns-3 (Wi-Fi 802.11n + Định tuyến động AODV)"
echo "2. Khởi chạy Mô phỏng Động lực học PX4 Multi-Vehicle + Gazebo (2 UAVs)"
echo "3. Chạy Kịch bản Điều khiển Bay MAVLink Tự động (Python Controller)"
echo "4. Chạy Cầu nối Đồng mô phỏng (Co-simulation Bridge) & Radar Visualizer"
echo "5. Xuất Đồ thị Báo cáo Hiệu năng Mạng (PDR, Latency, Hop Count, Distance)"
echo "6. CHẠY TOÀN BỘ DỰ ÁN TỰ ĐỘNG (Full Integrated Pipeline Demo)"
echo "0. Thoát"
echo "========================================================================"
echo -n "Vui lòng chọn chức năng [0-6]: "
read -r choice

case "$choice" in
    1)
        echo "[+] Đang chạy mô phỏng mạng ns-3..."
        bash "$SCRIPT_DIR/ns3_sim/run_ns3.sh"
        ;;
    2)
        echo "[+] Đang khởi động PX4 Multi-Vehicle SITL + Gazebo..."
        bash "$SCRIPT_DIR/px4_gazebo/launch_sitl_multi.sh"
        ;;
    3)
        echo "[+] Đang chạy kịch bản điều khiển bay PX4..."
        python3 "$SCRIPT_DIR/px4_gazebo/px4_multi_vehicle.py"
        ;;
    4)
        echo "[+] Khởi chạy Co-simulation Bridge và Live Visualizer..."
        python3 "$SCRIPT_DIR/co_simulation/live_visualizer.py"
        ;;
    5)
        echo "[+] Đang xử lý và xuất đồ thị phân tích..."
        python3 "$SCRIPT_DIR/co_simulation/plot_results.py"
        echo "[✓] Kiểm tra file: results/uav_relay_performance_charts.png"
        ;;
    6)
        echo "=================================================================="
        echo "  BẮT ĐẦU CHẠY THỬ NGHIỆM TỔNG HỢP (FULL INTEGRATED DEMO)"
        echo "=================================================================="
        python3 "$SCRIPT_DIR/co_simulation/relay_network_bridge.py" &
        BRIDGE_PID=$!
        sleep 1

        python3 "$SCRIPT_DIR/px4_gazebo/px4_multi_vehicle.py" --duration 55.0

        kill $BRIDGE_PID 2>/dev/null || true

        python3 "$SCRIPT_DIR/co_simulation/plot_results.py"

        echo "[✓] Hoàn tất toàn bộ chu trình thử nghiệm!"
        ;;
    0)
        echo "Tạm biệt!"
        exit 0
        ;;
    *)
        echo "Lựa chọn không hợp lệ."
        exit 1
        ;;
esac
