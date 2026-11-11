#!/usr/bin/env bash
set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$SCRIPT_DIR"

echo "=========================================================================="
echo "   TỰ ĐỘNG KHỞI CHẠY HỆ THỐNG ĐỒNG MÔ PHỎNG (3 TERMINAL ĐỘC LẬP)          "
echo "=========================================================================="
echo " Hệ thống sẽ tự động mở 3 cửa sổ Terminal trên Ubuntu:"
echo "   [Terminal 1]: Môi trường vật lý 3D Gazebo + 2 Bộ điều khiển bay PX4 SITL"
echo "   [Terminal 2]: Bộ điều khiển bay MAVLink Python (Cổng UDP 14540 & 14541)"
echo "   [Terminal 3]: Mô phỏng mạng không dây Ad-hoc đa chặng ns-3 (AODV)"
echo "--------------------------------------------------------------------------"

if command -v gnome-terminal &> /dev/null; then
    # Terminal 1: PX4 Multi-Vehicle SITL + Gazebo
    gnome-terminal --title="[1] PX4 SITL & Gazebo 3D" -- bash -c "
        echo '>>> [TERMINAL 1] Khởi động PX4 Multi-Vehicle SITL & Gazebo 3D...';
        bash '$SCRIPT_DIR/px4_gazebo/launch_sitl_multi.sh';
        exec bash
    "

    echo "[+] Đã mở Terminal 1 (PX4 SITL & Gazebo). Đang đợi Gazebo tải mô hình (10s)..."
    sleep 10

    # Terminal 2: Điều khiển bay MAVLink
    gnome-terminal --title="[2] MAVLink Mission Controller" -- bash -c "
        echo '>>> [TERMINAL 2] Khởi động Bộ điều khiển bay MAVLink...';
        python3 '$SCRIPT_DIR/px4_gazebo/px4_multi_vehicle.py' --duration 60.0;
        echo 'Nhấn phím bất kỳ để đóng...';
        read -n 1;
        exec bash
    "

    echo "[+] Đã mở Terminal 2 (MAVLink Flight Controller)."
    sleep 2

    # Terminal 3: Mô phỏng mạng ns-3 và Radar Visualizer
    gnome-terminal --title="[3] ns-3 Ad-hoc Network & Visualizer" -- bash -c "
        echo '>>> [TERMINAL 3] Khởi động Mô phỏng Mạng ns-3 (Wi-Fi 802.11n + AODV)...';
        bash '$SCRIPT_DIR/ns3_sim/run_ns3.sh';
        echo '';
        echo '>>> Khởi động Radar Visualizer hiển thị chuyển tiếp sóng...';
        python3 '$SCRIPT_DIR/co_simulation/live_visualizer.py';
        exec bash
    "

    echo "[+] Đã mở Terminal 3 (ns-3 Network Simulation & Visualizer)."
    echo "=========================================================================="
    echo "[✓] CẢ 3 TIẾN TRÌNH ĐANG CHẠY ĐỒNG THỜI!"
    echo "=========================================================================="

else
    echo "[!] Không phát hiện gnome-terminal. Chạy theo chế độ tiến trình nền (Background Processes):"
    
    python3 "$SCRIPT_DIR/co_simulation/relay_network_bridge.py" &
    PID_BRIDGE=$!
    sleep 1

    python3 "$SCRIPT_DIR/px4_gazebo/px4_multi_vehicle.py" --mock --duration 50.0 &
    PID_FLIGHT=$!

    bash "$SCRIPT_DIR/ns3_sim/run_ns3.sh"

    wait $PID_FLIGHT
    kill $PID_BRIDGE 2>/dev/null || true

    python3 "$SCRIPT_DIR/co_simulation/plot_results.py"
fi
