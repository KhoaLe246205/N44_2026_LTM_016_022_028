#!/usr/bin/env bash
set -e

echo "=========================================================="
echo "   KHỞI ĐỘNG PX4 MULTI-VEHICLE SITL TRÊN GAZEBO (2 DRONES) "
echo "=========================================================="

# Tìm thư mục PX4-Autopilot
PX4_DIR=""
POSSIBLE_PX4_PATHS=(
    "$HOME/PX4-Autopilot"
    "$HOME/Firmware"
    "$HOME/src/PX4-Autopilot"
    "/opt/PX4-Autopilot"
    "$HOME/px4"
)

for dir in "${POSSIBLE_PX4_PATHS[@]}"; do
    if [ -d "$dir" ] && ([ -f "$dir/Makefile" ] || [ -f "$dir/CMakeLists.txt" ]); then
        PX4_DIR="$dir"
        break
    fi
done

if [ -z "$PX4_DIR" ]; then
    echo "[!] Không tìm thấy thư mục PX4-Autopilot tự động."
    echo -n "Vui lòng nhập đường dẫn thư mục PX4-Autopilot (ví dụ: /home/khoale/PX4-Autopilot): "
    read -r PX4_DIR
fi

if [ ! -d "$PX4_DIR" ]; then
    echo "[LỖI] Thư mục '$PX4_DIR' không tồn tại!"
    exit 1
fi

echo "[+] Đã tìm thấy PX4: $PX4_DIR"
cd "$PX4_DIR"

# Kiểm tra script sitl_multiple_run có sẵn trong PX4
MULTI_SCRIPT=""
if [ -f "./Tools/simulation/gazebo-classic/sitl_multiple_run.sh" ]; then
    MULTI_SCRIPT="./Tools/simulation/gazebo-classic/sitl_multiple_run.sh"
    SIM_TYPE="gazebo-classic"
elif [ -f "./Tools/simulation/gz/sitl_multiple_run.sh" ]; then
    MULTI_SCRIPT="./Tools/simulation/gz/sitl_multiple_run.sh"
    SIM_TYPE="gz"
elif [ -f "./Tools/sitl_multiple_run.sh" ]; then
    MULTI_SCRIPT="./Tools/sitl_multiple_run.sh"
    SIM_TYPE="legacy"
fi

if [ -n "$MULTI_SCRIPT" ]; then
    echo "[+] Khởi chạy Multi-Vehicle SITL với 2 drone bằng $MULTI_SCRIPT..."
    echo "[+] Cấu hình cổng MAVLink:"
    echo "    - UAV 1 (Instance 0 / Relay):     UDP 14540"
    echo "    - UAV 2 (Instance 1 / Recon):     UDP 14541"
    echo "    - QGroundControl / GCS MAVLink:   UDP 14550"
    echo "----------------------------------------------------------"
    bash "$MULTI_SCRIPT" -m iris -n 2
else
    echo "[!] Không thấy sitl_multiple_run.sh, tiến hành khởi động thủ công 2 instance..."
    
    DONT_RUN=1 make px4_sitl_default gazebo-classic
    
    source Tools/simulation/gazebo-classic/setup_gazebo.bash $(pwd) $(pwd)/build/px4_sitl_default
    gazebo --verbose Tools/simulation/gazebo-classic/sitl_gazebo-classic/worlds/empty.world &
    GAZEBO_PID=$!
    sleep 5

    # Instance 0: UAV 1 (Relay)
    echo "[+] Khởi động UAV 1 (Relay) trên cổng 14540..."
    PX4_SYS_AUTOSTART=4001 PX4_SIM_MODEL=iris ./build/px4_sitl_default/bin/px4 -i 0 &
    
    # Instance 1: UAV 2 (Recon)
    echo "[+] Khởi động UAV 2 (Recon) trên cổng 14541..."
    PX4_SYS_AUTOSTART=4001 PX4_SIM_MODEL=iris ./build/px4_sitl_default/bin/px4 -i 1 &
    
    wait $GAZEBO_PID
fi
