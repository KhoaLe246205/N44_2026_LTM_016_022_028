#!/usr/bin/env bash
set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(dirname "$SCRIPT_DIR")"
RESULTS_DIR="$SCRIPT_DIR/results"
TELEMETRY_CSV="$PROJECT_ROOT/results/px4_flight_telemetry.csv"

mkdir -p "$RESULTS_DIR"

echo "=========================================================="
echo "    CHAY MO PHONG BASELINE: KHONG DUNG UAV RELAY"
echo "=========================================================="

# Tim thu muc ns-3
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
    echo -n "Nhap duong dan thu muc ns-3 cua ban: "
    read -r NS3_DIR
fi

if [ ! -d "$NS3_DIR" ]; then
    echo "[LOI] Thu muc '$NS3_DIR' khong ton tai!"
    exit 1
fi

echo "[+] Thu muc ns-3: $NS3_DIR"

# Sao chep source vao scratch va telemetry vao goc ns-3
cp -f "$SCRIPT_DIR/uav_direct_baseline.cc" "$NS3_DIR/scratch/"
if [ -f "$TELEMETRY_CSV" ]; then
    cp -f "$TELEMETRY_CSV" "$NS3_DIR/"
fi

cd "$NS3_DIR"
echo "[+] Bien dich va chay kich ban baseline..."

if [ -f "./ns3" ]; then
    ./ns3 run scratch/uav_direct_baseline
elif [ -f "./waf" ]; then
    ./waf --run scratch/uav_direct_baseline
else
    echo "[LOI] Khong tim thay ./ns3 hoac ./waf!"
    exit 1
fi

# Thu thap ket qua
echo "[+] Thu thap ket qua..."
cp -f ns3_baseline_metrics.csv "$RESULTS_DIR/" 2>/dev/null || true
cp -f uav_baseline_animation.xml "$RESULTS_DIR/" 2>/dev/null || true
cp -f uav-baseline-node-*.pcap "$RESULTS_DIR/" 2>/dev/null || true

echo "=========================================================="
echo "[THANH CONG] Ket qua da duoc luu tai: $RESULTS_DIR"
echo " - File CSV so lieu: $RESULTS_DIR/ns3_baseline_metrics.csv"
echo " - File animation:   $RESULTS_DIR/uav_baseline_animation.xml"
echo " - File PCAP trace:  $RESULTS_DIR/uav-baseline-node-*.pcap"
echo "=========================================================="
