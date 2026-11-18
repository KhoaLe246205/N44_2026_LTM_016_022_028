#!/usr/bin/env bash
set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(dirname "$SCRIPT_DIR")"
RESULTS_DIR="$SCRIPT_DIR/results"
mkdir -p "$RESULTS_DIR"
mkdir -p "$RESULTS_DIR/pcap_r250"
mkdir -p "$RESULTS_DIR/pcap_r350"
mkdir -p "$RESULTS_DIR/pcap_olsr"

echo "=========================================================="
echo "   THUC THI MO PHONG BO SUNG: DO NHAY R_CUTOFF VA OLSR   "
echo "=========================================================="

# 1. Tim thu muc cai dat ns-3
NS3_DIR=""
POSSIBLE_PATHS=(
    "$HOME/ns-allinone-3."*"/ns-3."*
    "$HOME/ns-3."*
    "$HOME/ns-3-dev"
    "/opt/ns-3."*
    "/usr/local/ns-3."*
    "$HOME/tarballs/ns-allinone-3."*"/ns-3."*
    "/mnt/hgfs/LT_Mang/ns-allinone-3."*"/ns-3."*
    "/mnt/hgfs/LT Mạng/ns-allinone-3."*"/ns-3."*
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
    echo "[!] Khong tu dong tim thay thu muc ns-3."
    echo -n "Vui long nhap duong dan thu muc ns-3 (vi du: /home/user/ns-allinone-3.41/ns-3.41): "
    read -r NS3_DIR
fi

if [ ! -d "$NS3_DIR" ]; then
    echo "[LOI] Thu muc '$NS3_DIR' khong ton tai!"
    exit 1
fi

echo "[+] Thu muc ns-3: $NS3_DIR"

# 2. Dong bo ma nguon va file viễn trắc telemetry vao scratch
echo "[+] Dong bo ma nguon sang $NS3_DIR/scratch/..."
cp -f "$PROJECT_ROOT/ns3_sim/uav_relay_aodv.cc" "$NS3_DIR/scratch/"
cp -f "$SCRIPT_DIR/uav_relay_olsr.cc" "$NS3_DIR/scratch/"

if [ -f "$PROJECT_ROOT/results/px4_flight_telemetry.csv" ]; then
    echo "[+] Dong bo file telemetry PX4..."
    cp -f "$PROJECT_ROOT/results/px4_flight_telemetry.csv" "$NS3_DIR/"
fi

# Sao chep file ket qua R=300m mac dinh neu da co
if [ -f "$PROJECT_ROOT/results/ns3_uav_relay_metrics.csv" ]; then
    cp -f "$PROJECT_ROOT/results/ns3_uav_relay_metrics.csv" "$RESULTS_DIR/ns3_baseline_r300_metrics.csv"
fi

cd "$NS3_DIR"

RUN_CMD="./ns3 run"
if [ ! -f "./ns3" ] && [ -f "./waf" ]; then
    RUN_CMD="./waf --run"
fi

# ---------------------------------------------------------
# THI NGHIEM 1: Do nhay ban kinh song R_cutoff = 250 m (AODV)
# ---------------------------------------------------------
echo ""
echo ">>> [1/3] DANG CHAY MO PHONG: DO NHAY R_cutoff = 250 m (AODV)..."
$RUN_CMD "scratch/uav_relay_aodv --rangeCutoff=250"
if [ -f "ns3_uav_relay_metrics.csv" ]; then
    cp -f ns3_uav_relay_metrics.csv "$RESULTS_DIR/ns3_sensitivity_r250_metrics.csv"
fi
mv -f uav-relay-node-*.pcap "$RESULTS_DIR/pcap_r250/" 2>/dev/null || true
echo "[✓] Hoan thanh thi nghiem R_cutoff = 250 m!"

# ---------------------------------------------------------
# THI NGHIEM 2: Do nhay ban kinh song R_cutoff = 350 m (AODV)
# ---------------------------------------------------------
echo ""
echo ">>> [2/3] DANG CHAY MO PHONG: DO NHAY R_cutoff = 350 m (AODV)..."
$RUN_CMD "scratch/uav_relay_aodv --rangeCutoff=350"
if [ -f "ns3_uav_relay_metrics.csv" ]; then
    cp -f ns3_uav_relay_metrics.csv "$RESULTS_DIR/ns3_sensitivity_r350_metrics.csv"
fi
mv -f uav-relay-node-*.pcap "$RESULTS_DIR/pcap_r350/" 2>/dev/null || true
echo "[✓] Hoan thanh thi nghiem R_cutoff = 350 m!"

# ---------------------------------------------------------
# THI NGHIEM 3: Doi chung giao thuc dinh tuyen OLSR (R_cutoff = 300 m)
# ---------------------------------------------------------
echo ""
echo ">>> [3/3] DANG CHAY MO PHONG: DOI CHUNG GIAO THUC OLSR (RFC 3626)..."
$RUN_CMD "scratch/uav_relay_olsr --rangeCutoff=300"
if [ -f "ns3_uav_relay_olsr_metrics.csv" ]; then
    cp -f ns3_uav_relay_olsr_metrics.csv "$RESULTS_DIR/ns3_olsr_metrics.csv"
fi
mv -f uav-relay-olsr-*.pcap "$RESULTS_DIR/pcap_olsr/" 2>/dev/null || true
echo "[✓] Hoan thanh thi nghiem OLSR!"

echo ""
echo "=========================================================="
echo "[THANH CONG] Toan bo 3 thi nghiem mo phong da hoan tat!"
echo "Ket qua da duoc luu tai: $RESULTS_DIR"
echo " - Do nhay R=250m: $RESULTS_DIR/ns3_sensitivity_r250_metrics.csv"
echo " - Do nhay R=350m: $RESULTS_DIR/ns3_sensitivity_r350_metrics.csv"
echo " - Giao thuc OLSR: $RESULTS_DIR/ns3_olsr_metrics.csv"
echo "=========================================================="
