#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import sys
import time
import math
import socket
import json
from pathlib import Path

if sys.platform.startswith('win'):
    sys.stdout.reconfigure(encoding='utf-8', errors='replace')
    sys.stderr.reconfigure(encoding='utf-8', errors='replace')

def main():
    results_dir = Path(__file__).resolve().parent.parent / "results"
    results_dir.mkdir(parents=True, exist_ok=True)
    metrics_csv = results_dir / "bridge_live_metrics.csv"

    rx_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    rx_sock.bind(("127.0.0.1", 19999))
    rx_sock.settimeout(1.0)

    print("==================================================================")
    print("      CO-SIMULATION NETWORK BRIDGE (AODV & WI-FI FORWARDING)      ")
    print("==================================================================")
    print(" - Lắng nghe tọa độ Drone từ PX4 SITL / Gazebo tại UDP 19999")
    print(" - Ngưỡng phát sóng vô tuyến Wi-Fi: R = 300.0 m")
    print(" - Tọa độ trạm mặt đất GCS:         (0, 0, 0)")
    print(" - Giao thức định tuyến:             AODV Dynamic Routing")
    print("------------------------------------------------------------------")

    tx_total = 0
    rx_total = 0
    direct_rx = 0
    relayed_rx = 0

    with open(metrics_csv, "w", encoding="utf-8") as f:
        f.write("time_sec,uav2_x,dist_gcs_recon,dist_relay_recon,route_state,hop_count,latency_ms,pdr\n")

        print("[+] Sẵn sàng nhận telemetry và giả lập định tuyến mạng...")
        try:
            while True:
                try:
                    data, _ = rx_sock.recvfrom(4096)
                    info = json.loads(data.decode('utf-8'))
                except socket.timeout:
                    continue

                sim_time = info["time"]
                uav1_pos = info["uav1"]
                uav2_pos = info["uav2"]
                dist_0_2 = info["dist_gcs_recon"]
                dist_1_2 = info["dist_relay_recon"]

                tx_total += 1

                # Kiểm tra vùng phủ sóng vô tuyến (R = 300m)
                RADIO_RANGE = 300.0
                direct_in_range = (dist_0_2 <= RADIO_RANGE)
                relay_in_range = (dist_1_2 <= RADIO_RANGE)

                # Tuyến trực tiếp 1 chặng khi trong tầm sóng; chuyển tiếp qua relay khi vượt ngưỡng
                if direct_in_range:
                    route_state = "DIRECT"
                    hop_count = 1
                    latency = 3.5 + (dist_0_2 / 300.0) * 2.0
                    rx_total += 1
                    direct_rx += 1
                elif relay_in_range:
                    route_state = "RELAY_FORWARD"
                    hop_count = 2
                    latency = 8.5 + (dist_1_2 / 300.0) * 4.0
                    rx_total += 1
                    relayed_rx += 1
                else:
                    route_state = "OUT_OF_RANGE"
                    hop_count = 0
                    latency = 0.0

                pdr = (rx_total / tx_total) * 100.0

                f.write(f"{sim_time:.2f},{uav2_pos['x']:.2f},{dist_0_2:.2f},{dist_1_2:.2f},"
                        f"{route_state},{hop_count},{latency:.2f},{pdr:.1f}\n")
                f.flush()

                if route_state == "DIRECT":
                    badge = "\033[92m[TRỰC TIẾP (1-HOP)]\033[0m"
                    link_repr = "GCS ============> UAV 2 (Trinh sát)"
                elif route_state == "RELAY_FORWARD":
                    badge = "\033[94m[BẮC CẦU RELAY (2-HOP)]\033[0m"
                    link_repr = f"GCS ---> UAV 1 (Dynamic Relay: {uav1_pos['x']:.0f}m) ---> UAV 2 (Trinh sát)"
                else:
                    badge = "\033[91m[MẤT KẾT NỐI]\033[0m"
                    link_repr = "GCS ---X--- UAV 2"

                sys.stdout.write(
                    f"\rT={sim_time:4.1f}s | Dist: {dist_0_2:5.1f}m | {badge} | {link_repr} | Delay={latency:4.1f}ms | PDR={pdr:5.1f}%  "
                )
                sys.stdout.flush()

        except KeyboardInterrupt:
            print("\n[!] Dừng Co-Simulation Bridge.")

    print(f"\n[✓] Số liệu hiệu năng đã ghi vào: {metrics_csv}")
    print(f"Tổng kết: Tx={tx_total}, Rx={rx_total} (Direct={direct_rx}, Relayed={relayed_rx})")

if __name__ == "__main__":
    main()
