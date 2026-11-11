#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import sys
import math
import time
from pathlib import Path

if sys.platform.startswith('win'):
    sys.stdout.reconfigure(encoding='utf-8', errors='replace')
    sys.stderr.reconfigure(encoding='utf-8', errors='replace')

import matplotlib
matplotlib.rcParams['font.sans-serif'] = ['DejaVu Sans', 'Liberation Sans', 'Ubuntu', 'Arial', 'sans-serif']
matplotlib.rcParams['font.family'] = 'sans-serif'
matplotlib.rcParams['axes.unicode_minus'] = False
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
import matplotlib.patches as patches

# Bán kính phủ sóng vô tuyến Wi-Fi (m)
RADIO_RANGE = 300.0
UAV1_POS = (250.0, 0.0)
SPEED = 10.0
TOTAL_TIME = 60.0

fig, ax = plt.subplots(figsize=(13, 7.5), dpi=120)
fig.canvas.manager.set_window_title("Mo Phong Truc Quan Mang UAV Chuyen Tiep Song (AODV Relay)")

def update(frame):
    ax.clear()
    sim_time = frame * 0.5
    if sim_time > TOTAL_TIME:
        sim_time = TOTAL_TIME

    x2 = min(SPEED * sim_time, 500.0)
    uav2_pos = (x2, 0.0)
    gcs_pos = (0.0, 0.0)

    # UAV 1 bám trung điểm động giữa GCS và UAV 2, chốt tối đa tại 250m
    x1 = min(x2 / 2.0, 250.0)
    uav1_pos = (x1, 0.0)

    d_0_2 = math.sqrt(x2**2)
    d_0_1 = math.sqrt(x1**2)
    d_1_2 = math.sqrt((x2 - x1)**2)

    circle_gcs = patches.Circle(gcs_pos, RADIO_RANGE, edgecolor='green', facecolor='lightgreen',
                                alpha=0.2, linestyle='--', linewidth=1.5, label='Vung phu GCS (R=300m)')
    ax.add_patch(circle_gcs)

    circle_relay = patches.Circle(uav1_pos, RADIO_RANGE, edgecolor='blue', facecolor='lightblue',
                                  alpha=0.2, linestyle='--', linewidth=1.5, label='Vung phu UAV 1 Relay (R=300m)')
    ax.add_patch(circle_relay)

    ax.axvline(x=300, color='darkred', linestyle=':', linewidth=1.8, alpha=0.8, label='Ranh gioi song truc tiep (300m)')

    ax.plot(0, 0, marker='s', markersize=14, color='green', markeredgecolor='black', label='GCS: Tram Mat Dat (0, 0)')
    ax.plot(uav1_pos[0], uav1_pos[1], marker='^', markersize=14, color='blue', markeredgecolor='black', label=f'UAV 1: Dynamic Relay ({x1:.0f}m, 0)')
    ax.plot(uav2_pos[0], uav2_pos[1], marker='o', markersize=12, color='red', markeredgecolor='black', label=f'UAV 2: Recon Node ({x2:.0f}m, 0)')
    ax.plot([0, 500], [0, 0], color='gray', linestyle=':', linewidth=1.2, zorder=1)

    # Xử lý trạng thái định tuyến: truyền thẳng 1 chặng hoặc chuyển tiếp 2 chặng
    if d_0_2 <= RADIO_RANGE:
        ax.plot([0, x2], [0, 0], color='green', linewidth=3.5, alpha=0.85, label='Link truc tiep: GCS <-> Recon (1-Hop)')
        status_text = "TRUC TIEP (DIRECT 1-HOP)"
        status_color = "green"
        hop_count = 1
        latency = 3.5 + (d_0_2 / 300.0) * 2.0
        pdr = 99.4
    else:
        # Khoảng cách vượt ngưỡng phủ sóng trực tiếp: ngắt kết nối trực tiếp và chuyển mạch qua relay
        ax.plot([0, 300], [0, 0], color='red', linewidth=1.5, linestyle=':', alpha=0.5)
        ax.scatter([300], [0], color='red', marker='x', s=150, linewidths=3, zorder=5, label='Dut song truc tiep (>300m)')

        ax.plot([0, uav1_pos[0]], [0, uav1_pos[1]], color='blue', linewidth=3.0, linestyle='-', alpha=0.9, label='Chang 1: GCS -> UAV 1')
        ax.plot([uav1_pos[0], x2], [uav1_pos[1], 0], color='cyan', linewidth=3.0, linestyle='-', alpha=0.9, label='Chang 2: UAV 1 -> Recon')
        status_text = "CHUYEN TIEP RELAY (AODV 2-HOPS)"
        status_color = "blue"
        hop_count = 2
        latency = 8.5 + (d_1_2 / 300.0) * 4.0
        pdr = 98.6

    hud_text = (
        f"THOI GIAN (TIME): {sim_time:4.1f}s / {TOTAL_TIME:.0f}s\n"
        f"TOA DO UAV 2: X = {x2:5.1f}m, Y = 0m, Z = 30m\n"
        f"KHOANG CACH GCS -> UAV 2: {d_0_2:5.1f}m (Nguong: 300m)\n"
        f"KHOANG CACH RELAY -> UAV 2: {d_1_2:5.1f}m\n"
        f"TRANG THAI MANG: {status_text}\n"
        f"SO CHANG (HOP COUNT): {hop_count} HOP(S)\n"
        f"GIAO THUC DINH TUYEN: AODV Dynamic Routing\n"
        f"TY LE NHAN GOI (PDR): {pdr:.1f}%\n"
        f"DO TRE TRUYEN DAN (LATENCY): {latency:.1f} ms"
    )
    ax.text(0.02, 0.96, hud_text, transform=ax.transAxes, fontsize=10,
            verticalalignment='top', family='monospace',
            bbox=dict(boxstyle='round,pad=0.6', facecolor='white', edgecolor=status_color, linewidth=2, alpha=0.95))

    ax.set_title("HE THONG MO PHONG UAV CHUYEN TIEP SONG AD-HOC (PX4 - GAZEBO - NS-3)",
                 fontsize=13, fontweight='bold', pad=12)
    ax.set_xlabel("Truc Toa Do X (met)", fontweight='bold')
    ax.set_ylabel("Truc Toa Do Y (met)", fontweight='bold')
    ax.set_xlim(-150, 650)
    ax.set_ylim(-380, 380)
    ax.set_aspect('equal')
    ax.grid(True, linestyle=':', alpha=0.6)
    ax.legend(loc='lower right', fontsize=8.5, framealpha=0.9)

def main():
    print("[+] Khoi chay giao dien truc quan hoa thoi gian thuc (Real-time Visualizer)...")
    anim = FuncAnimation(fig, update, frames=120, interval=150, repeat=True)
    plt.show()

if __name__ == "__main__":
    main()
