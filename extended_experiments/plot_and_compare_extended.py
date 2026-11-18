#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
plot_and_compare_extended.py:
Phan tich va ve bieu do doi sanh ket qua thuc nghiem bo sung:
1. Do nhay theo ban kinh song R_cutoff (250m vs 300m vs 350m)
2. So sanh hieu nang giao thuc dinh tuyen AODV (Reactive) va OLSR (Proactive)
"""

import os
import sys
import csv

if sys.platform.startswith('win'):
    sys.stdout.reconfigure(encoding='utf-8', errors='replace')
    sys.stderr.reconfigure(encoding='utf-8', errors='replace')

import matplotlib
matplotlib.rcParams['font.sans-serif'] = ['DejaVu Sans', 'Arial', 'Segoe UI', 'sans-serif']
matplotlib.rcParams['axes.unicode_minus'] = False
import matplotlib.pyplot as plt

def load_csv(path):
    if not os.path.exists(path):
        return None
    data = []
    with open(path, 'r', encoding='utf-8') as f:
        reader = csv.DictReader(f)
        for row in reader:
            parsed = {}
            for k, v in row.items():
                try:
                    parsed[k] = float(v)
                except:
                    parsed[k] = v
            data.append(parsed)
    return data

def analyze_dataset(data, label):
    if not data:
        return None
    tot_tx = int(data[-1]['tx_pkts'])
    tot_rx = int(data[-1]['rx_pkts'])
    tot_lost = tot_tx - tot_rx
    pdr = (tot_rx / tot_tx) * 100.0 if tot_tx > 0 else 0.0

    delays = [r['avg_delay_ms'] for r in data if r['avg_delay_ms'] > 0]
    avg_delay = sum(delays) / len(delays) if delays else 0.0
    max_delay = max(delays) if delays else 0.0

    # Tim thoi diem bat dau chuyen tiep qua Relay (actual_hops = 2)
    switch_time = None
    for r in data:
        if r['actual_hops'] == 2 and switch_time is None:
            switch_time = r['time_sec']

    return {
        "label": label,
        "tot_tx": tot_tx,
        "tot_rx": tot_rx,
        "tot_lost": tot_lost,
        "pdr": pdr,
        "avg_delay": avg_delay,
        "max_delay": max_delay,
        "switch_time": switch_time
    }

def main():
    base_dir = os.path.dirname(os.path.abspath(__file__))
    res_dir = os.path.join(base_dir, "results")
    project_root = os.path.dirname(base_dir)

    r250_path = os.path.join(res_dir, "ns3_sensitivity_r250_metrics.csv")
    r300_path = os.path.join(res_dir, "ns3_baseline_r300_metrics.csv")
    if not os.path.exists(r300_path):
        r300_path = os.path.join(project_root, "results", "ns3_uav_relay_metrics.csv")
    r350_path = os.path.join(res_dir, "ns3_sensitivity_r350_metrics.csv")
    olsr_path = os.path.join(res_dir, "ns3_olsr_metrics.csv")

    data_r250 = load_csv(r250_path)
    data_r300 = load_csv(r300_path)
    data_r350 = load_csv(r350_path)
    data_olsr = load_csv(olsr_path)

    print("=========================================================================================")
    print("                 TONG HOP KET QUA THUC NGHIEM BO SUNG (NS-3 SIMULATION)                  ")
    print("=========================================================================================")
    print(f"{'Kich ban / Tham so':<32} | {'Tx/Rx (Goi)':<15} | {'PDR (%)':<10} | {'Tre TB (ms)':<12} | {'Moc chuyen tuyen':<15}")
    print("-" * 95)

    datasets = [
        (data_r250, "Do nhay R_cutoff = 250 m (AODV)"),
        (data_r300, "Co so R_cutoff = 300 m (AODV)"),
        (data_r350, "Do nhay R_cutoff = 350 m (AODV)"),
        (data_olsr, "Dinh tuyen chu dong OLSR (300 m)")
    ]

    for d, lbl in datasets:
        stats = analyze_dataset(d, lbl)
        if stats:
            tx_rx_str = f"{stats['tot_tx']} / {stats['tot_rx']}"
            sw_str = f"t = {stats['switch_time']:.1f} s" if stats['switch_time'] else "Khong chuyen"
            print(f"{lbl:<32} | {tx_rx_str:<15} | {stats['pdr']:>7.2f} %  | {stats['avg_delay']:>9.3f} ms | {sw_str:<15}")
        else:
            print(f"{lbl:<32} | {'(Chua co du lieu - hay chay run_extended_experiments.sh)':<60}")
    print("=========================================================================================\n")

    # 1. Ve do thi phan tich do nhay R_cutoff neu co du lieu
    sensitivity_available = (data_r250 is not None and data_r300 is not None and data_r350 is not None)
    if sensitivity_available:
        fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(11, 7), sharex=True)

        t_250 = [r['time_sec'] for r in data_r250]
        pdr_250 = [r['pdr_percent'] for r in data_r250]
        del_250 = [r['avg_delay_ms'] for r in data_r250]

        t_300 = [r['time_sec'] for r in data_r300]
        pdr_300 = [r['pdr_percent'] for r in data_r300]
        del_300 = [r['avg_delay_ms'] for r in data_r300]

        t_350 = [r['time_sec'] for r in data_r350]
        pdr_350 = [r['pdr_percent'] for r in data_r350]
        del_350 = [r['avg_delay_ms'] for r in data_r350]

        # Subplot 1: PDR
        ax1.plot(t_250, pdr_250, 'r--', linewidth=2.0, label='R_cutoff = 250 m')
        ax1.plot(t_300, pdr_300, 'b-', linewidth=2.2, label='R_cutoff = 300 m (Thiet ke goc)')
        ax1.plot(t_350, pdr_350, 'g-.', linewidth=2.0, label='R_cutoff = 350 m')
        ax1.set_ylabel('Ty le nhan goi PDR (%)', fontsize=12, fontweight='bold')
        ax1.set_title('Danh gia do nhay cua he thong theo nguong ban kinh cat song R_cutoff', fontsize=13, fontweight='bold')
        ax1.set_ylim([95, 101])
        ax1.grid(True, linestyle=':', alpha=0.7)
        ax1.legend(loc='lower left', frameon=True)

        # Subplot 2: Delay
        ax2.plot(t_250, del_250, 'r--', linewidth=1.8, label='R_cutoff = 250 m')
        ax2.plot(t_300, del_300, 'b-', linewidth=2.0, label='R_cutoff = 300 m')
        ax2.plot(t_350, del_350, 'g-.', linewidth=1.8, label='R_cutoff = 350 m')
        ax2.set_xlabel('Thoi gian mo phong (giay)', fontsize=12, fontweight='bold')
        ax2.set_ylabel('Do tre truyen thong (ms)', fontsize=12, fontweight='bold')
        ax2.grid(True, linestyle=':', alpha=0.7)
        ax2.legend(loc='upper right', frameon=True)

        plt.tight_layout()
        out_chart1 = os.path.join(res_dir, "extended_sensitivity_r_cutoff.png")
        plt.savefig(out_chart1, dpi=300)
        plt.close()
        print(f"[✓] Da xuat do thi do nhay: {out_chart1}")

    # 2. Ve do thi so sanh AODV vs OLSR neu co du lieu
    protocol_available = (data_r300 is not None and data_olsr is not None)
    if protocol_available:
        fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(13, 5))

        t_aodv = [r['time_sec'] for r in data_r300]
        pdr_aodv = [r['pdr_percent'] for r in data_r300]
        del_aodv = [r['avg_delay_ms'] for r in data_r300]

        t_olsr = [r['time_sec'] for r in data_olsr]
        pdr_olsr = [r['pdr_percent'] for r in data_olsr]
        del_olsr = [r['avg_delay_ms'] for r in data_olsr]

        # PDR Comparison
        ax1.plot(t_aodv, pdr_aodv, 'b-', linewidth=2.2, label='AODV (Reactive RFC 3561)')
        ax1.plot(t_olsr, pdr_olsr, 'm--', linewidth=2.0, label='OLSR (Proactive RFC 3626)')
        ax1.set_xlabel('Thoi gian mo phong (giay)', fontsize=11, fontweight='bold')
        ax1.set_ylabel('PDR (%)', fontsize=11, fontweight='bold')
        ax1.set_title('Doi sanh Ty le nhan goi PDR: AODV vs OLSR', fontsize=12, fontweight='bold')
        ax1.set_ylim([95, 101])
        ax1.grid(True, linestyle=':', alpha=0.7)
        ax1.legend(loc='lower left', frameon=True)

        # Delay Comparison
        ax2.plot(t_aodv, del_aodv, 'b-', linewidth=2.0, label='AODV')
        ax2.plot(t_olsr, del_olsr, 'm--', linewidth=1.8, label='OLSR')
        ax2.set_xlabel('Thoi gian mo phong (giay)', fontsize=11, fontweight='bold')
        ax2.set_ylabel('Do tre trung binh (ms)', fontsize=11, fontweight='bold')
        ax2.set_title('Doi sanh Do tre truyen thong: AODV vs OLSR', fontsize=12, fontweight='bold')
        ax2.grid(True, linestyle=':', alpha=0.7)
        ax2.legend(loc='upper right', frameon=True)

        plt.tight_layout()
        out_chart2 = os.path.join(res_dir, "extended_protocol_aodv_vs_olsr.png")
        plt.savefig(out_chart2, dpi=300)
        plt.close()
        print(f"[✓] Da xuat do thi giao thuc: {out_chart2}")

if __name__ == "__main__":
    main()
