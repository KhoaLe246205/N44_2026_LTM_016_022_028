#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import sys
import time
import math
import socket
import json
import argparse
from pathlib import Path

if sys.platform.startswith('win'):
    sys.stdout.reconfigure(encoding='utf-8', errors='replace')
    sys.stderr.reconfigure(encoding='utf-8', errors='replace')

try:
    from pymavlink import mavutil
    HAS_PYMAVLINK = True
except ImportError:
    HAS_PYMAVLINK = False

class Drone:
    def __init__(self, name, target_x, speed=10.0):
        self.name = name
        self.target_x = target_x
        self.speed = speed
        self.port = None
        self.conn = None
        self.current_pos = [0.0, 0.0, 0.0]
        self.use_mock = False

    def assign_connection(self, port, conn):
        self.port = port
        self.conn = conn
        self.use_mock = False

    def send_setpoint(self, x, y, z):
        self.target_x = float(x)
        if self.use_mock or self.conn is None:
            return
        # Bitmask chỉ điều khiển vị trí x, y, z
        type_mask = 0b0000111111111000
        self.conn.mav.set_position_target_local_ned_send(
            0,
            self.conn.target_system, self.conn.target_component,
            mavutil.mavlink.MAV_FRAME_LOCAL_NED,
            type_mask,
            float(x), float(y), float(z),
            0, 0, 0,
            0, 0, 0,
            0, 0
        )

    def set_offboard(self):
        if self.use_mock or self.conn is None:
            return
        try:
            self.conn.set_mode_px4("OFFBOARD")
        except Exception:
            self.conn.mav.command_long_send(
                self.conn.target_system, self.conn.target_component,
                mavutil.mavlink.MAV_CMD_DO_SET_MODE,
                0,
                mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
                6, # OFFBOARD mode
                0, 0, 0, 0, 0
            )

    def arm(self):
        if self.use_mock or self.conn is None:
            return
        self.conn.mav.command_long_send(
            self.conn.target_system, self.conn.target_component,
            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
            0,
            1,      # Arm
            21196,  # Force arm trong môi trường SITL
            0, 0, 0, 0, 0
        )

    def update_position(self, dt):
        if self.use_mock or self.conn is None:
            dx = self.target_x - self.current_pos[0]
            dist = abs(dx)
            step = self.speed * dt
            if dist > 0.1:
                if step >= dist:
                    self.current_pos[0] = self.target_x
                else:
                    self.current_pos[0] += (dx / dist) * step
            self.current_pos[2] = 30.0
            return self.current_pos

        # Đọc dữ liệu tọa độ thực tế từ Gazebo
        while True:
            msg = self.conn.recv_match(type=['LOCAL_POSITION_NED', 'ODOMETRY'], blocking=False)
            if not msg:
                break
            if msg.get_type() == 'LOCAL_POSITION_NED':
                self.current_pos = [round(msg.x, 2), round(msg.y, 2), round(-msg.z, 2)]
            elif msg.get_type() == 'ODOMETRY':
                self.current_pos = [round(msg.x, 2), round(msg.y, 2), round(-msg.z, 2)]
        return self.current_pos


def scan_mavlink_ports():
    """Quét các cổng UDP MAVLink đang phát sóng từ PX4 SITL."""
    detected = []
    if not HAS_PYMAVLINK:
        return detected

    ports_to_check = [14540, 14541, 14542, 14543, 14544, 14550, 14551]
    print("[+] Đang tự động dò tìm các Drone đang chạy trong Gazebo (quét cổng 14540 - 14544)...")

    for port in ports_to_check:
        for conn_str in [f"udpin:0.0.0.0:{port}", f"udp:127.0.0.1:{port}"]:
            try:
                conn = mavutil.mavlink_connection(conn_str, timeout=1.2)
                hb = conn.wait_heartbeat(timeout=1.2)
                if hb is not None:
                    sys_id = conn.target_system
                    if not any(d['port'] == port for d in detected):
                        print(f"    [✓] ĐÃ PHÁT HIỆN DRONE: Cổng UDP {port} (System ID: {sys_id})")
                        detected.append({'port': port, 'conn': conn, 'sys_id': sys_id})
                    break
            except Exception:
                pass
    return detected


def main():
    parser = argparse.ArgumentParser(description="PX4 Multi-Vehicle Simultaneous Controller")
    parser.add_argument("--mock", action="store_true", help="Chạy chế độ mô phỏng độc lập không cần Gazebo")
    parser.add_argument("--speed", type=float, default=10.0, help="Tốc độ bay của UAV Trinh sát (m/s)")
    parser.add_argument("--duration", type=float, default=60.0, help="Tổng thời gian nhiệm vụ (giây)")
    args = parser.parse_args()

    results_dir = Path(__file__).resolve().parent.parent / "results"
    results_dir.mkdir(parents=True, exist_ok=True)
    telemetry_csv = results_dir / "px4_flight_telemetry.csv"

    print("==================================================================")
    print("  ĐIỀU KHIỂN BAY ĐỒNG THỜI 2 DRONE (TỰ ĐỘNG DÒ CỔNG MAVLINK PX4)  ")
    print("==================================================================")

    uav1 = Drone(name="UAV 1 (Relay)", target_x=250.0, speed=15.0)
    uav2 = Drone(name="UAV 2 (Recon)", target_x=500.0, speed=args.speed)

    if not args.mock and HAS_PYMAVLINK:
        active_drones = scan_mavlink_ports()
        if len(active_drones) >= 2:
            print(f"[✓] Tìm thấy {len(active_drones)} Drone thật trong Gazebo!")
            uav1.assign_connection(active_drones[0]['port'], active_drones[0]['conn'])
            uav2.assign_connection(active_drones[1]['port'], active_drones[1]['conn'])
        elif len(active_drones) == 1:
            print(f"[!] Chỉ tìm thấy 1 Drone trên cổng {active_drones[0]['port']}.")
            uav2.assign_connection(active_drones[0]['port'], active_drones[0]['conn'])
            uav1.use_mock = True
        else:
            print("[!] Không tìm thấy Drone nào qua cổng MAVLink -> Chuyển sang chế độ mô phỏng Mock.")
            uav1.use_mock = True
            uav2.use_mock = True
    else:
        uav1.use_mock = True
        uav2.use_mock = True

    print("\n------------------------------------------------------------------")
    print(f" - UAV 1 (Relay Node):      Cổng {uav1.port if uav1.port else 'Mock'} -> Bay đến (250m, 0, 30m)")
    print(f" - UAV 2 (Recon Node):      Cổng {uav2.port if uav2.port else 'Mock'} -> Bay thẳng 0m -> 500m")
    print("------------------------------------------------------------------")

    # Gửi chuỗi setpoint vị trí mồi trước khi chuyển sang OFFBOARD để tránh failsafe timeout
    print("\n[+] Đang gửi setpoint mồi đồng thời cho 2 Drone...")
    for _ in range(25):
        uav1.send_setpoint(0.0, 0.0, -30.0)
        uav2.send_setpoint(0.0, 0.0, -30.0)
        time.sleep(0.1)

    print("[+] Kích hoạt chế độ OFFBOARD...")
    uav1.set_offboard()
    uav2.set_offboard()

    for _ in range(5):
        uav1.send_setpoint(0.0, 0.0, -30.0)
        uav2.send_setpoint(0.0, 0.0, -30.0)
        time.sleep(0.1)

    print("[+] Gửi lệnh ARM cho cả 2 Drone...")
    uav1.arm()
    uav2.arm()

    print("[✓] CẢ 2 DRONE ĐÃ ĐƯỢC KÍCH HOẠT VÀ ĐANG CẤT CÁNH TRONG GAZEBO!\n")

    broadcast_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    BROADCAST_PORT = 19999

    start_time = time.time()
    last_update = start_time

    with open(telemetry_csv, "w", encoding="utf-8") as f:
        f.write("time_sec,uav1_x,uav1_y,uav1_z,uav2_x,uav2_y,uav2_z,dist_gcs_recon,dist_gcs_relay,dist_relay_recon\n")

        try:
            while True:
                now = time.time()
                elapsed = now - start_time
                if elapsed > args.duration:
                    break

                dt = now - last_update
                last_update = now

                # UAV 2 bay trinh sát thẳng về phía trước tới mốc 500m
                target_x2 = min(args.speed * elapsed, 500.0)
                uav2.send_setpoint(target_x2, 0.0, -30.0)

                # UAV 1 bám trung điểm động giữa GCS và UAV 2, chặn tối đa tại 250m
                target_x1 = min(target_x2 / 2.0, 250.0)
                uav1.send_setpoint(target_x1, 0.0, -30.0)

                pos1 = uav1.update_position(dt)
                pos2 = uav2.update_position(dt)

                dist_0_2 = math.sqrt(pos2[0]**2 + pos2[1]**2 + pos2[2]**2)
                dist_0_1 = math.sqrt(pos1[0]**2 + pos1[1]**2 + pos1[2]**2)
                dist_1_2 = math.sqrt((pos2[0]-pos1[0])**2 + (pos2[1]-pos1[1])**2 + (pos2[2]-pos1[2])**2)

                f.write(f"{elapsed:.2f},{pos1[0]:.2f},{pos1[1]:.2f},{pos1[2]:.2f},"
                        f"{pos2[0]:.2f},{pos2[1]:.2f},{pos2[2]:.2f},"
                        f"{dist_0_2:.2f},{dist_0_1:.2f},{dist_1_2:.2f}\n")
                f.flush()

                # Gửi tọa độ qua UDP cho ns-3 bridge
                telemetry_payload = json.dumps({
                    "time": round(elapsed, 2),
                    "uav1": {"x": round(pos1[0], 2), "y": round(pos1[1], 2), "z": round(pos1[2], 2)},
                    "uav2": {"x": round(pos2[0], 2), "y": round(pos2[1], 2), "z": round(pos2[2], 2)},
                    "dist_gcs_recon": round(dist_0_2, 2),
                    "dist_relay_recon": round(dist_1_2, 2)
                }).encode('utf-8')
                broadcast_sock.sendto(telemetry_payload, ("127.0.0.1", BROADCAST_PORT))

                status_link = "DIRECT (1-hop)" if dist_0_2 <= 300.0 else "RELAY VIA UAV 1 (2-hop)"
                sys.stdout.write(
                    f"\r[T={elapsed:5.1f}s] UAV1 (Relay): ({pos1[0]:5.1f}, {pos1[1]:4.1f}, {pos1[2]:4.1f}) | "
                    f"UAV2 (Recon): ({pos2[0]:5.1f}, {pos2[1]:4.1f}, {pos2[2]:4.1f}) | "
                    f"Dist GCS->Recon: {dist_0_2:5.1f}m -> [{status_link}]   "
                )
                sys.stdout.flush()
                time.sleep(0.1)

        except KeyboardInterrupt:
            print("\n[!] Dừng điều khiển nhiệm vụ.")

    print(f"\n[✓] Hoàn thành nhiệm vụ bay! Dữ liệu telemetry đã lưu vào: {telemetry_csv}")

if __name__ == "__main__":
    main()
