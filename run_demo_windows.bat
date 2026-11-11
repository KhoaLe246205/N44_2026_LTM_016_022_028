@echo off
chcp 65001 > nul
title UAV Flying Relay Node - Co-Simulation Demo
cls
echo ========================================================================
echo    ĐỀ TÀI 2: THIẾT KẾ UAV TRẠM CHUYỂN TIẾP SÓNG (FLYING RELAY NODE)
echo         GIAO DIỆN CHẠY THỬ NGHIỆM TRÊN WINDOWS (PYTHON DEMO)
echo ========================================================================
echo 1. Chạy Giao diện Trực quan Radar Động lực học & Chuyển tiếp sóng (2D Live Visualizer)
echo 2. Chạy Kịch bản Điều khiển Bay & Thu thập Tọa độ 3D Telemetry (px4_multi_vehicle.py)
echo 3. Chạy Cầu nối Đồng mô phỏng Mạng AODV (relay_network_bridge.py)
echo 4. Xuất Biểu đồ Đánh giá Hiệu năng Mạng (plot_results.py)
echo 0. Thoát
echo ========================================================================
set /p choice="Nhập lựa chọn của bạn [0-4]: "

if "%choice%"=="1" (
    echo Đang khởi chạy Radar Visualizer...
    python "%~dp0co_simulation\live_visualizer.py"
) else if "%choice%"=="2" (
    echo Đang chạy kịch bản bay...
    python "%~dp0px4_gazebo\px4_multi_vehicle.py" --mock
) else if "%choice%"=="3" (
    echo Đang chạy Network Bridge...
    python "%~dp0co_simulation\relay_network_bridge.py"
) else if "%choice%"=="4" (
    echo Đang xuất đồ thị hiệu năng...
    python "%~dp0co_simulation\plot_results.py"
    start "" "%~dp0results\uav_relay_performance_charts.png"
) else if "%choice%"=="0" (
    exit /b
) else (
    echo Lựa chọn không hợp lệ.
)
pause
