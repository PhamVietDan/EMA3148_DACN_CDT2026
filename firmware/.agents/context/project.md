# Project Overview

Dự án đồ án chuyên ngành điều khiển Drone (Quadrotor) sử dụng STM32F4, FreeRTOS, cảm biến IMU BMI088/BMI323, truyền thông SBUS và điều khiển động cơ qua ESC PWM.

## Purpose
- Điều khiển cân bằng Drone tầng thấp (vòng góc Angle, vòng tốc độ góc Rate).
- Xử lý tín hiệu điều khiển SBUS từ tay cầm và cơ chế Failsafe.
- Giao tiếp dữ liệu Telemetry / Blackbox qua USB CDC / GUI giám sát.

## Tech Stack
- Firmware: C, STM32CubeIDE / Makefile, FreeRTOS, HAL/LL Drivers.
- Hardware: STM32F405/F4xx, IMU SPI, 4x Brushless ESC (PWM), SBUS Receiver.
- Ground Control: GUI giám sát tham số thời gian thực, lưu trữ thông số Flash.

## Architecture
- Task Ctrl (1000 Hz): Đọc IMU, Kalman filter, PID Angle/Rate, Mixer ra 4 motor ESC.
- Task Slow (100 Hz): Xử lý tín hiệu SBUS, kiểm tra Failsafe / Arming logic.
- Task Telem (50 Hz): Telemetry qua USB CDC và ghi dữ liệu Blackbox.
