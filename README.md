# Kalman-Filter-MPU6500
A lightweight, C++ Kalman filter library designed for pitch, roll, and yaw estimation using only 6-axis IMU data (accelerometer + gyroscope). Optimized for STM32 microcontrollers running at high sampling rates (4 kHz), making it suitable for rockets, drones, or real-time embedded systems.

Features

Full 3-state (θx, θy, θz) Kalman filter

Supports 6 measurements: ax, ay, az, gx, gy, gz

Initializes process and measurement noise directly from IMU datasheet values

Lightweight matrix operations, no external libraries required

Works on resource-constrained microcontrollers like STM32F1

Provides predict, update, and state retrieval functions
