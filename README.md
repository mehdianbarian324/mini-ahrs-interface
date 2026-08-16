# mini-ahrs-interface
C interface for MiniAHRS module: serial communication and navigation data parsing
# MiniAHRS C Driver & Interface

A high-reliability Embedded C interface and driver library for the **MiniAHRS** (Attitude and Heading Reference System) sensor, tailored for avionics and navigation systems.

## 📌 Overview
This repository contains low-level drivers, protocol parsing logic, and integration tests for interfacing with the MiniAHRS module over serial communication (UART / RS-232). It handles high-frequency attitude telemetry (Roll, Pitch, Yaw) and sensor calibration routines.

## 📂 Repository Structure
```text
├── miniahrs.c / .h                           # Core MiniAHRS driver and packet decoder
├── readserial.c                              # Low-level POSIX serial port communication handler
├── demo.c                                    # Basic acquisition and continuous streaming demo
├── freqtest.c                                # Latency and sampling frequency benchmarking tool
├── test.c                                    # Complete functional verification and test harness
├── Makefile                                  # Build configuration for GCC toolchain
└── MiniAHRS-Datasheet-rev2.10_October_2021.pdf # Hardware datasheet and protocol specification
