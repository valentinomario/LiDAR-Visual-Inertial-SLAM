
# LiDAR-Visual-Inertial SLAM

This repository contains the implementation of a **LiDAR-Visual-Inertial SLAM system** designed to run on **NVIDIA Jetson Orin NX**. The system integrates **LiDAR, visual, and inertial sensors** using a tightly coupled optimization framework for **real-time odometry and mapping**. The implementation is based on existing open-source code, please refer to [the acknowledgement section](#acknowledgement).

The project is implemented in **ROS 2 Humble**, leveraging **CUDA acceleration** for image processing and is shipped with a **Docker** image to streamline development.

## Features

-   **Tightly Coupled Sensor Fusion**: Leveraging the strengths of LiDAR, visual, and inertial sensors to enhance state estimation.
-   **Enhanced Feature Tracking**: Combining visual and LiDAR data to reliably track features even in low-texture environments.
-   **Efficient Initialization**: Rapid initialization achieved by integrating visual Structure-from-Motion (SfM) with IMU preintegration.
-   **Global Consistency**: Employing pose graph optimization to manage drift through frequent loop closure detection.

## Table of Contents

-   [Quick-Start Guide](#quick-start-guide)
-   [System Architecture](#system-architecture)
-   [Hardware Requirements](#hardware-requirements)
-   [Acknowledgement](#acknowledgement)

----------

## Quick-Start Guide

Refer to the [Quick-Start Guide](./docs/quick-start.md) for step-by-step instructions on building and running the system using Docker.

----------


## System Architecture

The system consists of two parallel subsystems that exchange information to enhance state estimation:

###  1. Visual-Inertial System (VIS)
- Extracts and tracks **image features** from the camera.
- Performs **IMU preintegration** to refine motion estimation.
- Uses **LiDAR-derived depth information** to improve accuracy.
- Maintains a **sliding window optimization** for local consistency.
- Contributes to **loop closure detection** using visual feature matching.

### 2. LiDAR-Inertial System (LIS)
- Performs **IMU preintegration** for an initial motion estimation.
- Uses **IMU preintegration** to deskew LiDAR scans.
- Performs **scan matching** against a global map to compute odometry.
- Maintains a **factor graph optimization** to refine trajectory estimates.
- Detects **loop closures** based on LiDAR feature matching.
- Uses **VIS odometry estimates** to improve initial pose estimation.

### Information Exchange
The two subsystems operate **independently** but share key data:
- **VIS → LIS**:
  - Sends **visual odometry estimates** to LIS as an **initial guess** for scan-matching.
  - Contributes to **loop closure detection**, helping LIS correct drift.
- **LIS → VIS**:
  - Provides **LiDAR depth information** to VIS, improving feature triangulation.
  - Sends **pose corrections** when loop closures refine the global trajectory.

This tightly coupled architecture ensures **robust tracking**, reducing drift and improving **localization accuracy**, even in feature-poor environments.


----------

## Hardware Requirements

The system has been developed and tested with the following hardware:

-   **LiDAR**: Livox MID360
-   **Camera**: Arducam IMX219
-   **IMU**: Integrated Livox 6-axis IMU
-   **Processing Unit**: NVIDIA Jetson Orin NX
-   **Carrier Board**: Seeedstudio A603

While it is possible to run the system on different hardware, modifications may be required.

----------

## Acknowledgement

1.  The Visual-Inertial Subsystem builds upon [VINS-Mono](https://github.com/HKUST-Aerial-Robotics/VINS-Mono)
2.  The LiDAR-Inertial Subsystem builds upon [LIO-SAM](https://github.com/TixiaoShan/LIO-SAM)
3.  The tightly coupled strategy has been implemented following the strategy of [LVI-SAM](https://github.com/TixiaoShan/LVI-SAM)


