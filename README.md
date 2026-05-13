# T2_TrailTracer
# TrailTracer

TrailTracer is an offline pedestrian navigation system built on **Pedestrian Dead Reckoning (PDR)**.  
It estimates motion from an initial starting point using IMU sensor data, step detection, heading estimation, and breadcrumb-style path tracking.  
The project is designed to work where GPS is unreliable or unavailable, such as indoor areas, forests, tunnels, basements, and mountainous terrain.

---

## Overview

TrailTracer combines embedded sensing with live laptop visualization to show the user’s path in real time. The ESP32 processes IMU data from the MPU6050, estimates heading using Mahony AHRS and filtering, detects steps, and updates the position estimate.  
When the system enters guide mode, it retraces the recorded breadcrumb path and provides audio cues through the MAX98357A amplifier.

A Python overlay tool is used for validation and demonstration. It reads serial telemetry from the ESP32, plots the movement live using Matplotlib, and writes a Leaflet/Folium-based map for browser viewing.

---

## Key Features

- Offline navigation without GPS dependency
- Step-based dead reckoning using MPU6050
- Heading estimation using Mahony AHRS + Kalman filtering
- Breadcrumb path recording and backtracking
- OLED display for state, distance, and map view
- Guide mode with left / right / straight / off-course audio alerts
- Live serial telemetry for laptop-based validation
- Matplotlib live route visualization
- Leaflet/Folium map export for browser-based path viewing
- Simulation mode for testing without physical movement

---

## System Progression

### Phase 1 — Concept and Simulation
The first stage focused on the navigation concept and motion math.  
A laptop simulation was created to test:
- step length updates
- heading-based coordinate changes
- path plotting
- map export to CSV, KML, and HTML

This phase helped verify the dead-reckoning logic before using real hardware.

### Phase 2 — ESP32 + MPU6050 Integration
The second stage moved the logic onto the ESP32 using the MPU6050 IMU.  
This added:
- real sensor input
- step detection
- heading fusion
- position estimation
- OLED output

### Phase 3 — Guide Mode and Breadcrumb Backtracking
The third stage added return navigation.  
The system records each estimated position as a breadcrumb and then uses those stored points to guide the user back along the path.

### Phase 4 — Audio Guidance
The fourth stage added sound output through MAX98357A.  
Audio cues are enabled only in guide mode and include:
- straight
- left
- right
- off-course warning

### Phase 5 — Live Laptop Overlay
The final stage connects ESP32 telemetry to a Python overlay.  
This gives:
- live Matplotlib route updates
- browser-based map visualization
- path validation against real movement

---

## Hardware

- **ESP32 Dev Board**
- **MPU6050 IMU**
- **SSD1306 OLED display (128x64)**
- **MAX98357A I2S amplifier**
- **Speaker**
- **Push button**
- Breadboard and jumper wires
- USB cable for programming and serial monitoring

---

## Tech Stack

### Embedded / Firmware
- **Arduino C++**
- **ESP32 Arduino Core**
- **Adafruit MPU6050 library**
- **Adafruit SSD1306 library**
- **Adafruit GFX library**
- **FreeRTOS task for audio**
- **I2S audio output**

### Signal Processing
- **Mahony AHRS**
- **Kalman filtering**
- **Low-pass filtering**
- **Step detection using acceleration thresholding**
- **Breadcrumb navigation logic**

### Desktop / Overlay
- **Python 3**
- **Matplotlib**
- **Folium**
- **Leaflet.js** via Folium HTML output
- **PySerial**

### File Outputs
- **CSV** for telemetry
- **KML** for Google Earth
- **HTML** for live map display
- **PNG** for saved plots

---


