# TrailTracer – Offline Pedestrian Navigation

TrailTracer is an offline pedestrian navigation prototype based on dead reckoning.  
It uses an **ESP32**, **MPU6050 IMU**, **SSD1306 OLED** display, and **MAX98357A** audio amplifier.  
A Python visualizer generates CSV, KML, PNG, and HTML outputs from logged data.

---

## Table of Contents

1. [System Overview](#system-overview)
2. [Coordinate System](#coordinate-system)
3. [Mathematical Foundation](#mathematical-foundation)
   - [Position Update](#position-update-equations)
   - [Accelerometer Magnitude](#accelerometer-magnitude)
   - [Low-Pass Filter](#low-pass-filter)
   - [Step Detection](#step-detection)
   - [Mahony Filter (Orientation)](#mahony-filter-orientation-estimation)
   - [Yaw / Heading Angle](#yaw--heading-angle)
   - [1D Kalman Filter for Step Length](#1d-kalman-filter-for-step-length)
   - [Distance to Home](#distance-to-home)
   - [Bearing to Home](#bearing-to-home)
   - [Turn Decision Logic](#turn-decision-logic)
4. [Data Logging](#data-logging)
   - [Excel Columns](#excel-columns)
   - [guide_idx Meaning](#guide_idx-meaning)
5. [Why Earlier Maps Looked Wrong](#why-earlier-map-looked-wrong)
6. [Processing Pipeline](#processing-pipeline)
   - [ESP32 Processing](#esp32-processing)
   - [OLED UI Improvements](#oled-ui-improvements)
   - [Python Overlay Improvements](#python-overlay-improvements)
7. [Recommended Parameters](#recommended-parameters)
8. [How to Run](#how-to-run)
   - [ESP32 Setup](#esp32)
   - [Python Setup](#python)
9. [Output Files](#output-files)
10. [Notes & Limitations](#notes--limitations)
11. [Final Summary](#final-summary)

---

## System Overview

TrailTracer provides real-time pedestrian navigation without GPS by:

- Detecting steps with an MPU6050 accelerometer/gyroscope.
- Estimating heading using a Mahony AHRS filter.
- Smoothing motion with low-pass and Kalman filters.
- Updating local X/Y coordinates (dead reckoning).
- Guiding the user back to the starting point via visual/audio cues.

---

## Coordinate System

TrailTracer uses a **2D local navigation frame**:

- **X‑axis** → East / West movement
- **Y‑axis** → North / South movement
- **Home position** = `(0,0)`

### Direction Rules

| Direction | X value  | Y value  |
|-----------|----------|----------|
| North     | 0        | Positive |
| South     | 0        | Negative |
| East      | Positive | 0        |
| West      | Negative | 0        |

Heading angle from the MPU6050 (after Mahony filtering):

- `0°` = North  
- `90°` = East  
- `180°` = South  
- `270°` = West

---

## Mathematical Foundation

### Position Update Equations

Each detected step updates the current position:

$$
x_k = x_{k-1} + s_k \sin(\theta_k)
$$

$$
y_k = y_{k-1} + s_k \cos(\theta_k)
$$

| Symbol     | Meaning                  |
|------------|--------------------------|
| $x_k, y_k$ | Current position         |
| $s_k$      | Step length (meters)     |
| $\theta_k$ | Heading angle (degrees)  |

#### Example

Step length = $0.7$ m, heading = $90^\circ$ (East):  
$\sin(90^\circ)=1$, $\cos(90^\circ)=0$ → $dx = 0.7$, $dy = 0$.  
X increases by 0.7 m, Y unchanged → movement East.

---

### Accelerometer Magnitude

Raw accelerometer readings $A_x, A_y, A_z$ are combined to remove orientation dependency:

$$
a_{mag} = \sqrt{A_x^2 + A_y^2 + A_z^2}
$$

---

### Low-Pass Filter

A first-order IIR low-pass filter smooths the acceleration signal:

$$
a_f[k] = \alpha \, a_f[k-1] + (1-\alpha) \, a_{mag}[k]
$$

Typical $\alpha = 0.9$ keeps walking motion while removing vibration noise.

---

### Step Detection

A step is detected when:

1. Filtered acceleration exceeds a threshold: $a_f > \text{Threshold}$
2. The signal shows a clear peak.
3. At least **300 ms** have passed since the last step.

Typical threshold: $11.2 \, \text{m/s}^2$.

---

### Mahony Filter (Orientation Estimation)

The MPU6050 provides both accelerometer (stable but noisy) and gyroscope (smooth but drifting).  
The Mahony filter fuses them to produce stable **roll, pitch, and yaw** (heading).

- Gyroscope → fast response, low noise, long‑term drift.
- Accelerometer → no drift, but noisy.
- Mahony filter combines both for robust orientation.

Because the MPU6050 **has no magnetometer**, heading is **relative** to the initial orientation. Small drift may occur over long walks, but it is sufficient for short‑range navigation.

---

### Yaw / Heading Angle

Yaw is the direction the user faces. In this implementation, it is the filtered angle from the Mahony filter, used directly in the position update equations.

---

### 1D Kalman Filter for Step Length

Step lengths vary naturally. A 1D Kalman filter smooths the measured step length:

**Prediction:**  
$$
\hat{x}_k^- = \hat{x}_{k-1}
$$

**Kalman Gain:**  
$$
K_k = \frac{P_k^-}{P_k^- + R}
$$

**Update:**  
$$
\hat{x}_k = \hat{x}_k^- + K_k (z_k - \hat{x}_k^-)
$$

| Symbol      | Meaning                       |
|-------------|-------------------------------|
| $z_k$       | Measured step length          |
| $\hat{x}_k$ | Filtered step length          |
| $K_k$       | Kalman gain                   |
| $P_k^-$     | Predicted error covariance    |
| $R$         | Measurement noise covariance  |

Result: less fluctuation, more stable tracking, better path accuracy.

---

### Distance to Home

From current position $(x, y)$:

$$
D = \sqrt{x^2 + y^2}
$$

Example: $x = 3$ m, $y = 4$ m → $D = 5$ m.

---

### Bearing to Home

The angle (in degrees) pointing from current position back to the origin:

$$
\theta_{home} = \text{atan2}(-x, -y)
$$

This tells the ESP32 which direction the user must turn.

---

### Turn Decision Logic

Error angle = $\theta_{home} - \theta_{current}$ (normalised to $[-180^\circ, 180^\circ]$).

| Error Range           | Action      |
|-----------------------|-------------|
| $> +15^\circ$         | Turn Right  |
| $< -15^\circ$         | Turn Left   |
| Between $-15^\circ$ and $+15^\circ$ | Go Straight |

---

## Data Logging

### Excel Columns

Logged to `track_points.csv`:

| Column Name     | Meaning                          |
|----------------|----------------------------------|
| timestamp      | Time in milliseconds             |
| ax, ay, az     | Raw accelerometer values         |
| gx, gy, gz     | Raw gyroscope values             |
| heading_deg    | Current heading (degrees)        |
| step_detected  | 1 if a step was detected         |
| step_length    | Filtered step length (m)         |
| x_pos, y_pos   | Current X/Y coordinates          |
| distance_home  | Distance to home (m)             |
| guide_idx      | Guidance instruction code        |

### guide_idx Meaning

| guide_idx | Meaning      |
|-----------|--------------|
| 0         | Go Straight  |
| 1         | Turn Left    |
| 2         | Turn Right   |
| 3         | Arrived Home |

The Python visualiser uses this column to overlay arrows and turn instructions.

---

## Why Earlier Map Looked Wrong

Common issues that have been fixed:

1. **X/Y axis mismatch** – now using consistent North/East convention.
2. **Wrong angle convention** – corrected to $0^\circ$ = North, increasing clockwise.
3. **No heading normalisation** – angles are wrapped to $[0,360)$.
4. **Random drift accumulation** – reduced by Kalman and Mahony filters.
5. **No filtering** – low‑pass and Kalman filters applied.
6. **Incorrect overlay scaling** – Python plot uses equal aspect ratio.

---

## Processing Pipeline

### ESP32 Processing

1. Read MPU6050 (accelerometer + gyroscope).
2. Compute acceleration magnitude.
3. Apply low‑pass filter.
4. Detect steps (threshold + refractory period).
5. Estimate heading using Mahony filter.
6. Smooth step length with 1D Kalman filter.
7. Update X/Y position using dead reckoning.
8. Send data to serial (for Python logger).
9. Display UI on OLED.

### OLED UI Improvements

The OLED now shows:

- X position
- Y position
- Heading
- Distance to home
- Direction arrow
- Step count
- Guide mode status

### Python Overlay Improvements

The updated visualiser (`trailtracer_final.py`) provides:

- Correct axis orientation (North up).
- Real‑time path plotting.
- Turn instruction overlay (arrows).
- Stable scaling (no random jumps).
- Home marker display.
- CSV logging and KML/HTML/PNG export.

---

## Recommended Parameters

| Parameter           | Recommended Value |
|---------------------|-------------------|
| Sampling Rate       | 50 Hz             |
| LPF Alpha ($\alpha$)| 0.9               |
| Step Threshold      | 11.2 m/s²         |
| Refractory Time     | 300 ms            |
| Average Step Length | 0.70 m            |
| Mahony Kp           | 2.0               |
| Mahony Ki           | 0.0               |

---

## How to Run

### ESP32

1. Open `TrailTracer_final.ino` in Arduino IDE.
2. Install required libraries:
   - Adafruit GFX
   - Adafruit SSD1306
   - Adafruit MPU6050
   - Adafruit Unified Sensor
3. Select the correct board (ESP32 dev module) and port.
4. Upload the sketch.

### Python

Install dependencies:

```bash
pip install pyserial matplotlib folium
