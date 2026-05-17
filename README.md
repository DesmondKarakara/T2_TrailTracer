# T2_TrailTracer
# TrailTracer Final

TrailTracer is an offline pedestrian navigation prototype based on dead reckoning.  
This final version uses:

- ESP32
- MPU6050 IMU
- SSD1306 OLED
- MAX98357A audio amplifier hook
- Python visualizer for CSV / KML / PNG / HTML output

## Coordinate system

The project uses a local navigation frame:

# TrailTracer – Simple Mathematical Explanation

## 1. Coordinate System Used

TrailTracer uses a 2D coordinate system:

* **X-axis** → East / West movement
* **Y-axis** → North / South movement
* **Home position** = `(0,0)`

### Direction Rules

| Direction | X value  | Y value  |
| --------- | -------- | -------- |
| North     | 0        | Positive |
| South     | 0        | Negative |
| East      | Positive | 0        |
| West      | Negative | 0        |

The MPU6050 gives orientation (heading angle).

Heading is measured:

* `0° = North`
* `90° = East`
* `180° = South`
* `270° = West`

---

# 2. Position Update Equations

Each detected step updates the current position.

## Equations

[
x_k = x_{k-1} + s_k \sin(\theta_k)
]

[
y_k = y_{k-1} + s_k \cos(\theta_k)
]

Where:

| Symbol     | Meaning            |
| ---------- | ------------------ |
| (x_k)      | Current X position |
| (y_k)      | Current Y position |
| (s_k)      | Step length        |
| (\theta_k) | Heading angle      |

---

# 3. Example Movement

Suppose:

* Step length = `0.7 m`
* Heading = `90°`

Since:

[
\sin(90°)=1
]

[
\cos(90°)=0
]

Then:

[
dx = 0.7 \times 1 = 0.7
]

[
dy = 0.7 \times 0 = 0
]

So:

* X increases by `0.7`
* Y remains same

This means the user moved EAST.

---

# 4. Accelerometer Magnitude

The MPU6050 provides:

* Ax
* Ay
* Az

These are combined into one magnitude value.

## Formula

[
a_{mag} = \sqrt{Ax^2 + Ay^2 + Az^2}
]

This removes orientation dependency.

---

# 5. Low-Pass Filter

The accelerometer signal contains noise.

A low-pass filter smooths the signal.

## Formula

[
a_f[k] = \alpha a_f[k-1] + (1-\alpha)a_{mag}[k]
]

Where:

| Symbol     | Meaning                 |
| ---------- | ----------------------- |
| (a_f[k])   | Current filtered value  |
| (a_f[k-1]) | Previous filtered value |
| (\alpha)   | Filter constant         |

Typical value:

[
\alpha = 0.9
]

This keeps walking motion while removing vibration noise.

---

# 6. Step Detection

A step is detected when:

1. Filtered acceleration crosses threshold
2. Signal behaves like a peak
3. Enough time passed from previous step

## Conditions

[
a_f > Threshold
]

AND

[
TimeSinceLastStep > 300ms
]

Typical threshold:

[
Threshold = 11.2 ; m/s^2
]

---

# 7. Mahony Filter (Orientation Estimation)

The MPU6050 contains:

* Accelerometer
* Gyroscope

The Mahony filter combines both.

It provides:

* Roll
* Pitch
* Yaw (heading)

## Why Mahony Filter?

Gyroscope:

* Smooth
* Fast
* Drifts over time

Accelerometer:

* Stable
* Noisy

Mahony filter combines both for stable orientation.

---

# 8. Yaw / Heading Angle

Yaw is the direction user faces.

## Important Note

MPU6050 DOES NOT contain a magnetometer.

Therefore:

* Heading is RELATIVE
* Small drift occurs over long walks

Still sufficient for short-range navigation.

---

# 9. 1D Kalman Filter for Step Length

Walking steps are not identical.

Kalman filtering smooths step length.

## Prediction

[
\hat{x}*k^- = \hat{x}*{k-1}
]

## Kalman Gain

[
K_k = \frac{P_k^-}{P_k^- + R}
]

## Update

[
\hat{x}_k = \hat{x}_k^- + K_k(z_k - \hat{x}_k^-)
]

Where:

| Symbol      | Meaning              |
| ----------- | -------------------- |
| (z_k)       | Measured step length |
| (\hat{x}_k) | Filtered step length |
| (K_k)       | Kalman gain          |

Result:

* Less fluctuation
* More stable tracking
* Better path accuracy

---

# 10. Distance to Home

Current position:

[
(x,y)
]

Distance back to home:

[
D = \sqrt{x^2 + y^2}
]

Example:

If:

* x = 3 m
* y = 4 m

Then:

[
D = \sqrt{3^2 + 4^2}
]

[
D = 5m
]

---

# 11. Bearing to Home

Bearing angle toward home:

[
\theta_{home} = atan2(-x,-y)
]

This tells the ESP32 which direction user must turn.

---

# 12. Turn Decision Logic

## Error Angle

[
Error = \theta_{home} - \theta_{current}
]

## Decision

| Error Range           | Action      |
| --------------------- | ----------- |
| > +15°                | Turn Right  |
| < -15°                | Turn Left   |
| Between -15° and +15° | Go Straight |

---


# 13. Excel File Columns
## Excel Columns

| Column Name   | Meaning              |
| ------------- | -------------------- |
| timestamp     | Time in milliseconds |
| ax            | Accelerometer X      |
| ay            | Accelerometer Y      |
| az            | Accelerometer Z      |
| gx            | Gyroscope X          |
| gy            | Gyroscope Y          |
| gz            | Gyroscope Z          |
| heading_deg   | Heading angle        |
| step_detected | 1 if step detected   |
| step_length   | Step length          |
| x_pos         | X coordinate         |
| y_pos         | Y coordinate         |
| distance_home | Distance to home     |
| guide_idx     | Guidance instruction |

---

# 14. Meaning of guide_idx

| guide_idx Value | Meaning      |
| --------------- | ------------ |
| 0               | Go Straight  |
| 1               | Turn Left    |
| 2               | Turn Right   |
| 3               | Arrived Home |

The Python visualizer uses this column to overlay arrows and turn instructions.

---

# 15. Why Earlier Map Looked Wrong

Main reasons:

1. X/Y axis mismatch
2. Wrong angle convention
3. No heading normalization
4. Random drift accumulation
5. No filtering
6. Incorrect overlay scaling

The updated implementation fixes these problems using:

* Low-pass filtering
* Mahony orientation estimation
* 1D Kalman smoothing
* Correct North/East coordinate convention
* Stable dead reckoning equations
* Proper Python plot scaling

---

# 16. Final Processing Pipeline

## ESP32 Processing

1. Read MPU6050
2. Compute acceleration magnitude
3. Apply low-pass filter
4. Detect steps
5. Estimate heading using Mahony filter
6. Smooth step length with Kalman filter
7. Update X/Y position
8. Send data to Python logger
9. Display UI on OLED

---

# 17. OLED UI Improvements

Updated UI now shows:

* X position
* Y position
* Heading
* Distance to home
* Direction arrow
* Step count
* Guide mode status

This fixes the previous issue where Y-axis was missing.

---

# 18. Python Overlay Improvements

Updated Python visualizer:

* Correct axis orientation
* Proper North arrow
* Real-time path plotting
* Turn instruction overlay
* Stable scaling
* No random jumps
* Home marker display
* CSV logging support

---

# 19. Recommended Parameters

| Parameter           | Recommended Value |
| ------------------- | ----------------- |
| Sampling Rate       | 50 Hz             |
| LPF Alpha           | 0.9               |
| Step Threshold      | 11.2              |
| Refractory Time     | 300 ms            |
| Average Step Length | 0.70 m            |
| Mahony Kp           | 2.0               |
| Mahony Ki           | 0.0               |

---



# 20. How to run

### ESP32
1. Open `TrailTracer_final.ino`
2. Install required Arduino libraries:
   - Adafruit GFX
   - Adafruit SSD1306
   - Adafruit MPU6050
   - Adafruit Unified Sensor
3. Upload to ESP32

### Python
Install:

```bash
pip install pyserial matplotlib folium
```

Run simulation:

```bash
python trailtracer_final.py --mode sim
```

Run live ESP32 mode:

```bash
python trailtracer_final.py --mode serial --port COM3
```

## Output files

Saved in `resources/`:

- `track_points.csv`
- `track_points.kml`
- `tracker_map_live.html`
- `simulation_plot.png`

## Notes

Because the MPU6050 has no magnetometer, the heading is not absolute north unless the walk starts with a known orientation. For best results, start the walk facing a consistent direction.


# 21. Final Summary

TrailTracer works without GPS by:

* Detecting steps using MPU6050
* Estimating heading using Mahony AHRS
* Smoothing motion using filters
* Updating local X/Y coordinates
* Guiding user back to home

The system is:

* Offline
* Portable
* Low-cost
* Low-power
* Real-time
* Suitable for tunnels, forests, and indoor navigation
