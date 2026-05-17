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

- `x` = east / west displacement
- `y` = north / south displacement
- home = `(0,0)`

Heading is treated as clockwise from north for the local update:

\[
x_k = x_{k-1} + s_k \sin(\theta_k)
\]
\[
y_k = y_{k-1} + s_k \cos(\theta_k)
\]

## Filters used

### 1. Low-pass filter on accelerometer magnitude
Used to smooth motion before step detection:

\[
a_f[k] = \alpha a_f[k-1] + (1-\alpha)a_{mag}[k]
\]

### 2. Step detection
A step is declared when the filtered magnitude crosses a threshold and rises/falls like a peak.

### 3. Mahony AHRS
The ESP32 code uses an IMU-only Mahony filter from the MPU6050 accelerometer + gyro.

Important: MPU6050 has **no magnetometer**, so yaw is relative and drifts over long distances.

### 4. 1D Kalman filter
Used to smooth step length:

\[
K_k = \frac{P_k^-}{P_k^-+R}
\]
\[
\hat{x}_k = \hat{x}_k^- + K_k(z_k-\hat{x}_k^-)
\]

### 5. Dead reckoning
Each step updates the local position:

\[
dx = s \sin(\theta), \quad dy = s \cos(\theta)
\]

### 6. Guidance decision
To return home:

\[
\theta_{home} = \operatorname{atan2}(-x, -y)
\]

Then compare current heading vs home bearing to output:

- STRAIGHT
- LEFT
- RIGHT
- UTURN

## Files

- `TrailTracer_final.ino`
- `trailtracer_final.py`

## Arduino serial output

The ESP32 sends:

`STEP,<step>,<x_m>,<y_m>,<heading_deg>,<dist_home_m>,<phase>,<guid_idx>,<turn_cmd>`

Example:

`STEP,12,3.215,1.402,85.23,3.51,TRACKING,-1,NONE`

## CSV columns

The Python program writes:

- `step`
- `phase`
- `path_color`
- `x_m`
- `y_m`
- `heading_deg`
- `lat`
- `lon`
- `dist_home_m`
- `guid_idx`
- `turn_cmd`

### `guid_idx`
This is the guidance instruction index.

- `-1` means no guidance instruction
- `0,1,2,...` are successive guidance prompts

## How to run

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
