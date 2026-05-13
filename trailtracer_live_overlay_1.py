# trailtracer_live_overlay.py
# Modes:
#   --mode sim     : laptop-only demo square path
#   --mode serial  : live ESP32 serial telemetry
#
# Outputs in ./resources:
#   track_points.csv
#   track_points.kml
#   tracker_map_live.html
#   simulation_plot.png

import argparse
import csv
import math
import time
from dataclasses import dataclass
from pathlib import Path

import matplotlib.pyplot as plt

try:
    import folium
except ImportError:
    folium = None

try:
    import serial
    from serial.tools import list_ports
except ImportError:
    serial = None
    list_ports = None

STEP_LENGTH_M = 0.60
HOME_LAT = 12.9716
HOME_LON = 77.5946

# demo route used for laptop simulation
DEMO_STEPS = [
    (8, 0),     # east
    (8, 90),    # north
    (8, 180),   # west
    (8, 270),   # south
]

BASE_DIR = Path(__file__).resolve().parent
RESOURCE_DIR = BASE_DIR / "resources"
RESOURCE_DIR.mkdir(exist_ok=True)

CSV_FILE = RESOURCE_DIR / "track_points.csv"
KML_FILE = RESOURCE_DIR / "track_points.kml"
MAP_FILE = RESOURCE_DIR / "tracker_map_live.html"
PLOT_FILE = RESOURCE_DIR / "simulation_plot.png"


@dataclass
class Point:
    step: int
    heading_deg: float
    x_m: float
    y_m: float
    lat: float
    lon: float
    dist_home_m: float
    state: int = 0
    guide_idx: int = -1


def meters_to_latlon(dx_m: float, dy_m: float, lat0: float, lon0: float):
    dlat = dy_m / 111111.0
    dlon = dx_m / (111111.0 * math.cos(math.radians(lat0)))
    return lat0 + dlat, lon0 + dlon


def write_csv(points):
    with open(CSV_FILE, "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["step", "heading_deg", "x_m", "y_m", "lat", "lon", "dist_home_m", "state", "guide_idx"])
        for p in points:
            w.writerow([
                p.step, f"{p.heading_deg:.2f}", f"{p.x_m:.3f}", f"{p.y_m:.3f}",
                f"{p.lat:.8f}", f"{p.lon:.8f}", f"{p.dist_home_m:.3f}", p.state, p.guide_idx
            ])


def write_kml(points):
    header = """<?xml version="1.0" encoding="UTF-8"?>
<kml xmlns="http://www.opengis.net/kml/2.2"><Document><Placemark><LineString><tessellate>1</tessellate><coordinates>"""
    footer = """</coordinates></LineString></Placemark></Document></kml>"""
    with open(KML_FILE, "w", newline="") as f:
        f.write(header)
        f.write("\n".join(f"{p.lon},{p.lat},0" for p in points))
        f.write(footer)


def write_map(points):
    if folium is None or not points:
        return

    m = folium.Map(location=[HOME_LAT, HOME_LON], zoom_start=20, tiles="OpenStreetMap")
    folium.PolyLine([(p.lat, p.lon) for p in points], color="blue", weight=5, opacity=0.9).add_to(m)

    home = points[0]
    folium.Marker(
        [home.lat, home.lon],
        tooltip="HOME",
        popup="HOME",
        icon=folium.Icon(color="green", icon="home"),
    ).add_to(m)

    cur = points[-1]
    folium.Marker(
        [cur.lat, cur.lon],
        tooltip=f"Step {cur.step}",
        popup=f"Step {cur.step}",
        icon=folium.Icon(color="red", icon="user"),
    ).add_to(m)

    m.save(str(MAP_FILE))


def plot_route(points, title="Tracker Live Overlay"):
    if not points:
        return

    xs = [p.x_m for p in points]
    ys = [p.y_m for p in points]

    plt.clf()
    plt.plot(xs, ys, marker="o", linewidth=2)

    plt.scatter([0], [0], marker="x", s=120)
    plt.text(0, 0, " HOME", va="bottom")

    cur = points[-1]
    plt.title(f"{title} | step={cur.step} | dist={cur.dist_home_m:.2f} m")
    plt.xlabel("X (m)")
    plt.ylabel("Y (m)")
    plt.axis("equal")
    plt.grid(True)
    plt.tight_layout()
    plt.savefig(PLOT_FILE, dpi=200)
    plt.pause(0.001)


def refresh_outputs(points):
    write_csv(points)
    write_kml(points)
    write_map(points)
    plot_route(points)
    print(f"Saved outputs -> {RESOURCE_DIR}")


def simulate_points():
    points = [Point(0, 0.0, 0.0, 0.0, HOME_LAT, HOME_LON, 0.0, 0, -1)]
    x = y = 0.0
    step = 0

    for count, heading_deg in DEMO_STEPS:
        th = math.radians(heading_deg)
        for _ in range(count):
            step += 1
            x += STEP_LENGTH_M * math.cos(th)
            y += STEP_LENGTH_M * math.sin(th)
            lat, lon = meters_to_latlon(x, y, HOME_LAT, HOME_LON)
            d = math.sqrt(x*x + y*y)
            points.append(Point(step, heading_deg, x, y, lat, lon, d, 1, -1))
    return points


def parse_esp_line(line):
    # ESPLOG,ms,state,step,headingDeg,posX,posY,kfX,kfY,distHome,ax,ay,az,gx,gy,gz,relHome,guideIdx
    if not line.startswith("ESPLOG,"):
        return None
    parts = line.strip().split(",")
    if len(parts) < 18:
        return None

    try:
        state = int(parts[2])
        step = int(parts[3])
        heading_deg = float(parts[4])
        x = float(parts[7])  # kfX is more stable to show on map
        y = float(parts[8])  # kfY
        dist_home = float(parts[9])
        guide_idx = int(parts[17])
    except ValueError:
        return None

    lat, lon = meters_to_latlon(x, y, HOME_LAT, HOME_LON)
    return Point(step, heading_deg, x, y, lat, lon, dist_home, state, guide_idx)


def list_available_ports():
    """List all available serial ports."""
    if list_ports is None:
        return []
    ports = []
    for port, desc, hwid in list_ports.comports():
        ports.append((port, desc, hwid))
    return ports


def run_sim():
    points = []
    base = simulate_points()
    plt.ion()
    for i in range(1, len(base) + 1):
        points = base[:i]
        refresh_outputs(points)
        time.sleep(0.15)
    print("Simulation complete.")
    print(f"Open map: {MAP_FILE}")


def run_serial(port, baud):
    if serial is None:
        raise RuntimeError("pyserial is not installed. Run: pip install pyserial")
    
    # If port is None or "auto", try to detect
    if port is None or port == "auto":
        available = list_available_ports()
        if not available:
            raise RuntimeError("No serial ports found. Please connect the ESP32.")
        print(f"Available ports:\n{chr(10).join(f'  {p[0]}: {p[1]}' for p in available)}")
        port = available[0][0]
        print(f"Using auto-detected port: {port}")
    
    # Try to open the port with retries
    max_retries = 3
    for attempt in range(max_retries):
        try:
            print(f"Connecting to {port} @ {baud} baud (attempt {attempt+1}/{max_retries})...")
            ser = serial.Serial(port, baudrate=baud, timeout=1)
            print(f"Connected to {port}.")
            break
        except serial.SerialException as e:
            print(f"  Error: {e}")
            if attempt < max_retries - 1:
                print(f"  Retrying in 1 second...")
                time.sleep(1)
            else:
                available = list_available_ports()
                print(f"\nFailed to connect after {max_retries} attempts.")
                if available:
                    print("Available ports:")
                    for p, desc, hwid in available:
                        print(f"  {p}: {desc}")
                    print(f"\nTry: python trailtracer_live_overlay.py --mode serial --port {available[0][0]}")
                else:
                    print("No serial ports detected. Check if ESP32 is connected.")
                raise
    
    points = [Point(0, 0.0, 0.0, 0.0, HOME_LAT, HOME_LON, 0.0, 0, -1)]
    plt.ion()
    print(f"Listening on {port} @ {baud} ...")

    while True:
        raw = ser.readline().decode(errors="ignore").strip()
        if not raw:
            continue

        p = parse_esp_line(raw)
        if p is None:
            continue

        if p.step == points[-1].step:
            points[-1] = p
        elif p.step > points[-1].step:
            points.append(p)

        refresh_outputs(points)
        print(raw)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--mode", choices=["sim", "serial"], default="sim")
    ap.add_argument("--port", default="auto", help="Serial port (e.g., COM3 or 'auto' to detect)")
    ap.add_argument("--baud", type=int, default=115200)
    ap.add_argument("--list-ports", action="store_true", help="List available serial ports and exit")
    args = ap.parse_args()

    if args.list_ports:
        ports = list_available_ports()
        if ports:
            print("Available serial ports:")
            for port, desc, hwid in ports:
                print(f"  {port}: {desc}")
        else:
            print("No serial ports found.")
        return

    if args.mode == "sim":
        run_sim()
    else:
        run_serial(args.port, args.baud)


if __name__ == "__main__":
    main()