# trailtracer_live_overlay.py (UPDATED FOR ESP32)
# Optimized for the working ESP32 code with Mahony fusion
#
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

try:
    import pyttsx3
except ImportError:
    pyttsx3 = None

# ===== CONFIGURATION FROM ESP32 =====
STEP_LENGTH_M = 0.65  # Match ESP32: STEP_LENGTH_M 0.65f
HOME_LAT = 12.9716
HOME_LON = 77.5946
DIRECTION_THRESHOLD_DEG = 15

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


class AudioGuide:
    """Text-to-speech guidance system"""
    def __init__(self):
        self.engine = None
        import platform
        self.is_mac = platform.system() == 'Darwin'
        if not self.is_mac and pyttsx3:
            try:
                self.engine = pyttsx3.init()
                self.engine.setProperty('rate', 150)  # Speed
            except Exception as e:
                print(f"Warning: Could not initialize pyttsx3: {e}")
    
    def speak(self, text):
        """Speak text if available"""
        print(f"🔊 {text}")
        if self.is_mac:
            import subprocess
            subprocess.Popen(['say', text])
        elif self.engine:
            try:
                self.engine.say(text)
                self.engine.runAndWait()
            except Exception as e:
                print(f"Audio error: {e}")


def get_turn_instruction(current_heading, target_heading):
    """Get turn instruction based on heading change"""
    diff = (target_heading - current_heading) % 360
    if diff > 180:
        diff = diff - 360
    
    if abs(diff) < 30: # Increased threshold to avoid noise
        return "Go straight"
    elif diff > 0:
        if diff > 120:
            return "Make a U-turn"
        else:
            return "Turn right"
    else:
        if abs(diff) > 120:
            return "Make a U-turn"
        else:
            return "Turn left"


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


def plot_route(points, title="Trail Tracer Live Overlay"):
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
    plt.gca().set_aspect('equal', adjustable='datalim')
    plt.margins(0.15) # Add 15% margin to prevent going out of bounds
    plt.grid(True)
    plt.tight_layout()
    plt.savefig(PLOT_FILE, dpi=200)
    plt.pause(0.001)


def refresh_outputs(points):
    write_csv(points)
    write_kml(points)
    write_map(points)
    plot_route(points)


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
    """
    Parse ESP32 output formats:
    1. STEP,<step>,<x_m>,<y_m> - Position from Kalman filter
    2. TRACKING START - Session started
    3. GUIDING START - Return to home started
    4. Any other text - Print debug
    """
    parts = line.strip().split(",")
    
    if line.startswith("STEP,"):
        if len(parts) >= 4:
            try:
                step = int(parts[1])
                x = float(parts[2])
                y = float(parts[3])
                
                # Heading will be calculated in run_serial based on movement delta
                heading_deg = 0.0
                dist_home = math.sqrt(x*x + y*y)
                
                lat, lon = meters_to_latlon(x, y, HOME_LAT, HOME_LON)
                return Point(step, heading_deg, x, y, lat, lon, dist_home, 1, -1)
            except ValueError:
                return None
    
    return None


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
    
    # If port is "auto", try to find available ESP32 ports
    if port == "auto":
        if list_ports is None:
            raise RuntimeError("Could not import list_ports from pyserial")
        available_ports = [p.device for p in list_ports.comports()]
        if not available_ports:
            print("\n" + "="*60)
            print("ERROR: No serial ports found!")
            print("="*60)
            print("\nPossible solutions:")
            print("1. Connect your ESP32 to USB")
            print("2. Install CH340 driver (generic ESP32):")
            print("   - Mac: brew install wch-ch34x-usb-serial-driver")
            print("   - Windows: https://github.com/nodemcu/ch340g-usb-driver")
            print("3. Try a different USB cable or port")
            print("\nTo manually specify port, use:")
            print("  python trailtracer_live_overlay.py --mode serial --port COM3")
            print("="*60 + "\n")
            raise RuntimeError("No serial ports found. Please check your ESP32 connection.")
        
        # Prefer /dev/cu.* over /dev/tty.* on Mac, COM ports on Windows
        cu_ports = [p for p in available_ports if "cu." in p or "COM" in p]
        port = cu_ports[0] if cu_ports else available_ports[0]
        print(f"✓ Auto-detected port: {port}")
    
    ser = serial.Serial(port, baudrate=baud, timeout=1)
    points = [Point(0, 0.0, 0.0, 0.0, HOME_LAT, HOME_LON, 0.0, 0, -1)]
    guide = AudioGuide()
    plt.ion()
    
    # Tracking state
    tracking = False
    guiding = False
    return_path = []
    last_announced_heading = None
    
    print(f"✓ Connected on {port} @ {baud}")
    print("Waiting for ESP32 to send tracking data...")

    while True:
        try:
            raw = ser.readline().decode(errors="ignore").strip()
            if not raw:
                continue

            # Log all output
            print(f"[ESP32] {raw}")

            # Check for state changes
            if "TRACKING START" in raw:
                tracking = True
                guiding = False
                guide.speak("Tracking started")
                print("▶ TRACKING STARTED")
                continue
            
            if "GUIDING START" in raw:
                tracking = False
                guiding = True
                return_path = list(reversed(points[1:]))  # Reverse path
                guide.speak("Starting return to home")
                print("🔙 RETURN TO HOME STARTED")
                continue

            # Parse position data
            parsed = parse_esp_line(raw)
            if parsed is None:
                continue
            
            p = parsed

            if tracking:
                # Add point to path
                if p.step > points[-1].step:
                    # Calculate actual heading based on movement vector
                    dx = p.x_m - points[-1].x_m
                    dy = p.y_m - points[-1].y_m
                    
                    if math.hypot(dx, dy) > 0.05: # At least 5cm movement
                        p.heading_deg = math.degrees(math.atan2(dy, dx))
                    else:
                        p.heading_deg = points[-1].heading_deg
                        
                    if last_announced_heading is None:
                        last_announced_heading = p.heading_deg
                        
                    # Check for significant heading change (e.g., 45 degrees) since last announcement
                    diff = (p.heading_deg - last_announced_heading) % 360
                    if diff > 180:
                        diff -= 360
                        
                    if abs(diff) > 45: 
                        turn_instruction = get_turn_instruction(last_announced_heading, p.heading_deg)
                        if "Turn" in turn_instruction or "U-turn" in turn_instruction:
                            guide.speak(turn_instruction)
                            last_announced_heading = p.heading_deg

                    points.append(p)
                    print(f"✓ Step {p.step}: X={p.x_m:.2f}m, Y={p.y_m:.2f}m, Dist={p.dist_home_m:.2f}m, H={p.heading_deg:.0f}°")
                elif p.step == points[-1].step:
                    points[-1] = p
            
            elif guiding and return_path:
                # Following return path
                if len(return_path) > 0:
                    target = return_path[0]
                    dist = math.sqrt((p.x_m - target.x_m)**2 + (p.y_m - target.y_m)**2)
                    
                    if dist < 0.3:  # Reached waypoint
                        return_path.pop(0)
                        guide.speak("Waypoint reached")
                        
                        if not return_path:
                            guide.speak("You are home")
                            guiding = False
                            print("✓ ARRIVED HOME")

            refresh_outputs(points)
            
        except KeyboardInterrupt:
            print("\n⏹ Tracking stopped")
            guide.speak("Tracking stopped")
            break
        except Exception as e:
            print(f"Error: {e}")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--mode", choices=["sim", "serial"], default="sim")
    ap.add_argument("--port", default="auto", help="Serial port (default: auto-detect)")
    ap.add_argument("--baud", type=int, default=115200)
    args = ap.parse_args()

    if args.mode == "sim":
        run_sim()
    else:
        run_serial(args.port, args.baud)


if __name__ == "__main__":
    main()