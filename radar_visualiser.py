import serial
import serial.tools.list_ports
import math
import struct
import sys
import threading
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import matplotlib.patheffects as pe

# Shared memory to pass data from the Serial Thread to the GUI Thread
latest_targets = []
data_lock = threading.Lock()

class RadarLD2451:
    def __init__(self, baudrate=115200):
        port = self.auto_detect_port()
        self.ser = serial.Serial(port, baudrate, timeout=1)
        self.is_running = True
        self.HEADER = b'\xF4\xF3\xF2\xF1' 

    def auto_detect_port(self):
        print("Scanning for active USB-to-TTL serial ports...")
        ports = serial.tools.list_ports.comports()
        valid_ports = [p for p in ports if "COM" in p.device or "/dev/ttyUSB" in p.device or "/dev/ttyACM" in p.device]
        
        if not valid_ports:
            print("Error: No USB serial ports found! Check wiring.")
            sys.exit(1)
            
        if len(valid_ports) == 1:
            print(f"--> Auto-detected radar on: {valid_ports[0].device}")
            return valid_ports[0].device
        else:
            print("Multiple serial ports found:")
            for i, p in enumerate(valid_ports):
                print(f"  [{i}] {p.device} - {p.description}")
            choice = int(input("Select the port index for the Radar: "))
            return valid_ports[choice].device

    def parse_target(self, target_bytes):
        if len(target_bytes) < 5:
            return None
        
        raw_angle = target_bytes[0]
        distance = target_bytes[1]
        speed_dir = target_bytes[2]
        speed_mag = target_bytes[3]
        snr = target_bytes[4]

        angle_deg = raw_angle - 128
        velocity = speed_mag * (1 if speed_dir == 0x00 else -1)
        
        return {"distance": distance, "angle": angle_deg, "velocity": velocity, "snr": snr}

    def read_serial_loop(self):
        print("Listening for LD2451 Radar Data...")
        buffer = bytearray()
        
        while self.is_running:
            if self.ser.in_waiting > 0:
                buffer += self.ser.read(self.ser.in_waiting)
                header_idx = buffer.find(self.HEADER)
                
                if header_idx != -1:
                    buffer = buffer[header_idx:]
                    if len(buffer) >= 6: 
                        payload_len = struct.unpack('<H', buffer[4:6])[0]
                        total_frame_len = 6 + payload_len + 4 
                        
                        if len(buffer) >= total_frame_len:
                            payload = buffer[6 : 6 + payload_len]
                            frame_targets = []
                            
                            if len(payload) >= 2:
                                object_count = payload[0]
                                for i in range(min(object_count, 3)):
                                    start_idx = 2 + (i * 5)
                                    if start_idx + 5 <= len(payload):
                                        target_data = self.parse_target(payload[start_idx : start_idx + 5])
                                        if target_data:
                                            frame_targets.append(target_data)
                            
                            with data_lock:
                                global latest_targets
                                latest_targets = frame_targets
                                
                            buffer = buffer[total_frame_len:]
                else:
                    if len(buffer) > 1024:
                        buffer = buffer[-4:]

    def stop(self):
        self.is_running = False
        self.ser.close()

# --- Matplotlib GUI Setup (TIGHT DARK MODE) ---
plt.style.use('dark_background')

fig = plt.figure(figsize=(14, 9))
fig.canvas.manager.set_window_title('UAV mmWave Radar Scope')

# 1. THE RADAR SCOPE
ax = fig.add_axes([0.0, 0.0, 1.0, 1.0], polar=True)
ax.set_thetamin(-90)
ax.set_thetamax(90)
ax.set_theta_zero_location('N') 

# UPDATE: Set max distance to 100 meters based on LD2451 specs
ax.set_rlim(0, 100) 

# Grid styling
ax.grid(color="#7A7A7A", linestyle='--', linewidth=0.5)
ax.tick_params(colors='lightgray')

# FIX: Set labeled major grid lines every 30 degrees
ax.set_thetagrids(np.arange(-90, 91, 30))

# FIX: Add unlabelled minor grid lines every 10 degrees (Requires radians)
ax.set_xticks(np.radians(np.arange(-90, 91, 10)), minor=True)
ax.grid(which='minor', color="#797979", linestyle=':', linewidth=0.5)

# UPDATE: Smooth radial gradient layer scaled to 100m
theta_grid, r_grid = np.meshgrid(np.linspace(-np.pi/2, np.pi/2, 40), np.linspace(0, 100, 40))
val_grid = r_grid / 100.0 
contour = ax.contourf(theta_grid, r_grid, -val_grid, 30, cmap=plt.cm.Greens_r, alpha=0.3)

# Scatter plot initialization
scatter = ax.scatter([], [], c=[], cmap='coolwarm', s=200, edgecolors='white', linewidths=0.5, vmin=-20, vmax=20)

# 2. THE VELOCITY BAR
cbar_ax = fig.add_axes([0.2, 0.9, 0.6, 0.02])
cbar = plt.colorbar(scatter, cax=cbar_ax, orientation='horizontal')
cbar.set_label('Velocity (km/h)\n[Red=Approaching, Blue=Receding]', color='white', labelpad=10, fontsize=8)
cbar.ax.yaxis.set_tick_params(color='white', labelcolor='white')

# 3. THE 3-COLUMN DATA TABLE
col_texts = []
x_positions = [0.30, 0.50, 0.70]

for x_pos in x_positions:
    t = fig.text(x_pos, 0.15, "", ha='center', va='top', color='lightgreen', fontsize=10, family='monospace', linespacing=1.5,)
    col_texts.append(t)

annotations = []

def update_gui(frame):
    global annotations
    with data_lock:
        targets = latest_targets.copy()
    
    for ann in annotations:
        ann.remove()
    annotations.clear()

    if targets:
        theta = [math.radians(-t["angle"]) for t in targets]
        r = [t["distance"] for t in targets]
        colors = [t["velocity"] for t in targets]
        sizes = [max(300, t["snr"] * 5) for t in targets]
        
        scatter.set_offsets(list(zip(theta, r)))
        scatter.set_array(np.array(colors))
        scatter.set_sizes(sizes)
        
        for i, t in enumerate(targets):
            ann = ax.text(theta[i], r[i], str(i + 1), 
                          color='white', fontsize=11, fontweight='bold', ha='center', va='center',
                          path_effects=[pe.withStroke(linewidth=3, foreground="black")])
            annotations.append(ann)
            
    else:
        scatter.set_offsets(np.empty((0, 2)))
        
    for i in range(3):
        if i < len(targets):
            t = targets[i]
            text_str = (
                f"--- OBJECT {i+1} ---\n"
                f"Dist : {t['distance']:>5.1f} m\n"
                f"Angle: {t['angle']:>5} °\n"
                f"Vel  : {t['velocity']:>5.1f} km/h"
            )
        else:
            text_str = (
                f"--- OBJECT {i+1} ---\n"
                f"Dist :   N/A   \n"
                f"Angle:   N/A   \n"
                f"Vel  :   N/A   "
            )
        col_texts[i].set_text(text_str)
    
    return scatter, *col_texts

if __name__ == "__main__":
    radar = RadarLD2451()
    serial_thread = threading.Thread(target=radar.read_serial_loop, daemon=True)
    serial_thread.start()
    
    ani = animation.FuncAnimation(fig, update_gui, interval=50, blit=False, cache_frame_data=False)
    
    try:
        plt.show() 
    except KeyboardInterrupt:
        pass
    finally:
        radar.stop()