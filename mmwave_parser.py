import serial
import serial.tools.list_ports
import math
import struct
import sys

class RadarLD2451:
    def __init__(self, baudrate=115200):
        # Auto-detect the port before initializing serial
        port = self.auto_detect_port()
        
        self.ser = serial.Serial(port, baudrate, timeout=1)
        self.is_running = True
        self.HEADER = b'\xFD\xFC\xFB\xFA'

    def auto_detect_port(self):
        """Scans for active serial ports on Windows and Linux."""
        print("Scanning for active USB-to-TTL serial ports...")
        ports = serial.tools.list_ports.comports()
        valid_ports = []
        
        for port in ports:
            # Filter for standard Windows and Linux USB serial prefixes
            if "COM" in port.device or "/dev/ttyUSB" in port.device or "/dev/ttyACM" in port.device:
                valid_ports.append(port)
        
        if not valid_ports:
            print("Error: No USB serial ports found! Please check your wiring and ensure the adapter is plugged in.")
            sys.exit(1)
            
        if len(valid_ports) == 1:
            print(f"--> Auto-detected radar on: {valid_ports[0].device} ({valid_ports[0].description})")
            return valid_ports[0].device
        else:
            # If multiple USB serial devices are plugged in, prompt the user
            print("Multiple serial ports found:")
            for i, p in enumerate(valid_ports):
                print(f"  [{i}] {p.device} - {p.description}")
            
            while True:
                try:
                    choice = int(input("Select the port index for the Radar: "))
                    if 0 <= choice < len(valid_ports):
                        return valid_ports[choice].device
                    print("Invalid index. Try again.")
                except ValueError:
                    print("Please enter a valid number.")

    def parse_target(self, target_bytes, target_id):
        if len(target_bytes) < 5:
            return
        
        # Extract raw values based on the C++ protocol
        raw_angle = target_bytes[0]
        distance = target_bytes[1]
        speed_dir = target_bytes[2]
        speed_mag = target_bytes[3]
        snr = target_bytes[4]

        # Apply the math from RadarTarget.cpp
        angle_deg = raw_angle - 128
        velocity = speed_mag * (1 if speed_dir == 0x00 else -1)
        
        # Convert polar coordinates to Cartesian (X, Y) relative to the sensor
        angle_rad = math.radians(angle_deg)
        x = round(distance * math.cos(angle_rad), 2)
        y = round(distance * math.sin(angle_rad), 2)

        print(f"Target {target_id}: Dist={distance}m, Angle={angle_deg}°, Vel={velocity}km/h, X={x}, Y={y}, SNR={snr}")

    def run(self):
        print("\nListening for LD2451 Radar Data...")
        buffer = bytearray()
        
        while self.is_running:
            if self.ser.in_waiting > 0:
                buffer += self.ser.read(self.ser.in_waiting)
                
                # Hunt for the Start Header
                header_idx = buffer.find(self.HEADER)
                
                if header_idx != -1:
                    # Strip everything before the header
                    buffer = buffer[header_idx:]
                    
                    # Ensure we have enough bytes to read the 2-byte length
                    if len(buffer) >= 6: 
                        payload_len = struct.unpack('<H', buffer[4:6])[0]
                        total_frame_len = 6 + payload_len + 4 # Header(4) + Len(2) + Payload + Footer(4)
                        
                        if len(buffer) >= total_frame_len:
                            # We have a complete frame! Extract the payload
                            payload = buffer[6 : 6 + payload_len]
                            
                            object_count = payload[0]
                            print(f"\n--- New Frame: {object_count} Objects Detected ---")
                            
                            # Parse each 5-byte target chunk
                            for i in range(object_count):
                                start_idx = 2 + (i * 5)
                                target_chunk = payload[start_idx : start_idx + 5]
                                self.parse_target(target_chunk, i+1)
                                
                            # Clear the processed frame from the buffer
                            buffer = buffer[total_frame_len:]

if __name__ == "__main__":
    # The initialization no longer requires a hardcoded port
    radar = RadarLD2451(baudrate=115200) 
    try:
        radar.run()
    except KeyboardInterrupt:
        radar.ser.close()
        print("\nRadar connection closed.")