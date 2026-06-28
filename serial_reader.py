import serial
import threading
import serial.tools.list_ports
import re
import os
import csv
import sys
import time
import argparse
from datetime import datetime
from PyQt5.QtCore import pyqtSignal, QObject

class SerialReader(QObject):
    dataReceived = pyqtSignal()

    def __init__(self):
        QObject.__init__(self)
        self.ser = None
        self.thread = None
        self.running = False
        self.selected_port = None
        self.csv_file = None
        self.csv_writer = None
        self.current_sf = None
        self.current_bw = None
        self.current_tp = None
        self.distance = None
        self.base_dir = "test_data"
        self.last_message_time = None  # Track time of last message for calculating time between messages
        self.csv_skipped_exists = False  # Flag to track if CSV was skipped because file already exists
        
        # Data attributes for CAN payload
        self.velocity = 0.0
        self.distance_travelled = 0.0
        self.battery_volt = 0.0
        self.battery_current = 0.0
        self.battery_cell_LOW_volt = 0.0
        self.battery_cell_HIGH_volt = 0.0
        self.battery_cell_AVG_volt = 0.0
        self.battery_cell_LOW_temp = 0.0
        self.battery_cell_HIGH_temp = 0.0
        self.battery_cell_AVG_temp = 0.0
        self.battery_cell_ID_HIGH_temp = 0.0
        self.battery_cell_ID_LOW_temp = 0.0
        self.BMS_temp = 0.0
        self.motor_current = 0.0
        self.motor_temp = 0.0
        self.motor_controller_temp = 0.0
        self.MPPT1_watt = 0.0
        self.MPPT2_watt = 0.0
        self.MPPT3_watt = 0.0
        self.MPPT_total_watt = 0.0
        
        # List of available data attributes
        self.available_data = [
            'velocity', 'distance_travelled', 'battery_volt', 'battery_current',
            'battery_cell_LOW_volt', 'battery_cell_HIGH_volt', 'battery_cell_AVG_volt',
            'battery_cell_LOW_temp', 'battery_cell_HIGH_temp', 'battery_cell_AVG_temp',
            'battery_cell_ID_HIGH_temp', 'battery_cell_ID_LOW_temp', 'BMS_temp',
            'motor_current', 'motor_temp', 'motor_controller_temp',
            'MPPT1_watt', 'MPPT2_watt', 'MPPT3_watt', 'MPPT_total_watt'
        ]
        
        # Initialize latest values and history dictionaries
        self.latest_values = {var: 0.0 for var in self.available_data}
        # History with 100 data points per variable
        self.history = {var: [0.0] * 100 for var in self.available_data}

    def start(self, port=None):
        """
        Start the serial reader.
        
        Args:
            port (str, optional): COM port to use (e.g., 'COM3'). 
                                  If None, will auto-select if only one port is available.
        """
        # Use provided port or existing selected_port
        if port:
            self.selected_port = port
            print(f"[DEBUG] Using specified COM port: {self.selected_port}")
        elif not self.selected_port:
            # Auto-select the only available COM port if none explicitly chosen
            ports = [p.device for p in serial.tools.list_ports.comports()]
            if len(ports) == 1:
                self.selected_port = ports[0]
                print(f"[DEBUG] Only one COM port available, auto-selecting {self.selected_port}")
            else:
                print("[DEBUG] No COM port selected or multiple ports available.")
                print(f"[DEBUG] Available ports: {', '.join(ports)}")
                print("[DEBUG] Use set_port('COMx') or start('COMx') to specify a port.")
                return
        try:
            self.ser = serial.Serial(self.selected_port, 115200)
            self.running = True
            self.thread = threading.Thread(target=self.read_serial_data, daemon=True)
            self.thread.start()
            print(f"[DEBUG] Serial reader started on {self.selected_port}.")
        except serial.SerialException as e:
            print(f"[DEBUG] Failed to open port {self.selected_port}: {e}")

    def stop(self):
        self.running = False
        if self.thread:
            self.thread.join()
        if self.ser and self.ser.is_open:
            self.ser.close()
        if self.csv_file:
            self.csv_file.close()
        print("[DEBUG] Serial reader stopped.")

    def set_port(self, port):
        """Set the COM port to use for serial communication."""
        self.selected_port = port
        print(f"[DEBUG] COM port set to {port}")

    def set_distance(self, distance):
        self.distance = distance
        print(f"[DEBUG] Distance set to {distance}m")

        # Create a new csv file when we change settings.
    def create_new_csv(self, sf, bw, tp):
        # Close previous CSV if open
        if self.csv_file:
            try:
                self.csv_file.close()
                print("[DEBUG] Closed previous CSV file")
            except Exception as e:
                print(f"[DEBUG] Error closing previous CSV: {e}")
        
        if not self.distance:
            print("[DEBUG] No distance set, cannot create CSV")
            return
        
        # Create distance folder (e.g., distance_6.25m, distance_12.5m)
        distance_folder = os.path.join(self.base_dir, f"distance_{self.distance}m")
        os.makedirs(distance_folder, exist_ok=True)
        
        # Create CSV filename based on configuration (e.g., SF7_BW125000_TP10.csv)
        filename = f"SF{sf}_BW{bw}_TP{tp}.csv"
        filepath = os.path.join(distance_folder, filename)
        
        # Check if file already exists
        if os.path.exists(filepath):
            print(f"[DEBUG] CSV file already exists: {filepath}")
            print(f"[DEBUG] Skipping creation to avoid overwriting existing data")
            # Set csv_file and csv_writer to None so we don't write to it
            self.csv_file = None
            self.csv_writer = None
            self.csv_skipped_exists = True
            return
        
        # Reset flag when creating new file
        self.csv_skipped_exists = False
        
        try:
            self.csv_file = open(filepath, 'w', newline='')
            self.csv_writer = csv.writer(self.csv_file)
            self.csv_writer.writerow(['timestamp', 'rssi', 'kalman_rssi', 'sma_rssi', 'time_between_messages_ms', 'payload'])
            self.csv_file.flush()
            print(f"[DEBUG] Created new CSV: {filepath}")
            print(f"[DEBUG] CSV file is open: {not self.csv_file.closed}")
        except Exception as e:
            print(f"[DEBUG] Error creating CSV file: {e}")
            self.csv_file = None
            self.csv_writer = None

    def read_serial_data(self):
        print("[DEBUG] Serial reading thread running.")
        rssi_value = None
        kalman_rssi_value = None
        sma_rssi_value = None
        payload_data = None
        first_line_processed = False
        
        while self.running:
            # Add small delay to prevent CPU spinning
            if not (self.ser and self.ser.in_waiting > 0):
                time.sleep(0.01)
                continue
            if self.ser and self.ser.in_waiting > 0:
                line = self.ser.readline().decode('utf-8', errors='ignore').strip()
                print(f"[DEBUG] Raw line read: '{line}'")

                # Check for parameter change - format: "Applied parameters from sender -> SF: %d, BW: %ld Hz, TP: %d dBm"
                # This message indicates the config for UPCOMING messages, so create CSV immediately
                if 'Applied parameters from sender' in line:
                    match = re.search(r'SF:\s*(\d+).*BW:\s*(\d+).*TP:\s*(\d+)', line)
                    if match:
                        new_sf = int(match.group(1))
                        new_bw = int(match.group(2))
                        new_tp = int(match.group(3))
                        
                        # Only create new CSV if config actually changed
                        if (self.current_sf != new_sf or self.current_bw != new_bw or self.current_tp != new_tp):
                            self.current_sf = new_sf
                            self.current_bw = new_bw
                            self.current_tp = new_tp
                            self.create_new_csv(self.current_sf, self.current_bw, self.current_tp)
                            # Reset last message time when new CSV is created
                            self.last_message_time = None
                            first_line_processed = True
                            print(f"[DEBUG] New config detected: SF={self.current_sf}, BW={self.current_bw}, TP={self.current_tp}")
                        else:
                            print(f"[DEBUG] Config unchanged, keeping current CSV")
                    continue
                
                # Check if first line is a CFG message (format: "CFG sf=X sbw=Y tp=Z" or "LoRa data: CFG sf=X...")
                # This tells us the initial configuration for the first messages
                if not first_line_processed:
                    # Check for CFG in LoRa data line
                    if 'LoRa data:' in line and ('CFG' in line or 'cfg' in line):
                        match = re.search(r'sf\s*=\s*(\d+).*sbw\s*=\s*(\d+).*tp\s*=\s*(\d+)', line, re.IGNORECASE)
                        if match:
                            self.current_sf = int(match.group(1))
                            self.current_bw = int(match.group(2))
                            self.current_tp = int(match.group(3))
                            self.create_new_csv(self.current_sf, self.current_bw, self.current_tp)
                            self.last_message_time = None
                            first_line_processed = True
                            print(f"[DEBUG] Detected initial config from CFG message: SF={self.current_sf}, BW={self.current_bw}, TP={self.current_tp}")
                            continue
                    # Also check for standalone CFG line
                    elif line.startswith('CFG ') or line.startswith('cfg '):
                        match = re.search(r'sf\s*=\s*(\d+).*sbw\s*=\s*(\d+).*tp\s*=\s*(\d+)', line, re.IGNORECASE)
                        if match:
                            self.current_sf = int(match.group(1))
                            self.current_bw = int(match.group(2))
                            self.current_tp = int(match.group(3))
                            self.create_new_csv(self.current_sf, self.current_bw, self.current_tp)
                            self.last_message_time = None
                            first_line_processed = True
                            print(f"[DEBUG] Detected initial config from CFG message: SF={self.current_sf}, BW={self.current_bw}, TP={self.current_tp}")
                            continue

                # Parse new single-line format: "DATA|LoRa_data:<payload>|RSSI:<value>|Kalman_RSSI:<value>|SMA_RSSI:<value>"
                if line.startswith('DATA|'):
                    try:
                        # Split by | to get each field
                        parts = line.split('|')
                        for part in parts:
                            if part.startswith('LoRa_data:'):
                                payload_data = part.split(':', 1)[1].strip()
                            elif part.startswith('RSSI:'):
                                rssi_value = float(part.split(':', 1)[1].strip())
                            elif part.startswith('Kalman_RSSI:'):
                                kalman_rssi_value = float(part.split(':', 1)[1].strip())
                            elif part.startswith('SMA_RSSI:'):
                                sma_rssi_value = float(part.split(':', 1)[1].strip())
                        
                        # With single-line format, we have all data immediately
                        # Fall through to processing logic below (don't continue)
                    except (ValueError, IndexError) as e:
                        print(f"[DEBUG] Error parsing DATA line: {e}, line: {line}")
                        # Reset values on error
                        rssi_value = None
                        kalman_rssi_value = None
                        sma_rssi_value = None
                        payload_data = None
                        continue
                
                # Legacy parsing for old multi-line format (for backward compatibility)
                elif line.startswith('RSSI:') and not line.startswith('DATA|'):
                    try:
                        rssi_value = float(line.split(':')[1].strip())
                    except ValueError:
                        pass
                    continue
                
                elif line.startswith('Kalman RSSI:'):
                    try:
                        kalman_rssi_value = float(line.split(':')[1].strip())
                    except ValueError:
                        pass
                    continue

                elif line.startswith('SMA RSSI'):
                    try:
                        # Handle both semicolon and colon
                        value_str = line.split(':')[-1] if ':' in line else line.split(';')[-1]
                        sma_rssi_value = float(value_str.strip())
                    except ValueError:
                        pass
                    continue

                elif 'LoRa data:' in line and not line.startswith('DATA|'):
                    payload_data = line.split('LoRa data:')[1].strip()
                    continue
                
                else:
                    # Not a data line, skip to next iteration
                    continue

                # Process and save when we have all required data (RSSI, Kalman RSSI, and payload)
                # Note: SMA RSSI is optional, but we wait for the complete message set
                # The receiver sends: LoRa data, RSSI, Kalman RSSI, SMA RSSI in sequence
                # We process when we have RSSI, Kalman RSSI, and payload (SMA is optional)
                if rssi_value is not None and kalman_rssi_value is not None and payload_data:
                    # Calculate time between messages
                    current_time = datetime.now()
                    time_between_ms = ''
                    if self.last_message_time is not None:
                        time_diff = current_time - self.last_message_time
                        time_between_ms = int(time_diff.total_seconds() * 1000)  # Convert to milliseconds
                    self.last_message_time = current_time
                    
                    # Create CSV file if we have distance but no CSV file yet (using current or default config)
                    if self.distance and not self.csv_writer:
                        # Use current config if available, otherwise use defaults
                        sf = self.current_sf if self.current_sf is not None else 7
                        bw = self.current_bw if self.current_bw is not None else 62500
                        tp = self.current_tp if self.current_tp is not None else 2
                        self.create_new_csv(sf, bw, tp)
                        print(f"[DEBUG] Created CSV file with initial config: SF={sf}, BW={bw}, TP={tp}")
                    
                    # Save to CSV if writer is available
                    if self.csv_writer:
                        timestamp = current_time.isoformat()
                        # Use SMA RSSI if available, otherwise use empty string
                        sma_val = sma_rssi_value if sma_rssi_value is not None else ''
                        try:
                            self.csv_writer.writerow([timestamp, rssi_value, kalman_rssi_value, sma_val, time_between_ms, payload_data])
                            self.csv_file.flush()
                            print(f"[DEBUG] Saved row {self.csv_file.tell()} bytes: RSSI={rssi_value:.2f}, Kalman={kalman_rssi_value:.2f}, Time={time_between_ms}ms")
                        except Exception as e:
                            print(f"[DEBUG] Error writing to CSV: {e}")
                    else:
                        if not self.distance:
                            print("[DEBUG] No distance set, skipping CSV save. Use set_distance() to enable CSV logging.")
                        elif self.csv_skipped_exists:
                            # Only print this message once per config to avoid spam
                            pass  # Message already printed when CSV was skipped
                        else:
                            print("[DEBUG] CSV writer not available, data not saved (this may happen before config is received)")

                    # Process payload data for GUI (if it contains numeric data)
                    # Try to parse as numeric data, but don't skip if it fails
                    parts = payload_data.split()
                    # Truncate or pad to exactly 20 items
                    if len(parts) > 20:
                        parts = parts[:20]
                    elif len(parts) < 20:
                        parts += ['0.0'] * (20 - len(parts))

                    # Convert to floats (try each part, use 0.0 if conversion fails)
                    values = []
                    for p in parts:
                        try:
                            values.append(float(p))
                        except ValueError:
                            values.append(0.0)

                    # Unpack values into attributes
                    (
                        self.velocity,
                        self.distance_travelled,
                        self.battery_volt,
                        self.battery_current,
                        self.battery_cell_LOW_volt,
                        self.battery_cell_HIGH_volt,
                        self.battery_cell_AVG_volt,
                        self.battery_cell_LOW_temp,
                        self.battery_cell_HIGH_temp,
                        self.battery_cell_AVG_temp,
                        self.battery_cell_ID_HIGH_temp,
                        self.battery_cell_ID_LOW_temp,
                        self.BMS_temp,
                        self.motor_current,
                        self.motor_temp,
                        self.motor_controller_temp,
                        self.MPPT1_watt,
                        self.MPPT2_watt,
                        self.MPPT3_watt,
                        self.MPPT_total_watt
                    ) = values

                    # Update latest values
                    for var in self.available_data:
                        self.latest_values[var] = getattr(self, var)

                    # Update history
                    for key in self.available_data:
                        self.history[key].pop(0)
                        self.history[key].append(getattr(self, key))

                    # Emit signal
                    self.dataReceived.emit()

                    # Reset values after processing to prepare for next message
                    rssi_value = None
                    kalman_rssi_value = None
                    sma_rssi_value = None
                    payload_data = None


def main():
    """Main function to run serial reader as standalone script."""
    parser = argparse.ArgumentParser(description='LoRa Serial Reader - Reads and logs LoRa receiver data')
    parser.add_argument('--port', '-p', type=str, help='COM port to use (e.g., COM3)')
    parser.add_argument('--distance', '-d', type=float, help='Distance in meters (e.g., 6.25)')
    parser.add_argument('--list-ports', '-l', action='store_true', help='List available COM ports and exit')
    
    args = parser.parse_args()
    
    # List ports if requested
    if args.list_ports:
        ports = [p.device for p in serial.tools.list_ports.comports()]
        if ports:
            print("Available COM ports:")
            for port in ports:
                print(f"  {port}")
        else:
            print("No COM ports found.")
        return
    
    # Create reader instance
    reader = SerialReader()
    
    # Set distance if provided
    if args.distance:
        reader.set_distance(args.distance)
        print(f"Distance set to {args.distance}m")
    else:
        print("Warning: No distance specified. CSV files will not be created until distance is set.")
        print("Use --distance or -d to set distance, or call set_distance() method.")
    
    # Start reader
    try:
        if args.port:
            reader.start(args.port)
        else:
            reader.start()
        
        if not reader.running:
            print("Failed to start serial reader.")
            return
        
        print("\n" + "="*60)
        print("Serial Reader Started")
        print("="*60)
        print(f"Port: {reader.selected_port}")
        if reader.distance:
            print(f"Distance: {reader.distance}m")
        print("Press Ctrl+C to stop...")
        print("="*60 + "\n")
        
        # Keep running until interrupted
        try:
            while reader.running:
                time.sleep(0.1)
        except KeyboardInterrupt:
            print("\n\nStopping serial reader...")
            reader.stop()
            print("Serial reader stopped.")
    
    except Exception as e:
        print(f"Error: {e}")
        reader.stop()
        sys.exit(1)


if __name__ == "__main__":
    main()