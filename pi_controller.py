#!/usr/bin/env python3
"""
Raspberry Pi Controller - Minimal Version
Runs a CV detection model, geotags detections, and sends coordinates over telemetry to GCS.
"""


import os
import time
import json
from datetime import datetime
import subprocess
import cv2
import numpy as np
import signal
import sys

# Import necessary modules for CV detection and geolocation
from modules.yellow_crop_detector import YellowCropDetector, CropDetection
from modules.geolocation import GeoLocationCalculator
from modules.mavlink_detection_sender import MAVLinkDetectionSender
from pymavlink import mavutil

# Load configuration from config.json
def load_config():
    try:
        with open('config.json', 'r') as f:
            config = json.load(f)
        return config
    except Exception as e:
        print(f"Error loading config.json: {e}")
        return {}

config = load_config()

# Configuration
PI_ID = config.get('pi_id', os.environ.get('PI_ID', 'pi_001'))
DETECTION_ENABLED = config.get('detection', {}).get('enabled', True)
DETECTION_COOLDOWN = config.get('detection', {}).get('detection_cooldown', 3.0)

# Initialize components
geo_calculator = GeoLocationCalculator()
detector = YellowCropDetector(config=config)

# Establish a connection to the MAVLink system
try:
    print("Attempting to connect to MAVLink system...")
    master = mavutil.mavlink_connection(config['pixhawk']['connection_string'], baud=config['pixhawk']['baud_rate'])
    print("Connection established. Waiting for heartbeat...")
    master.wait_heartbeat(timeout=30)
    print("Heartbeat received!")
except Exception as e:
    print(f"Error connecting to MAVLink system: {e}")

# Initialize MAVLinkDetectionSender with the master connection
mavlink_sender = MAVLinkDetectionSender(master)


last_detection_time = 0

def signal_handler(sig, frame):
    print("\nShutting down gracefully...")
    sys.exit(0)

signal.signal(signal.SIGINT, signal_handler)

def process_detections(detections, telemetry):
    """Process detections, geotag them, and send coordinates over telemetry."""
    global last_detection_time

    if not detections:
        return

    current_time = time.time()
    if current_time - last_detection_time < DETECTION_COOLDOWN:
        print("Cooldown active, skipping detections.")
        return

    last_detection_time = current_time

    for detection in detections:
        try:
            # Debugging: Log telemetry data
            print(f"[DEBUG] Telemetry data: {telemetry}")

            # Ensure telemetry contains required fields
            altitude_agl = telemetry.get('altitude', 0.0)  # Default to 0.0 if missing
            heading_deg = telemetry.get('heading', 0.0)   # Default to 0.0 if missing

            # Geotag detection

            gps_coords = geo_calculator.calculate_coordinates(
                pixel_x=detection.centroid[0],
                pixel_y=detection.centroid[1],
                drone_lat=telemetry['latitude'],
                drone_lon=telemetry['longitude'],
                altitude_agl=altitude_agl,
                heading_deg=heading_deg  # Default heading to 0.0 if not provided
            )
            lat, lon, alt = gps_coords

            # Prepare detection data
            detection_data = {
                'pi_id': PI_ID,
                'detection_id': detection.detection_id,
                'latitude': lat,
                'longitude': lon,
                'altitude': alt,
                'timestamp': datetime.now().isoformat()
            }

            # Send detection over telemetry
            mavlink_sender.send_detection(detection_data)
            print(f"Detection sent: {detection_data}")

        except Exception as e:
            print(f"Error processing detection: {e}")

# Function to decode MAVLink messages
def decode_mavlink_message():
    try:
        message = master.recv_match(blocking=True, timeout=5)
        if message:
            print(f"Received MAVLink message: {message.to_dict()}")
        else:
            print("No message received within timeout.")
    except Exception as e:
        print(f"Error decoding MAVLink message: {e}")


# Headless rpicam-still frame capture (no -w/-h, resize in Python)
def capture_frame_and_resize(width=640, height=480):
    temp_file = 'frame.jpg'
    cmd = [
        'rpicam-still',
        '-o', temp_file,
        '-t', '1',
        '-n',
        '--immediate',
        '--nopreview'
    ]
    try:
        subprocess.run(cmd, check=True, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        frame = cv2.imread(temp_file)
        if frame is not None:
            os.remove(temp_file)
            frame = cv2.resize(frame, (width, height))
        return frame
    except Exception as e:
        print(f"rpicam-still error: {e}")
        return None


def main():
    """Main function to run the headless yellow detection loop."""
    print(f"🚀 Starting Pi Controller for {PI_ID}")

    if not DETECTION_ENABLED:
        print("Detection is disabled in the configuration.")
        return

    print("🌾 Detection enabled. Starting detection immediately...")

    # Simulate telemetry data (replace with actual telemetry source)
    telemetry = {
        'latitude': 0.0,
        'longitude': 0.0,
        'altitude': 0.0
    }


    frame_count = 0
    save_dir = 'output_frames_detected'
    os.makedirs(save_dir, exist_ok=True)

    try:
        while True:
            # 1. Capture frame
            frame = capture_frame_and_resize(width=640, height=480)
            print(f"[DEBUG] Frame {frame_count}: Capture {'OK' if frame is not None else 'FAILED'}")
            if frame is None:
                print("Failed to capture frame with rpicam-still.")
                continue

            # 2. Run yellow detection
            detections = detector.detect(frame)
            print(f"[DEBUG] Frame {frame_count}: {len(detections)} detections found")

            # 3. Process detections (geotag, send, log)
            process_detections(detections, telemetry)

            # 4. Optionally, save annotated frame for review
            if detections:
                annotated = detector.visualize_detections(frame, detections, show_info=True)
                out_path = os.path.join(save_dir, f'frame_{frame_count:05d}.jpg')
                cv2.imwrite(out_path, annotated)
                print(f"Saved detection frame: {out_path}")
            frame_count += 1

            # 5. Adjust sleep for drone speed (2m/s):
            # At 2m/s, 5 fps gives a detection every 0.4m. Adjust as needed.
            time.sleep(0.2)

    except KeyboardInterrupt:
        print("\nShutting down...")
    except Exception as e:
        print(f"❌ Error: {e}")

if __name__ == '__main__':
    main()
