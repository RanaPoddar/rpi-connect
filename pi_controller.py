#!/usr/bin/env python3
"""
Raspberry Pi Controller - Minimal Version
Runs a CV detection model, geotags detections, and sends coordinates over telemetry to GCS.
"""

import os
import time
import json
from datetime import datetime
from threading import Lock

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
master = mavutil.mavlink_connection('/dev/serial0', baud=57600)  # Replace with your connection string
master.wait_heartbeat()  # Wait for the heartbeat signal to confirm connection
print("✅ Connected to MAVLink system")

# Initialize MAVLinkDetectionSender with the master connection
mavlink_sender = MAVLinkDetectionSender(master)

detection_lock = Lock()
last_detection_time = 0

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
            # Geotag detection
            gps_coords = geo_calculator.calculate_coordinates(
                telemetry['latitude'], telemetry['longitude'], telemetry['altitude'], detection
            )

            # Prepare detection data
            detection_data = {
                'pi_id': PI_ID,
                'detection_id': detection.detection_id,
                'latitude': gps_coords['latitude'],
                'longitude': gps_coords['longitude'],
                'altitude': gps_coords['altitude'],
                'timestamp': datetime.now().isoformat()
            }

            # Send detection over telemetry
            mavlink_sender.send_detection(detection_data)
            print(f"Detection sent: {detection_data}")

        except Exception as e:
            print(f"Error processing detection: {e}")

def main():
    """Main function to run the detection loop."""
    print(f"🚀 Starting Pi Controller for {PI_ID}")

    if not DETECTION_ENABLED:
        print("Detection is disabled in the configuration.")
        return

    print("🌾 Detection enabled. Starting detection immediately...")

    try:
        # Simulate telemetry data (replace with actual telemetry source)
        telemetry = {
            'latitude': 0.0,
            'longitude': 0.0,
            'altitude': 0.0
        }

        while True:
            print("🔄 Running detection loop...")
            print(f"📡 Current telemetry: {telemetry}")

            # Run detection
            with detection_lock:
                frame = None  # Replace with actual frame capture logic
                detections = detector.detect(frame)
                print(f"📸 Detections: {detections}")
                process_detections(detections, telemetry)

            time.sleep(1)  # Adjust loop frequency as needed

    except KeyboardInterrupt:
        print("\nShutting down...")
    except Exception as e:
        print(f"❌ Error: {e}")

if __name__ == '__main__':
    main()
