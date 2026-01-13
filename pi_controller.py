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
import subprocess
import cv2
import numpy as np
from concurrent.futures import ThreadPoolExecutor
from queue import Queue
from threading import Thread
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

detection_lock = Lock()
last_detection_time = 0

# Frame buffer for continuous capture
frame_buffer = Queue(maxsize=5)

# Flag to stop threads gracefully
stop_threads = False

def signal_handler(sig, frame):
    global stop_threads
    print("\nShutting down gracefully...")
    stop_threads = True
    sys.exit(0)

# Register signal handler for graceful shutdown
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

# Fallback to rpicam for frame capture
def capture_frame():
    subprocess.run(["rpicam-still", "-o", "frame.jpg"])
    frame = cv2.imread("frame.jpg")
    return frame

# Update frame capture to handle retries
def capture_frames_continuously():
    cap = None
    while not stop_threads:
        try:
            if cap is None or not cap.isOpened():
                cap = cv2.VideoCapture(0)
                cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
                cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)

            ret, frame = cap.read()
            if ret and not frame_buffer.full():
                frame_buffer.put(frame)
            elif not ret:
                print("Error: Unable to capture frame. Retrying...")
        except Exception as e:
            print(f"Error in frame capture: {e}")
        finally:
            time.sleep(0.1)  # Avoid busy-waiting

    if cap:
        cap.release()

# Update frame processing to check stop flag
def process_frames_live():
    while not stop_threads:
        if not frame_buffer.empty():
            frame = frame_buffer.get()
            detections = detector.detect(frame)
            process_detections(detections, telemetry)
        else:
            print("No frames available in buffer.")
        time.sleep(0.1)  # Avoid busy-waiting

# Start threads without daemon mode
capture_thread = Thread(target=capture_frames_continuously)
processing_thread = Thread(target=process_frames_live)

capture_thread.start()
processing_thread.start()

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

        # Main loop only for telemetry and other tasks
        while True:
            decode_mavlink_message()
            time.sleep(0.1)

    except KeyboardInterrupt:
        print("\nShutting down...")
    except Exception as e:
        print(f"❌ Error: {e}")

# Wait for threads to finish
capture_thread.join()
processing_thread.join()

if __name__ == '__main__':
    main()
