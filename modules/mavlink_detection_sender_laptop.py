"""
Laptop-friendly MAVLink Detection Sender

Provides a drop-in, laptop-testable variant of `mavlink_detection_sender.py`.
- Uses UDP (`udp:127.0.0.1:14550`) by default to talk to local GCS (e.g., QGroundControl)
- Falls back to a MockMaster that logs STATUSTEXT messages when no MAVLink connection available
- Includes a simple webcam demo that runs the `YellowCropDetector` and sends detections

Usage:
  python modules/mavlink_detection_sender_laptop.py [--udp udp:127.0.0.1:14550]

"""

import time
import json
import threading
import argparse
from typing import Dict, List, Optional
from datetime import datetime

import cv2

try:
    from pymavlink import mavutil
    PYMAVLINK_AVAILABLE = True
except Exception:
    PYMAVLINK_AVAILABLE = False

from modules.yellow_crop_detector import YellowCropDetector, CropDetection


class MockMAV:
    """Very small mock object with `mav.statustext_send` to emulate pymavlink master."""

    class _mav:
        @staticmethod
        def statustext_send(severity, text):
            # text may be bytes; decode for pretty printing
            try:
                t = text.decode('utf-8') if isinstance(text, (bytes, bytearray)) else text
            except Exception:
                t = str(text)
            print(f"[MOCK MAV STATUSTEXT] severity={severity} text={t}")

    mav = _mav()


class MAVLinkDetectionSenderLaptop:
    """Laptop-friendly MAVLink detection sender.

    Methods mirror the original `MAVLinkDetectionSender` but tolerate absence of a serial
    connection by using UDP or the MockMAV above.
    """

    SEVERITY_INFO = 6
    SEVERITY_NOTICE = 5
    SEVERITY_WARNING = 4

    def __init__(self, master=None, enabled: bool = True):
        self.master = master or (MockMAV() if not PYMAVLINK_AVAILABLE else None)
        self.enabled = enabled
        self.detection_count = 0
        self.min_send_interval = 0.5
        self.last_send_time = 0.0

    def _send_statustext(self, text: str, severity: int = SEVERITY_INFO):
        text = text[:50]
        try:
            if hasattr(self.master, 'mav'):
                # pymavlink style
                self.master.mav.statustext_send(severity, text.encode('utf-8'))
            else:
                # fallback mock prints
                print(f"[STATUSTEXT] {severity}: {text}")

            self.last_send_time = time.time()
        except Exception as e:
            print(f"⚠️  STATUSTEXT send error (laptop): {e}")

    def send_detection(self, detection: Dict) -> bool:
        if not self.enabled:
            return False
        try:
            det_id = detection.get('detection_id', 'unknown')
            lat = detection.get('latitude', 0.0)
            lon = detection.get('longitude', 0.0)
            confidence = detection.get('confidence', 0.0)
            area = detection.get('detection_area', 0.0)

            message = f"DET|{det_id[:15]}|{lat:.6f}|{lon:.6f}|{confidence:.2f}|{int(area)}"
            # Rate limiting
            if time.time() - self.last_send_time < self.min_send_interval:
                time.sleep(self.min_send_interval)

            self._send_statustext(message, self.SEVERITY_INFO)
            self.detection_count += 1
            return True
        except Exception as e:
            print(f"❌ Failed to send detection (laptop): {e}")
            return False


def load_config(path: str = 'config.json') -> dict:
    try:
        with open(path, 'r') as f:
            return json.load(f)
    except Exception:
        return {}


def run_webcam_demo(udp_addr: Optional[str] = None, config_path: str = 'config.json'):
    """Open webcam, detect yellow crops, and send detections over MAVLink/Mock."""
    config = load_config(config_path)
    detector = YellowCropDetector(config=config)

    # Setup MAVLink master (UDP if possible)
    master = None
    if PYMAVLINK_AVAILABLE and udp_addr:
        try:
            master = mavutil.mavlink_connection(udp_addr)
            # don't block waiting for heartbeat in laptop tests
            print(f"✅ Connected to MAVLink via {udp_addr}")
        except Exception as e:
            print(f"⚠️  Could not open UDP MAVLink ({udp_addr}): {e}")
            master = None

    if master is None:
        master = MockMAV()

    sender = MAVLinkDetectionSenderLaptop(master=master, enabled=True)

    cap = cv2.VideoCapture(0)
    if not cap.isOpened():
        print("❌ Could not open webcam")
        return

    print("🎥 Webcam demo started - press 'q' to quit, 's' to save a snapshot")

    while True:
        ret, frame = cap.read()
        if not ret:
            print("❌ Failed to read frame")
            break

        detections = detector.detect(frame)

        # Send detections
        for det in detections:
            # Convert CropDetection dataclass into a simple dict for transmission
            payload = {
                'detection_id': det.detection_id,
                'latitude': det.latitude or 0.0,
                'longitude': det.longitude or 0.0,
                'confidence': det.confidence,
                'detection_area': det.area,
                'bounding_box': {
                    'x': det.bbox[0], 'y': det.bbox[1], 'width': det.bbox[2], 'height': det.bbox[3]
                },
                'timestamp': det.timestamp,
                'mission_id': config.get('mission_id', 'laptop_test')
            }

            sender.send_detection(payload)

        # Visualize
        out = detector.visualize_detections(frame, detections, show_info=True)
        cv2.imshow('Laptop MAVLink Detection Demo', out)

        key = cv2.waitKey(1) & 0xFF
        if key == ord('q'):
            break
        elif key == ord('s'):
            fname = f"webcam_capture_{int(time.time())}.jpg"
            cv2.imwrite(fname, out)
            print(f"📸 Saved: {fname}")

    cap.release()
    cv2.destroyAllWindows()


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='Laptop MAVLink Detection Sender (webcam demo)')
    parser.add_argument('--udp', default='udp:127.0.0.1:14550', help='UDP MAVLink endpoint (e.g., udp:127.0.0.1:14550)')
    parser.add_argument('--config', default='config.json', help='Path to config.json')
    args = parser.parse_args()

    if not PYMAVLINK_AVAILABLE:
        print("⚠️  pymavlink not available - using MockMAV (messages will be printed)")

    run_webcam_demo(udp_addr=args.udp if PYMAVLINK_AVAILABLE else None, config_path=args.config)
