"""
MAVLink Detection Sender - Sends crop detection data over MAVLink telemetry
Enables detection transmission over long-range radio when WiFi/LTE unavailable
"""

import time
import json
import struct
import threading
from typing import Dict, Optional, List
from datetime import datetime
from pymavlink import mavutil

class MAVLinkDetectionSender:
    """
    Sends crop detection data over MAVLink using custom STATUSTEXT messages.
    Provides fallback transmission when Socket.IO (WiFi/LTE) is unavailable.
    
    Uses STATUSTEXT with custom prefixes for structured data transmission:
    - DET: Detection event with coordinates and metadata
    - DSTAT: Detection statistics
    """
    
    # Message severity levels
    SEVERITY_INFO = mavutil.mavlink.MAV_SEVERITY_INFO
    SEVERITY_NOTICE = mavutil.mavlink.MAV_SEVERITY_NOTICE
    SEVERITY_WARNING = mavutil.mavlink.MAV_SEVERITY_WARNING
    
    def __init__(self, master, enabled: bool = True):
        """
        Initialize MAVLink detection sender
        
        Args:
            master: PyMAVLink connection object
            enabled: Enable/disable MAVLink transmission
        """
        self.master = master
        self.enabled = enabled
        self.detection_count = 0
        self.last_send_time = 0
        self.send_queue = []
        self.queue_lock = threading.Lock()
        
        # Rate limiting
        self.min_send_interval = 0.5  # Minimum 0.5s between messages
        self.max_queue_size = 50  # Maximum queued detections
        
        print(f"📡 MAVLink Detection Sender initialized (enabled: {enabled})")
        
    def send_detection(self, detection_data: Dict) -> bool:
        """
        Send crop detection over MAVLink telemetry
        
        Args:
            detection_data: Detection dictionary with GPS, confidence, etc.
            
        Returns:
            True if sent successfully, False otherwise
        """
        if not self.enabled or not self.master:
            return False
            
        try:
            # Extract key detection info
            det_id = detection_data.get('detection_id', 'unknown')
            lat = detection_data.get('latitude', 0.0)
            lon = detection_data.get('longitude', 0.0)
            confidence = detection_data.get('confidence', 0.0)
            area = detection_data.get('detection_area', 0.0)
            mission_id = detection_data.get('mission_id', 'none')
            
            # Format compact detection message (STATUSTEXT is limited to 50 chars)
            # Format: DET|ID|LAT|LON|CONF|AREA
            message = f"DET|{det_id[:15]}|{lat:.6f}|{lon:.6f}|{confidence:.2f}|{int(area)}"
            
            # Send via STATUSTEXT
            self._send_statustext(message, self.SEVERITY_INFO)
            
            self.detection_count += 1
            print(f"📡 MAVLink: Sent detection {det_id} via telemetry")
            
            return True
            
        except Exception as e:
            print(f"❌ MAVLink detection send failed: {e}")
            return False
    
    def send_detection_batch(self, detections: List[Dict]) -> int:
        """
        Send multiple detections efficiently
        
        Args:
            detections: List of detection dictionaries
            
        Returns:
            Number of successfully sent detections
        """
        if not self.enabled or not detections:
            return 0
            
        sent_count = 0
        for detection in detections:
            if self.send_detection(detection):
                sent_count += 1
                time.sleep(self.min_send_interval)  # Rate limiting
                
        return sent_count
    
    def send_detection_summary(self, summary: Dict) -> bool:
        """
        Send detection statistics/summary over MAVLink
        
        Args:
            summary: Dictionary with detection stats
            
        Returns:
            True if sent successfully
        """
        if not self.enabled or not self.master:
            return False
            
        try:
            total = summary.get('total_detections', 0)
            active = summary.get('detection_active', False)
            mission = summary.get('mission_id', 'none')
            
            # Format: DSTAT|TOTAL|ACTIVE|MISSION
            message = f"DSTAT|{total}|{1 if active else 0}|{mission[:20]}"
            
            self._send_statustext(message, self.SEVERITY_NOTICE)
            print(f"📡 MAVLink: Sent detection summary (total: {total})")
            
            return True
            
        except Exception as e:
            print(f"❌ MAVLink summary send failed: {e}")
            return False
    
    def send_detection_metadata(self, detection_data: Dict) -> bool:
        """
        Send detailed detection metadata in multiple messages
        Useful when bandwidth allows more detailed transmission
        
        Args:
            detection_data: Full detection dictionary
            
        Returns:
            True if sent successfully
        """
        if not self.enabled or not self.master:
            return False
            
        try:
            det_id = detection_data.get('detection_id', 'unknown')
            
            # Message 1: Basic info
            msg1 = f"DMET1|{det_id[:15]}|{detection_data.get('timestamp', '')[:20]}"
            self._send_statustext(msg1, self.SEVERITY_INFO)
            time.sleep(0.1)
            
            # Message 2: Bounding box
            bbox = detection_data.get('bounding_box', {})
            msg2 = f"DMET2|{bbox.get('x', 0)}|{bbox.get('y', 0)}|{bbox.get('width', 0)}|{bbox.get('height', 0)}"
            self._send_statustext(msg2, self.SEVERITY_INFO)
            time.sleep(0.1)
            
            # Message 3: Flight info
            mode = detection_data.get('drone_mode', 'UNK')
            speed = detection_data.get('ground_speed', 0.0)
            msg3 = f"DMET3|{mode}|{speed:.1f}|{detection_data.get('heading', 0):.1f}"
            self._send_statustext(msg3, self.SEVERITY_INFO)
            
            print(f"📡 MAVLink: Sent detection metadata for {det_id}")
            return True
            
        except Exception as e:
            print(f"❌ MAVLink metadata send failed: {e}")
            return False
    
    def _send_statustext(self, text: str, severity: int = SEVERITY_INFO):
        """
        Send STATUSTEXT message over MAVLink
        
        Args:
            text: Message text (max 50 characters)
            severity: MAVLink severity level
        """
        try:
            # Truncate to 50 chars (STATUSTEXT limit)
            text = text[:50]
            
            self.master.mav.statustext_send(
                severity,
                text.encode('utf-8')
            )
            
            self.last_send_time = time.time()
            
        except Exception as e:
            print(f"⚠️  STATUSTEXT send error: {e}")
    
    def enable(self):
        """Enable MAVLink detection transmission"""
        self.enabled = True
        print("📡 MAVLink detection sender enabled")
        
    def disable(self):
        """Disable MAVLink detection transmission"""
        self.enabled = False
        print("📡 MAVLink detection sender disabled")
        
    def get_stats(self) -> Dict:
        """Get transmission statistics"""
        return {
            'enabled': self.enabled,
            'detections_sent': self.detection_count,
            'queue_size': len(self.send_queue),
            'last_send': self.last_send_time
        }


class MAVLinkDetectionReceiver:
    """
    Receives crop detection data from MAVLink STATUSTEXT messages on GCS side.
    Parses structured detection messages sent from Pi over long-range radio.
    """
    
    def __init__(self, master, callback=None):
        """
        Initialize MAVLink detection receiver
        
        Args:
            master: PyMAVLink connection object
            callback: Function to call when detection received
        """
        self.master = master
        self.callback = callback
        self.running = False
        self.thread = None
        
        # Detection assembly (for multi-part messages)
        self.partial_detections = {}
        
        print("📡 MAVLink Detection Receiver initialized")
        
    def _listen_loop(self):
        """Background thread that listens for detection messages"""
        print("📡 MAVLink detection receiver started")
        
        while self.running:
            try:
                # Listen for STATUSTEXT messages
                msg = self.master.recv_match(type='STATUSTEXT', blocking=True, timeout=1.0)
                
                if msg:
                    text = msg.text.decode('utf-8') if isinstance(msg.text, bytes) else msg.text
                    
                    # Parse detection messages
                    if text.startswith('DET|'):
                        self._handle_detection_message(text)
                    elif text.startswith('DSTAT|'):
                        self._handle_detection_summary(text)
                    elif text.startswith('DMET'):
                        self._handle_detection_metadata(text)
                        
            except Exception as e:
                if self.running:
                    print(f"⚠️  MAVLink receive error: {e}")
                time.sleep(0.1)
                
        print("📡 MAVLink detection receiver stopped")
        
    def _handle_detection_message(self, text: str):
        """Parse and handle detection message"""
        try:
            # Format: DET|ID|LAT|LON|CONF|AREA
            parts = text.split('|')
            
            if len(parts) >= 6:
                detection = {
                    'detection_id': parts[1],
                    'latitude': float(parts[2]),
                    'longitude': float(parts[3]),
                    'confidence': float(parts[4]),
                    'detection_area': int(parts[5]),
                    'source': 'mavlink',
                    'timestamp': datetime.now().isoformat()
                }
                
                print(f"📡 Received detection via MAVLink: {detection['detection_id']}")
                
                if self.callback:
                    self.callback('detection', detection)
                    
        except Exception as e:
            print(f"⚠️  Failed to parse detection: {e}")
            
    def _handle_detection_summary(self, text: str):
        """Parse and handle detection summary"""
        try:
            # Format: DSTAT|TOTAL|ACTIVE|MISSION
            parts = text.split('|')
            
            if len(parts) >= 4:
                summary = {
                    'total_detections': int(parts[1]),
                    'detection_active': bool(int(parts[2])),
                    'mission_id': parts[3],
                    'source': 'mavlink',
                    'timestamp': datetime.now().isoformat()
                }
                
                print(f"📡 Received detection summary via MAVLink: {summary['total_detections']} total")
                
                if self.callback:
                    self.callback('summary', summary)
                    
        except Exception as e:
            print(f"⚠️  Failed to parse summary: {e}")
            
    def _handle_detection_metadata(self, text: str):
        """Parse and handle detection metadata"""
        try:
            # Handle multi-part metadata messages
            if text.startswith('DMET1|'):
                parts = text.split('|')
                det_id = parts[1]
                self.partial_detections[det_id] = {
                    'detection_id': det_id,
                    'timestamp': parts[2]
                }
            elif text.startswith('DMET2|'):
                parts = text.split('|')
                # bbox info
                pass
            elif text.startswith('DMET3|'):
                parts = text.split('|')
                # flight info
                pass
                
        except Exception as e:
            print(f"⚠️  Failed to parse metadata: {e}")
            
    def start(self):
        """Start listening for detection messages"""
        if not self.running:
            self.running = True
            self.thread = threading.Thread(target=self._listen_loop, daemon=True)
            self.thread.start()
            print("🚀 MAVLink detection receiver started")
            
    def stop(self):
        """Stop listening for detection messages"""
        self.running = False
        if self.thread:
            self.thread.join(timeout=2.0)
        print("🛑 MAVLink detection receiver stopped")


# Example usage on Pi side
if __name__ == "__main__":
    print("MAVLink Detection Sender Test")
    
    try:
        # Connect to Pixhawk
        master = mavutil.mavlink_connection('/dev/serial0', baud=57600)
        master.wait_heartbeat()
        print("✅ Connected to Pixhawk")
        
        # Create sender
        sender = MAVLinkDetectionSender(master, enabled=True)
        
        # Test detection
        test_detection = {
            'detection_id': 'mission_001_det_0001',
            'latitude': 40.7128,
            'longitude': -74.0060,
            'confidence': 0.95,
            'detection_area': 1200,
            'mission_id': 'mission_001'
        }
        
        sender.send_detection(test_detection)
        time.sleep(1)
        
        # Test summary
        sender.send_detection_summary({
            'total_detections': 5,
            'detection_active': True,
            'mission_id': 'mission_001'
        })
        
        print("✅ Test complete")
        
    except Exception as e:
        print(f"❌ Error: {e}")
