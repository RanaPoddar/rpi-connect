#!/usr/bin/env python3
"""
Raspberry Pi Controller - Runs on the Pi to receive commands via Socket.IO
Handles camera, system control, WebRTC streaming, and drone telemetry
"""

# Python 3.10+ compatibility fix for dronekit - MUST be imported first
import fix_collections

import socketio
import json
import subprocess
import time
import os
import sys
from datetime import datetime
import psutil
from picamera2 import Picamera2
from picamera2.encoders import JpegEncoder
from picamera2.outputs import FileOutput
import io
import base64
from threading import Thread, Lock
import numpy as np

# Import Pixhawk telemetry module
try:
    from modules.pixhawk_telemetry import PixhawkTelemetry
    PIXHAWK_MODULE_AVAILABLE = True
except ImportError as e:
    print(f" Pixhawk telemetry module not available: {e}")
    PIXHAWK_MODULE_AVAILABLE = False
    PixhawkTelemetry = None

# Import MAVLink commander module
try:
    from modules.mavlink_commander import MAVLinkCommander
    MAVLINK_COMMANDER_AVAILABLE = True
except ImportError as e:
    print(f" MAVLink commander module not available: {e}")
    MAVLINK_COMMANDER_AVAILABLE = False
    MAVLinkCommander = None

# Import Safety Manager module
try:
    from modules.safety_manager import SafetyManager
    SAFETY_MANAGER_AVAILABLE = True
except ImportError as e:
    print(f"⚠️  Safety Manager module not available: {e}")
    SAFETY_MANAGER_AVAILABLE = False
    SafetyManager = None

# Import Yellow Crop Detector module
try:
    from modules.yellow_crop_detector import YellowCropDetector, CropDetection
    from modules.geolocation import GeoLocationCalculator
    import cv2
    DETECTOR_AVAILABLE = True
except ImportError as e:
    print(f"⚠️  Yellow Crop Detector module not available: {e}")
    DETECTOR_AVAILABLE = False
    YellowCropDetector = None
    CropDetection = None
    GeoLocationCalculator = None

# Import MAVLink Command Receiver module
try:
    from modules.mavlink_command_receiver import MAVLinkCommandReceiver
    MAVLINK_COMMAND_RECEIVER_AVAILABLE = True
except ImportError as e:
    print(f"⚠️  MAVLink Command Receiver module not available: {e}")
    MAVLINK_COMMAND_RECEIVER_AVAILABLE = False
    MAVLinkCommandReceiver = None

# Import MAVLink Detection Sender module
try:
    from modules.mavlink_detection_sender import MAVLinkDetectionSender
    MAVLINK_DETECTION_SENDER_AVAILABLE = True
except ImportError as e:
    print(f"⚠️  MAVLink Detection Sender module not available: {e}")
    MAVLINK_DETECTION_SENDER_AVAILABLE = False
    MAVLinkDetectionSender = None

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
SERVER_URL = config.get('server_url', os.environ.get('SERVER_URL', 'http://localhost:3000'))
PI_ID = config.get('pi_id', os.environ.get('PI_ID', 'pi_001'))
CAMERA_ENABLED = config.get('camera', {}).get('enabled', True)
PIXHAWK_ENABLED = config.get('pixhawk', {}).get('enabled', True)
PIXHAWK_CONFIG = config.get('pixhawk', {})

# Ini socket  client
sio = socketio.Client(reconnection=True, reconnection_attempts=0)

# Camera state
camera_lock = Lock()
camera_active = False
streaming_active = False

class PiController:
    def __init__(self):
        self.camera = None
        self.is_running = True
        self.pixhawk = None
        self.commander = None
        self.safety_manager = None
        self.detector = None
        self.mavlink_receiver = None
        self.mavlink_detection_sender = None
        self.pixhawk_enabled = PIXHAWK_ENABLED and PIXHAWK_MODULE_AVAILABLE
        
        # Mission Management
        self.current_mission_id = None
        self.mission_active = False
        
        # Detection state
        self.detection_enabled = config.get('detection', {}).get('enabled', False)
        self.detection_active = False
        self.last_detection_time = 0
        self.detection_cooldown = config.get('detection', {}).get('detection_cooldown', 3.0)
        self.detection_count = 0
        
        # Periodic image capture
        self.periodic_capture_config = config.get('periodic_capture', {})
        self.periodic_capture_enabled = self.periodic_capture_config.get('enabled', False)
        self.periodic_capture_interval = self.periodic_capture_config.get('interval_seconds', 2.0)
        self.periodic_capture_active = False
        self.periodic_capture_count = 0
        self.periodic_capture_thread = None
        self.last_periodic_capture_time = 0
        
        # Initialize GeoLocation Calculator
        self.geo_calculator = None
        if DETECTOR_AVAILABLE and GeoLocationCalculator:
            try:
                self.geo_calculator = GeoLocationCalculator()
                print("📍 GeoLocation Calculator initialized")
            except Exception as e:
                print(f"⚠️  GeoLocation Calculator init failed: {e}")
        
        # Initialize Yellow Crop Detector if available
        if DETECTOR_AVAILABLE and self.detection_enabled:
            print("🌾 Initializing Yellow Crop Detector...")
            try:
                self.detector = YellowCropDetector(config=config)
                print("✅ Yellow Crop Detector initialized")
            except Exception as e:
                print(f"❌ Failed to initialize detector: {e}")
                self.detector = None
        
        # Initialize Pixhawk telemetry if enabled
        if self.pixhawk_enabled:
            print("🚁 Initializing Pixhawk telemetry...")
            self.pixhawk = PixhawkTelemetry(PIXHAWK_CONFIG)
            if self.pixhawk.connect():
                print("✅ Pixhawk telemetry initialized")
                # Start telemetry updates with callback
                self.pixhawk.start_telemetry_updates(callback=self._on_telemetry_update)
                
                # Initialize MAVLink Commander if available
                if MAVLINK_COMMANDER_AVAILABLE:
                    print("🎮 Initializing MAVLink Commander...")
                    self.commander = MAVLinkCommander(
                        vehicle=self.pixhawk.vehicle,
                        simulation_mode=PIXHAWK_CONFIG.get('simulation_mode', False)
                    )
                    print("✅ MAVLink Commander initialized")
                else:
                    print("⚠️  MAVLink Commander not available")
                
                # Initialize Safety Manager if available
                if SAFETY_MANAGER_AVAILABLE:
                    print("🛡️  Initializing Safety Manager...")
                    safety_config = config.get('safety', {})
                    self.safety_manager = SafetyManager(safety_config)
                    
                    # Set safety callbacks
                    self.safety_manager.set_warning_callback(self._on_safety_warning)
                    self.safety_manager.set_emergency_callback(self._on_safety_emergency)
                    
                    # Start safety monitoring
                    self.safety_manager.start_monitoring(
                        get_telemetry_func=lambda: self.pixhawk.get_telemetry() if self.pixhawk else {}
                    )
                    print("✅ Safety Manager initialized and monitoring started")
                else:
                    print("⚠️  Safety Manager not available")
                    
                # Initialize MAVLink Command Receiver if available and enabled
                mavlink_command_config = config.get('mavlink_command_receiver', {})
                if (mavlink_command_config.get('enabled', False) and 
                    MAVLINK_COMMAND_RECEIVER_AVAILABLE and self.pixhawk.vehicle):
                    print("📡 Initializing MAVLink Command Receiver...")
                    try:
                        # Share the existing master connection (no new connection!)
                        if hasattr(self.pixhawk.vehicle, '_master'):
                            mavlink_master = self.pixhawk.vehicle._master
                            
                            self.mavlink_receiver = MAVLinkCommandReceiver(
                                mavlink_master,
                                callback=self._handle_mavlink_command
                            )
                            self.mavlink_receiver.start()
                            print("✅ MAVLink Command Receiver initialized - Long range control enabled!")
                        else:
                            print("⚠️  Vehicle._master not available, skipping command receiver")
                            self.mavlink_receiver = None
                    except Exception as e:
                        print(f"⚠️  MAVLink Command Receiver failed to initialize: {e}")
                        self.mavlink_receiver = None
                else:
                    if not mavlink_command_config.get('enabled', False):
                        print("⚠️  MAVLink Command Receiver disabled in config (prevents serial port conflicts)")
                    else:
                        print("⚠️  MAVLink Command Receiver not available")
                
                # Initialize MAVLink Detection Sender if available
                if MAVLINK_DETECTION_SENDER_AVAILABLE and self.pixhawk.vehicle:
                    print("📡 Initializing MAVLink Detection Sender...")
                    try:
                        # Share the existing master connection (no new connection!)
                        if hasattr(self.pixhawk.vehicle, '_master'):
                            mavlink_master = self.pixhawk.vehicle._master
                            
                            self.mavlink_detection_sender = MAVLinkDetectionSender(
                                mavlink_master,
                                enabled=config.get('mavlink_detection', {}).get('enabled', True)
                            )
                            print("✅ MAVLink Detection Sender initialized - Hybrid transmission enabled!")
                        else:
                            print("⚠️  Vehicle._master not available, skipping detection sender")
                            self.mavlink_detection_sender = None
                    except Exception as e:
                        print(f"⚠️  MAVLink Detection Sender failed to initialize: {e}")
                        self.mavlink_detection_sender = None
                else:
                    print("⚠️  MAVLink Detection Sender not available")
            else:
                print("❌ Failed to initialize Pixhawk telemetry")
                self.pixhawk = None
        else:
            print("⚠️  Pixhawk telemetry disabled or module not available")
        
    def get_system_stats(self):
        """Get Pi system statistics"""
        try:
            cpu_temp = self.get_cpu_temperature()
            cpu_percent = psutil.cpu_percent(interval=1)
            memory = psutil.virtual_memory()
            disk = psutil.disk_usage('/')
            
            return {
                'cpu_temp': cpu_temp,
                'cpu_usage': cpu_percent,
                'memory_usage': memory.percent,
                'memory_available': memory.available // (1024 * 1024),  # MB
                'disk_usage': disk.percent,
                'disk_free': disk.free // (1024 * 1024 * 1024),  # GB
                'timestamp': datetime.now().isoformat()
            }
        except Exception as e:
            print(f"Error getting system stats: {e}")
            return {}
    
    def get_cpu_temperature(self):
        """Get CPU temperature"""
        try:
            temp = subprocess.check_output(['vcgencmd', 'measure_temp']).decode()
            return float(temp.replace('temp=', '').replace("'C\n", ''))
        except:
            return None
    
    def _on_telemetry_update(self, telemetry_data):
        """Callback for Pixhawk telemetry updates - sends to server via Socket.IO"""
        try:
            if sio.connected:
                # Add connection status to telemetry
                if not telemetry_data.get('connected', False):
                    telemetry_data['status_message'] = 'Pixhawk Not Connected'
                else:
                    telemetry_data['status_message'] = 'Connected'
                
                sio.emit('drone_telemetry', {
                    'pi_id': PI_ID,
                    'telemetry': telemetry_data,
                    'timestamp': datetime.now().isoformat()
                })
            
            # Update safety manager telemetry timestamp
            if self.safety_manager:
                self.safety_manager.update_telemetry_timestamp()
                
        except Exception as e:
            print(f"Error sending telemetry: {e}")
    
    def _on_safety_warning(self, warning: dict):
        """Callback for safety warnings"""
        print(f"⚠️  SAFETY WARNING: {warning.get('type', 'unknown')}")
        
        # Send warning to server
        if sio.connected:
            sio.emit('safety_warning', {
                'pi_id': PI_ID,
                'warning': warning,
                'timestamp': datetime.now().isoformat()
            })
    
    def _on_safety_emergency(self, emergency: dict):
        """Callback for safety emergencies - take automatic action"""
        action = emergency.get('action')
        reason = emergency.get('reason')
        
        print(f"🚨 SAFETY EMERGENCY: {reason}")
        print(f"   Automatic action: {action}")
        
        # Send emergency notification to server
        if sio.connected:
            sio.emit('safety_emergency', {
                'pi_id': PI_ID,
                'emergency': emergency,
                'timestamp': datetime.now().isoformat()
            })
        
        # Execute automatic safety action
        if self.commander:
            try:
                if action == 'RTL':
                    result = self.commander.return_to_launch()
                    print(f"   RTL executed: {result.get('success', False)}")
                elif action == 'LAND':
                    result = self.commander.emergency_stop()
                    print(f"   Emergency LAND executed: {result.get('success', False)}")
            except Exception as e:
                print(f"   ❌ Failed to execute safety action: {e}")
    
    def _handle_mavlink_command(self, command_type: str, params: dict):
        """
        Handle MAVLink commands received over long-range radio.
        This enables control of Pi functions even when outside WiFi range.
        
        Args:
            command_type: Type of command ('start_detection', 'stop_detection', etc.)
            params: Command parameters from MAVLink message
        """
        print(f"📡 MAVLink Command Handler: {command_type}")
        
        try:
            if command_type == 'start_detection':
                self.detection_active = True
                print("🌾 Detection started via MAVLink")
                
                # Send status update via Socket.IO if connected
                if sio.connected:
                    sio.emit('detection_status', {
                        'pi_id': PI_ID,
                        'status': 'active',
                        'message': 'Detection started via MAVLink',
                        'source': 'mavlink'
                    })
                    
            elif command_type == 'stop_detection':
                self.detection_active = False
                print("🌾 Detection stopped via MAVLink")
                
                if sio.connected:
                    sio.emit('detection_status', {
                        'pi_id': PI_ID,
                        'status': 'inactive',
                        'message': 'Detection stopped via MAVLink',
                        'source': 'mavlink'
                    })
                    
            elif command_type == 'request_stats':
                print("📊 Stats requested via MAVLink")
                if self.detector and sio.connected:
                    stats = self.detector.get_statistics()
                    stats['detection_count'] = self.detection_count
                    stats['detection_active'] = self.detection_active
                    sio.emit('detection_stats', {
                        'pi_id': PI_ID,
                        'stats': stats,
                        'source': 'mavlink'
                    })
                    
            elif command_type == 'start_capture':
                interval = params.get('param1', 2.0)
                print(f"📸 Periodic capture started via MAVLink (interval={interval}s)")
                self.periodic_capture_interval = interval
                self.start_periodic_capture()
                
            elif command_type == 'stop_capture':
                print("📸 Periodic capture stopped via MAVLink")
                self.stop_periodic_capture()
                
            else:
                print(f"⚠️  Unknown MAVLink command: {command_type}")
                
        except Exception as e:
            print(f"❌ MAVLink command handler error: {e}")
    
    def _process_detections(self, detections: list, frame: np.ndarray):
        """Process detections and send to server"""
        if not detections or not sio.connected:
            return
        
        # Skip if no active mission
        if not self.mission_active or not self.current_mission_id:
            print("⚠️  Detection skipped - no active mission")
            return
        
        try:
            # Get current GPS coordinates
            telemetry = self.pixhawk.get_telemetry() if self.pixhawk else {}
            timestamp = datetime.now().isoformat()
            
            for detection in detections:
                # Generate unique detection ID with mission prefix
                self.detection_count += 1
                unique_id = f"{self.current_mission_id}_det_{self.detection_count:04d}_{int(time.time())}"
                detection.detection_id = unique_id
                
                # Calculate accurate ground GPS coordinates using photogrammetry
                gps_data = telemetry.get('gps', {})
                if self.geo_calculator and gps_data.get('lat') and gps_data.get('lon'):
                    try:
                        # Get drone telemetry
                        drone_lat = gps_data.get('lat', 0.0)
                        drone_lon = gps_data.get('lon', 0.0)
                        altitude_agl = gps_data.get('relative_alt', 0.0)  # Altitude above ground
                        attitude = telemetry.get('attitude', {})
                        heading_deg = telemetry.get('heading', 0.0)
                        vehicle_pitch_deg = attitude.get('pitch', 0.0)  # Vehicle pitch
                        vehicle_roll_deg = attitude.get('roll', 0.0)    # Vehicle roll
                        
                        # Calculate effective camera pitch (vehicle pitch + camera mount offset)
                        # Camera is fixed bottom-facing at -90° relative to vehicle
                        camera_pitch_deg = vehicle_pitch_deg + self.geo_calculator.CAMERA_MOUNT_PITCH_DEG
                        camera_roll_deg = vehicle_roll_deg  # Roll passes through
                        
                        # Convert pixel coordinates to ground GPS using photogrammetry
                        ground_lat, ground_lon = self.geo_calculator.pixel_to_gps(
                            pixel_x=detection.centroid[0],
                            pixel_y=detection.centroid[1],
                            drone_lat=drone_lat,
                            drone_lon=drone_lon,
                            altitude_agl=altitude_agl,
                            heading_deg=heading_deg,
                            pitch_deg=camera_pitch_deg,  # Camera pitch in world frame
                            roll_deg=camera_roll_deg
                        )
                        
                        detection.latitude = ground_lat
                        detection.longitude = ground_lon
                        detection.altitude = 0.0  # Ground level
                        
                        print(f"📍 [{self.current_mission_id}] Photogrammetry: Pixel({detection.centroid[0]:.0f},{detection.centroid[1]:.0f}) "
                              f"→ GPS({ground_lat:.7f},{ground_lon:.7f}) | "
                              f"Alt:{altitude_agl:.1f}m, Hdg:{heading_deg:.1f}°, CamPitch:{camera_pitch_deg:.1f}°")
                        
                    except Exception as e:
                        print(f"⚠️  Photogrammetry calculation failed: {e}, using drone GPS")
                        detection.latitude = gps_data.get('lat', 0.0)
                        detection.longitude = gps_data.get('lon', 0.0)
                        detection.altitude = gps_data.get('relative_alt', 0.0)
                else:
                    # Fallback to drone GPS if photogrammetry unavailable
                    print(f"⚠️  No GPS data available - GPS: lat={gps_data.get('lat', 0)}, lon={gps_data.get('lon', 0)}")
                    detection.latitude = gps_data.get('lat', 0.0)
                    detection.longitude = gps_data.get('lon', 0.0)
                    detection.altitude = gps_data.get('relative_alt', 0.0)
                
                # Prepare detection data (metadata only, no image)
                detection_data = detection.to_dict()
                detection_data.update({
                    'detection_id': unique_id,
                    'mission_id': self.current_mission_id,
                    'pi_id': PI_ID,
                    'timestamp': timestamp,
                    'drone_mode': telemetry.get('mode', 'UNKNOWN'),
                    'simulation': telemetry.get('simulation', False),
                    'heading': telemetry.get('heading'),
                    'ground_speed': telemetry.get('groundspeed')
                })
                
                # Send to server via Socket.IO (WiFi/LTE) - metadata only
                socketio_sent = False
                if sio.connected:
                    try:
                        sio.emit('crop_detection', detection_data)
                        socketio_sent = True
                        print(f"   📤 Socket.IO: Detection {self.detection_count:04d} sent: {unique_id}")
                    except Exception as e:
                        print(f"   ⚠️  Socket.IO send failed: {e}")
                
                # Send via MAVLink (Long-range radio fallback/backup)
                mavlink_sent = False
                if self.mavlink_detection_sender and self.mavlink_detection_sender.enabled:
                    try:
                        mavlink_sent = self.mavlink_detection_sender.send_detection(detection_data)
                        if mavlink_sent:
                            print(f"   📡 MAVLink: Detection {self.detection_count:04d} sent: {unique_id}")
                    except Exception as e:
                        print(f"   ⚠️  MAVLink send failed: {e}")
                
                # Log transmission status
                if socketio_sent and mavlink_sent:
                    print(f"   ✅ [{self.current_mission_id}] Detection {self.detection_count:04d} sent via BOTH channels")
                elif socketio_sent:
                    print(f"   ⚠️  [{self.current_mission_id}] Detection sent via Socket.IO only (MAVLink unavailable)")
                elif mavlink_sent:
                    print(f"   ⚠️  [{self.current_mission_id}] Detection sent via MAVLink only (Socket.IO unavailable)")
                else:
                    print(f"   ❌ [{self.current_mission_id}] Detection {self.detection_count:04d} FAILED - no transmission channel available!")
                
                # Optional: Save locally
                if config.get('detection', {}).get('save_detection_images', False):
                    self._save_detection_locally(detection, frame)
                
        except Exception as e:
            print(f"❌ Error processing detections: {e}")
    
    def _save_detection_locally(self, detection: CropDetection, frame: np.ndarray):
        """Save detection image locally"""
        try:
            import cv2
            import os
            
            # Create detections directory
            det_dir = os.path.join(os.path.dirname(__file__), 'detected_crops')
            os.makedirs(det_dir, exist_ok=True)
            
            # Save detection image with bounding box
            viz_frame = self.detector.visualize_detections(frame.copy(), [detection])
            filename = f"{detection.detection_id}.jpg"
            filepath = os.path.join(det_dir, filename)
            cv2.imwrite(filepath, viz_frame)
            
            # Save metadata
            import json
            metadata_file = os.path.join(det_dir, f"{detection.detection_id}.json")
            with open(metadata_file, 'w') as f:
                json.dump(detection.to_dict(), f, indent=2)
                
        except Exception as e:
            print(f"Failed to save detection locally: {e}")
    
    def get_pixhawk_status(self):
        """Get Pixhawk connection status and current telemetry"""
        if not self.pixhawk:
            return {
                'enabled': False,
                'connected': False,
                'message': 'Pixhawk module not initialized'
            }
        
        status = self.pixhawk.get_connection_status()
        telemetry = self.pixhawk.get_telemetry()
        
        return {
            'enabled': True,
            'connected': status['connected'],
            'simulation_mode': status['simulation_mode'],
            'telemetry': telemetry
        }
    
    def execute_command(self, command, args=None):
        """Execute system commands"""
        try:
            if command == 'reboot':
                subprocess.Popen(['sudo', 'reboot'])
                return {'success': True, 'message': 'Rebooting...'}
            
            elif command == 'shutdown':
                subprocess.Popen(['sudo', 'shutdown', 'now'])
                return {'success': True, 'message': 'Shutting down...'}
            
            elif command == 'restart_service':
                service = args.get('service', '')
                result = subprocess.run(['sudo', 'systemctl', 'restart', service], 
                                      capture_output=True, text=True)
                return {
                    'success': result.returncode == 0,
                    'message': result.stdout or result.stderr
                }
            
            elif command == 'camera_test':
                return self.test_camera()
            
            # MAVLink commander commands
            elif command == 'arm':
                return self.arm_vehicle(args)
            
            elif command == 'disarm':
                return self.disarm_vehicle(args)
            
            elif command == 'set_mode':
                return self.set_flight_mode(args)
            
            elif command == 'takeoff':
                return self.takeoff_vehicle(args)
            
            elif command == 'land':
                return self.land_vehicle(args)
            
            elif command == 'goto_location':
                return self.goto_location(args)
            
            elif command == 'upload_mission':
                return self.upload_mission(args)
            
            elif command == 'start_mission':
                return self.start_mission(args)
            
            elif command == 'clear_mission':
                return self.clear_mission(args)
            
            elif command == 'get_mission_progress':
                return self.get_mission_progress(args)
            
            elif command == 'rtl':
                return self.return_to_launch(args)
            
            elif command == 'emergency_stop':
                return self.emergency_stop(args)
            
            elif command == 'capture_image':
                return self.capture_image()
            
            elif command == 'pixhawk_status':
                return {'success': True, 'data': self.get_pixhawk_status()}
            
            elif command == 'pixhawk_reconnect':
                return self.reconnect_pixhawk()
            
            elif command == 'mark_waypoint':
                return self.mark_waypoint(args)
            
            else:
                return {'success': False, 'message': f'Unknown command: {command}'}
                
        except Exception as e:
            return {'success': False, 'message': str(e)}
    
    def test_camera(self):
        """Test if camera is accessible"""
        global camera_active, CAMERA_ENABLED
        
        if not CAMERA_ENABLED:
            return {'success': False, 'message': 'Camera disabled'}
        
        try:
            with camera_lock:
                if camera_active:
                    return {'success': True, 'message': 'Camera already active'}
                
                # Check if cameras are available
                cameras = Picamera2.global_camera_info()
                if not cameras:
                    return {'success': False, 'message': 'No cameras detected. Check camera connection and enable in raspi-config.'}
                
                print(f"Available cameras: {cameras}")
                
                test_camera = Picamera2()
                config = test_camera.create_preview_configuration(
                    main={"size": (640, 480)},
                    controls={
                        "AwbEnable": True,
                        "AwbMode": 4,  # Daylight mode (natural colors)
                        "Saturation": 1.0,
                        "Contrast": 1.0
                    }
                )
                test_camera.configure(config)
                test_camera.start()
                time.sleep(2)  # Camera warm-up
                test_camera.stop()
                test_camera.close()
                
                return {'success': True, 'message': f'Camera test successful. Found {len(cameras)} camera(s).'}
        except Exception as e:
            return {'success': False, 'message': f'Camera test failed: {str(e)}'}
    
    def capture_image(self):
        """Capture a single image from camera"""
        global CAMERA_ENABLED, camera_active, streaming_active
        
        if not CAMERA_ENABLED:
            return {'success': False, 'message': 'Camera disabled'}
        
        try:
            import cv2
            
            # Create captured_images directory if it doesn't exist
            capture_dir = os.path.join(os.path.dirname(__file__), 'captured_images')
            os.makedirs(capture_dir, exist_ok=True)
            
            # Generate filename with timestamp
            timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
            filename = f'capture_{PI_ID}_{timestamp}.jpg'
            filepath = os.path.join(capture_dir, filename)
            
            # If camera is already streaming, capture from active stream
            if camera_active and self.camera:
                with camera_lock:
                    # Capture from the active camera
                    image_array = self.camera.capture_array()
                    
                    # Convert RGB to BGR for JPEG encoding
                    image_bgr = cv2.cvtColor(image_array, cv2.COLOR_RGB2BGR)
                    
                    # Save to disk
                    cv2.imwrite(filepath, image_bgr, [cv2.IMWRITE_JPEG_QUALITY, 85])
                    
                    # Encode for transmission
                    _, buffer = cv2.imencode('.jpg', image_bgr, [cv2.IMWRITE_JPEG_QUALITY, 85])
                    image_data = base64.b64encode(buffer).decode('utf-8')
                    
                    return {
                        'success': True,
                        'message': f'Image captured and saved to {filename}',
                        'image': image_data,
                        'filepath': filepath,
                        'filename': filename,
                        'timestamp': datetime.now().isoformat()
                    }
            
            # If camera is not active, create new instance for single capture
            with camera_lock:
                # Check if cameras are available
                cameras = Picamera2.global_camera_info()
                if not cameras:
                    return {'success': False, 'message': 'No cameras detected. Check camera connection and enable in raspi-config.'}
                
                camera = Picamera2()
                config = camera.create_still_configuration(
                    main={"size": (1920, 1080)},
                    controls={
                        "AwbEnable": True,
                        "AwbMode": 4,  # Daylight mode (natural colors)
                        "Saturation": 1.0,
                        "Contrast": 1.0,
                        "Sharpness": 1.0
                    }
                )
                camera.configure(config)
                camera.start()
                time.sleep(2)  # Camera warm-up
                
                # Capture to numpy array
                image_array = camera.capture_array()
                camera.stop()
                camera.close()
                
                # Convert RGB to BGR for JPEG encoding
                image_bgr = cv2.cvtColor(image_array, cv2.COLOR_RGB2BGR)
                
                # Save to disk
                cv2.imwrite(filepath, image_bgr, [cv2.IMWRITE_JPEG_QUALITY, 85])
                
                # Encode for transmission
                _, buffer = cv2.imencode('.jpg', image_bgr, [cv2.IMWRITE_JPEG_QUALITY, 85])
                image_data = base64.b64encode(buffer).decode('utf-8')
                
                return {
                    'success': True,
                    'message': f'Image captured and saved to {filename}',
                    'image': image_data,
                    'filepath': filepath,
                    'filename': filename,
                    'timestamp': datetime.now().isoformat()
                }
        except Exception as e:
            return {'success': False, 'message': f'Capture failed: {str(e)}'}
    
    def start_camera_stream(self, settings):
        """Start camera streaming"""
        global camera_active, streaming_active
        
        with camera_lock:
            if camera_active:
                return {'success': False, 'message': 'Camera already active'}
            
            try:
                # Check if cameras are available
                cameras = Picamera2.global_camera_info()
                if not cameras:
                    return {'success': False, 'message': 'No cameras detected. Check camera connection and enable in raspi-config.'}
                
                self.camera = Picamera2()
                resolution = tuple(settings.get('resolution', [640, 480]))
                # Use BGR888 format directly to match OpenCV
                config = self.camera.create_video_configuration(
                    main={"size": resolution, "format": "BGR888"},
                    controls={
                        "FrameRate": settings.get('framerate', 30),
                        "AeEnable": True,  # Auto exposure
                        "AwbEnable": True,  # Enable auto white balance
                        "AwbMode": 0,  # Auto mode (adapts to lighting)
                        "Saturation": 1.0,
                        "Contrast": 1.0,
                        "Brightness": 0.0,
                        "Sharpness": 1.0
                    }
                )
                self.camera.configure(config)
                self.camera.start()
                
                # Wait for camera auto exposure and white balance to settle
                print(f"Camera starting - waiting 2 seconds for auto-adjust...")
                time.sleep(2)
                print(f"Camera ready with BGR888 format for streaming")
                
                camera_active = True
                streaming_active = True
                
                # Start streaming thread
                Thread(target=self._stream_frames, daemon=True).start()
                
                # Start periodic capture if enabled and mission active
                if self.periodic_capture_enabled and self.mission_active:
                    self.start_periodic_capture()
                
                return {'success': True, 'message': 'Streaming started'}
            except Exception as e:
                camera_active = False
                return {'success': False, 'message': str(e)}
    
    def stop_camera_stream(self):
        """Stop camera streaming"""
        global camera_active, streaming_active
        
        streaming_active = False
        time.sleep(0.5)
        
        with camera_lock:
            if self.camera:
                try:
                    self.camera.stop()
                    self.camera.close()
                except:
                    pass
                self.camera = None
            camera_active = False
        
        # Stop periodic capture if active
        if self.periodic_capture_active:
            self.stop_periodic_capture()
        
        return {'success': True, 'message': 'Streaming stopped'}
    
    def start_periodic_capture(self):
        """Start periodic image capture thread"""
        if self.periodic_capture_active:
            return {'success': False, 'message': 'Periodic capture already active'}
        
        if not self.mission_active:
            return {'success': False, 'message': 'No active mission'}
        
        self.periodic_capture_active = True
        self.periodic_capture_thread = Thread(target=self._periodic_capture_loop, daemon=True)
        self.periodic_capture_thread.start()
        print(f"📸 Periodic capture started: {self.periodic_capture_interval}s interval")
        return {'success': True, 'message': 'Periodic capture started'}
    
    def stop_periodic_capture(self):
        """Stop periodic image capture"""
        self.periodic_capture_active = False
        if self.periodic_capture_thread:
            self.periodic_capture_thread.join(timeout=2.0)
        print("📸 Periodic capture stopped")
        return {'success': True, 'message': 'Periodic capture stopped'}
    
    def _periodic_capture_loop(self):
        """Thread loop for periodic image capture"""
        import cv2
        
        while self.periodic_capture_active and self.mission_active and self.is_running:
            try:
                current_time = time.time()
                
                # Check if enough time has passed
                if current_time - self.last_periodic_capture_time < self.periodic_capture_interval:
                    time.sleep(0.1)
                    continue
                
                # Capture frame from camera
                if self.camera:
                    frame = self.camera.capture_array()
                    
                    # Convert RGB to BGR for OpenCV
                    if len(frame.shape) == 3 and frame.shape[2] == 3:
                        frame_bgr = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
                    else:
                        frame_bgr = frame
                    
                    # Get telemetry
                    telemetry = self.pixhawk.get_telemetry() if self.pixhawk else {}
                    timestamp = datetime.now().isoformat()
                    
                    # Generate unique image ID
                    self.periodic_capture_count += 1
                    image_id = f"{self.current_mission_id}_img_{self.periodic_capture_count:04d}_{int(time.time())}"
                    
                    # Encode image
                    quality = self.periodic_capture_config.get('quality', 85)
                    _, buffer = cv2.imencode('.jpg', frame_bgr, [cv2.IMWRITE_JPEG_QUALITY, quality])
                    image_data = base64.b64encode(buffer).decode('utf-8')
                    
                    # Prepare image data packet
                    image_packet = {
                        'image_id': image_id,
                        'mission_id': self.current_mission_id,
                        'pi_id': PI_ID,
                        'timestamp': timestamp,
                        'image': image_data,
                        'image_type': 'periodic',
                        'latitude': telemetry.get('latitude', 0.0),
                        'longitude': telemetry.get('longitude', 0.0),
                        'altitude': telemetry.get('altitude', 0.0),
                        'heading': telemetry.get('heading', 0.0),
                        'mode': telemetry.get('mode', 'UNKNOWN')
                    }
                    
                    # Send to server via Socket.IO (WiFi/LTE primary)
                    socketio_sent = False
                    if self.periodic_capture_config.get('send_to_server', True) and sio.connected:
                        try:
                            sio.emit('periodic_image', image_packet)
                            socketio_sent = True
                            print(f"   📷 Periodic image {self.periodic_capture_count:04d} sent via Socket.IO")
                        except Exception as e:
                            print(f"   ⚠️  Socket.IO periodic image send failed: {e}")
                    
                    # Fallback to MAVLink if Socket.IO failed or unavailable
                    mavlink_sent = False
                    if not socketio_sent and self.mavlink_detection_sender and self.mavlink_detection_sender.enabled:
                        try:
                            # Send compressed metadata over MAVLink (full image too large)
                            # Images are stored locally for post-landing sync
                            metadata = {
                                'image_id': image_id,
                                'mission_id': self.current_mission_id,
                                'timestamp': timestamp,
                                'latitude': telemetry.get('latitude', 0.0),
                                'longitude': telemetry.get('longitude', 0.0),
                                'altitude': telemetry.get('altitude', 0.0),
                                'image_type': 'periodic',
                                'stored_locally': True
                            }
                            mavlink_sent = self.mavlink_detection_sender.send_image_metadata(metadata)
                            if mavlink_sent:
                                print(f"   📡 Periodic image {self.periodic_capture_count:04d} metadata sent via MAVLink")
                        except Exception as e:
                            print(f"   ⚠️  MAVLink image metadata send failed: {e}")
                    
                    # Log transmission status
                    if socketio_sent and mavlink_sent:
                        print(f"   ✅ Periodic image sent via BOTH channels")
                    elif socketio_sent:
                        print(f"   ✅ Periodic image sent via Socket.IO only")
                    elif mavlink_sent:
                        print(f"   ⚠️  Periodic image metadata sent via MAVLink only (WiFi unavailable)")
                    else:
                        print(f"   ❌ Periodic image FAILED - no transmission channel available!")
                    
                    self.last_periodic_capture_time = current_time
                    
            except Exception as e:
                print(f"❌ Error in periodic capture: {e}")
                time.sleep(1.0)
        
        print("📸 Periodic capture loop ended")
    
    def _stream_frames(self):
        """Stream camera frames via Socket.IO with optional detection"""
        global streaming_active
        import cv2
        
        try:
            while streaming_active:
                # Capture frame (BGR888 config, but may actually be RGB)
                frame = self.camera.capture_array()
                
                if not streaming_active:
                    break
                
                # Swap R and B channels (fix color inversion)
                frame_bgr = frame[:, :, ::-1].copy()
                
                # Perform detection if enabled
                if self.detection_active and self.detector:
                    current_time = time.time()
                    
                    # Check cooldown
                    if current_time - self.last_detection_time >= self.detection_cooldown:
                        try:
                            detections = self.detector.detect(frame_bgr)
                            
                            if detections:
                                print(f"🌾 Detected {len(detections)} yellow crop(s)")
                                self.last_detection_time = current_time
                                self.detection_count += len(detections)
                                
                                # Process detections and send to server
                                self._process_detections(detections, frame_bgr)
                            
                            # Visualize detections on frame
                            frame_bgr = self.detector.visualize_detections(frame_bgr, detections)
                        except Exception as e:
                            print(f"Detection error: {e}")
                
                # Encode BGR frame as JPEG (OpenCV expects BGR)
                _, buffer = cv2.imencode('.jpg', frame_bgr, [cv2.IMWRITE_JPEG_QUALITY, 75])
                frame_data = base64.b64encode(buffer).decode('utf-8')
                
                # Send frame to server (only if connected)
                if sio.connected:
                    sio.emit('camera_frame', {
                        'pi_id': PI_ID,
                        'frame': frame_data,
                        'timestamp': time.time()
                    })
                
                time.sleep(0.1)  # ~10 fps (reduced to prevent disconnection)
                
        except Exception as e:
            print(f"Streaming error: {e}")
        finally:
            streaming_active = False
    
    def reconnect_pixhawk(self):
        """Reconnect to Pixhawk flight controller"""
        if not self.pixhawk_enabled:
            return {'success': False, 'message': 'Pixhawk module not enabled'}
        
        try:
            # Disconnect if already connected
            if self.pixhawk:
                self.pixhawk.disconnect()
            
            # Reinitialize
            self.pixhawk = PixhawkTelemetry(PIXHAWK_CONFIG)
            
            if self.pixhawk.connect():
                self.pixhawk.start_telemetry_updates(callback=self._on_telemetry_update)
                return {'success': True, 'message': 'Reconnected to Pixhawk'}
            else:
                return {'success': False, 'message': 'Failed to reconnect to Pixhawk'}
        
        except Exception as e:
            return {'success': False, 'message': f'Reconnection error: {str(e)}'}
    
    def mark_waypoint(self, args=None):
        """Mark current GPS location as a waypoint"""
        try:
            if not self.pixhawk:
                return {'success': False, 'message': 'Pixhawk not available'}
            
            # Get current telemetry
            telemetry = self.pixhawk.get_telemetry()
            
            if not telemetry.get('connected', False):
                return {'success': False, 'message': 'Pixhawk not connected'}
            
            gps_data = telemetry.get('gps', {})
            lat = gps_data.get('lat', 0.0)
            lon = gps_data.get('lon', 0.0)
            alt = gps_data.get('alt', 0.0)
            
            # Check if GPS has valid fix
            if lat == 0.0 and lon == 0.0:
                return {'success': False, 'message': 'No GPS fix available'}
            
            # Get additional data
            waypoint_name = args.get('name', '') if args else ''
            
            # Create waypoint data
            waypoint_data = {
                'waypoint_id': f'WP_{int(time.time() * 1000)}_{PI_ID}',
                'pi_id': PI_ID,
                'name': waypoint_name,
                'latitude': lat,
                'longitude': lon,
                'altitude': alt,
                'relative_altitude': gps_data.get('relative_alt', 0.0),
                'heading': telemetry.get('heading', 0),
                'flight_mode': telemetry.get('flight_mode', 'UNKNOWN'),
                'timestamp': time.time(),
                'datetime': datetime.now().isoformat(),
                'gps_satellites': gps_data.get('satellites', 0),
                'gps_fix_type': gps_data.get('fix_type', 0)
            }
            
            # Send waypoint to server via Socket.IO
            if sio.connected:
                sio.emit('waypoint_marked', waypoint_data)
                print(f"✓ Waypoint marked: {waypoint_data['waypoint_id']} at ({lat:.6f}, {lon:.6f})")
            
            return {
                'success': True,
                'message': f'Waypoint marked at ({lat:.6f}, {lon:.6f})',
                'data': waypoint_data
            }
            
        except Exception as e:
            return {'success': False, 'message': f'Error marking waypoint: {str(e)}'}
    
    # ============ MAVLINK COMMANDER METHODS ============
    
    def arm_vehicle(self, args=None):
        """ARM the drone"""
        if not self.commander:
            return {'success': False, 'message': 'MAVLink Commander not available'}
        
        try:
            force = args.get('force', False) if args else False
            result = self.commander.arm(force=force)
            return result
        except Exception as e:
            return {'success': False, 'message': f'ARM failed: {str(e)}'}
    
    def disarm_vehicle(self, args=None):
        """DISARM the drone"""
        if not self.commander:
            return {'success': False, 'message': 'MAVLink Commander not available'}
        
        try:
            force = args.get('force', False) if args else False
            result = self.commander.disarm(force=force)
            return result
        except Exception as e:
            return {'success': False, 'message': f'DISARM failed: {str(e)}'}
    
    def set_flight_mode(self, args=None):
        """Change flight mode"""
        if not self.commander:
            return {'success': False, 'message': 'MAVLink Commander not available'}
        
        if not args or 'mode' not in args:
            return {'success': False, 'message': 'Mode parameter required'}
        
        try:
            mode = args['mode']
            result = self.commander.set_mode(mode)
            return result
        except Exception as e:
            return {'success': False, 'message': f'Mode change failed: {str(e)}'}
    
    def takeoff_vehicle(self, args=None):
        """Takeoff to specified altitude"""
        if not self.commander:
            return {'success': False, 'message': 'MAVLink Commander not available'}
        
        if not args or 'altitude' not in args:
            return {'success': False, 'message': 'Altitude parameter required'}
        
        try:
            altitude = float(args['altitude'])
            timeout = args.get('timeout', 60)
            result = self.commander.takeoff(altitude, timeout=timeout)
            return result
        except Exception as e:
            return {'success': False, 'message': f'Takeoff failed: {str(e)}'}
    
    def land_vehicle(self, args=None):
        """Land the drone"""
        if not self.commander:
            return {'success': False, 'message': 'MAVLink Commander not available'}
        
        try:
            timeout = args.get('timeout', 60) if args else 60
            result = self.commander.land(timeout=timeout)
            return result
        except Exception as e:
            return {'success': False, 'message': f'Landing failed: {str(e)}'}
    
    def goto_location(self, args=None):
        """Navigate to GPS coordinates"""
        if not self.commander:
            return {'success': False, 'message': 'MAVLink Commander not available'}
        
        if not args or 'lat' not in args or 'lon' not in args:
            return {'success': False, 'message': 'Latitude and longitude required'}
        
        try:
            lat = float(args['lat'])
            lon = float(args['lon'])
            altitude = float(args.get('altitude')) if 'altitude' in args else None
            groundspeed = float(args.get('groundspeed')) if 'groundspeed' in args else None
            timeout = args.get('timeout', 120)
            
            result = self.commander.goto_location(lat, lon, altitude, groundspeed, timeout)
            return result
        except Exception as e:
            return {'success': False, 'message': f'Navigation failed: {str(e)}'}
    
    def upload_mission(self, args=None):
        """Upload multi-waypoint mission"""
        if not self.commander:
            return {'success': False, 'message': 'MAVLink Commander not available'}
        
        if not args or 'waypoints' not in args:
            return {'success': False, 'message': 'Waypoints list required'}
        
        try:
            waypoints = args['waypoints']
            takeoff_alt = float(args.get('takeoff_alt')) if 'takeoff_alt' in args else None
            
            result = self.commander.upload_mission(waypoints, takeoff_alt=takeoff_alt)
            return result
        except Exception as e:
            return {'success': False, 'message': f'Mission upload failed: {str(e)}'}
    
    def start_mission(self, args=None):
        """Start the uploaded mission"""
        if not self.commander:
            return {'success': False, 'message': 'MAVLink Commander not available'}
        
        try:
            result = self.commander.start_mission()
            
            # Start mission monitoring if successful
            if result.get('success'):
                print("🔍 Starting mission monitoring...")
                self._start_mission_monitoring()
            
            return result
        except Exception as e:
            error_msg = f'Mission start failed: {str(e)}'
            print(f"❌ {error_msg}")
            
            # Notify server of error
            if sio.connected:
                sio.emit('mission_error', {
                    'pi_id': PI_ID,
                    'error': error_msg,
                    'timestamp': datetime.now().isoformat()
                })
            
            return {'success': False, 'message': error_msg}
    
    def _start_mission_monitoring(self):
        """Monitor mission progress and handle completion/errors"""
        import threading
        
        def monitor():
            try:
                while self.mission_active and self.pixhawk and self.pixhawk.vehicle:
                    # Check if mission is completed
                    if self.commander:
                        progress = self.commander.get_mission_progress()
                        
                        # Check for mission completion
                        if not self.commander.vehicle.armed and progress.get('mode') == 'AUTO':
                            print("✅ Mission completed - drone has landed")
                            if sio.connected:
                                sio.emit('mission_completed', {
                                    'pi_id': PI_ID,
                                    'timestamp': datetime.now().isoformat()
                                })
                            break
                        
                        # Check for errors/failsafe
                        if self.pixhawk.vehicle:
                            mode = str(self.pixhawk.vehicle.mode.name)
                            if mode in ['RTL', 'LAND'] and progress.get('current_waypoint', 0) < progress.get('total_waypoints', 0):
                                error_msg = f'Mission aborted - unexpected {mode} mode'
                                print(f"⚠️  {error_msg}")
                                if sio.connected:
                                    sio.emit('mission_error', {
                                        'pi_id': PI_ID,
                                        'error': error_msg,
                                        'timestamp': datetime.now().isoformat()
                                    })
                                break
                    
                    time.sleep(2)  # Check every 2 seconds
                    
            except Exception as e:
                error_msg = f'Mission monitoring error: {str(e)}'
                print(f"❌ {error_msg}")
                if sio.connected:
                    sio.emit('mission_error', {
                        'pi_id': PI_ID,
                        'error': error_msg,
                        'timestamp': datetime.now().isoformat()
                    })
        
        monitor_thread = threading.Thread(target=monitor, daemon=True)
        monitor_thread.start()
    
    def clear_mission(self, args=None):
        """Clear the current mission"""
        if not self.commander:
            return {'success': False, 'message': 'MAVLink Commander not available'}
        
        try:
            result = self.commander.clear_mission()
            return result
        except Exception as e:
            return {'success': False, 'message': f'Mission clear failed: {str(e)}'}
    
    def get_mission_progress(self, args=None):
        """Get current mission progress"""
        if not self.commander:
            return {'success': False, 'message': 'MAVLink Commander not available'}
        
        try:
            result = self.commander.get_mission_progress()
            return {'success': True, 'data': result}
        except Exception as e:
            return {'success': False, 'message': f'Failed to get mission progress: {str(e)}'}
    
    def return_to_launch(self, args=None):
        """Return to launch point (RTL)"""
        if not self.commander:
            return {'success': False, 'message': 'MAVLink Commander not available'}
        
        try:
            result = self.commander.return_to_launch()
            return result
        except Exception as e:
            return {'success': False, 'message': f'RTL failed: {str(e)}'}
    
    def emergency_stop(self, args=None):
        """Emergency stop - immediate LAND"""
        if not self.commander:
            return {'success': False, 'message': 'MAVLink Commander not available'}
        
        try:
            result = self.commander.emergency_stop()
            return result
        except Exception as e:
            return {'success': False, 'message': f'Emergency stop failed: {str(e)}'}
    
    # ============ SAFETY MANAGER METHODS ============
    
    def set_geofence(self, args=None):
        """Set geofence center to current location or specified coordinates"""
        if not self.safety_manager:
            return {'success': False, 'message': 'Safety Manager not available'}
        
        try:
            if args and 'lat' in args and 'lon' in args:
                lat = float(args['lat'])
                lon = float(args['lon'])
            else:
                # Use current location
                if not self.pixhawk:
                    return {'success': False, 'message': 'Cannot get current location - Pixhawk not available'}
                
                telemetry = self.pixhawk.get_telemetry()
                gps = telemetry.get('gps', {})
                lat = gps.get('lat', 0)
                lon = gps.get('lon', 0)
                
                if lat == 0 and lon == 0:
                    return {'success': False, 'message': 'No GPS fix available'}
            
            self.safety_manager.set_geofence_center(lat, lon)
            
            return {
                'success': True,
                'message': f'Geofence center set to ({lat:.6f}, {lon:.6f})',
                'center': {'lat': lat, 'lon': lon}
            }
        except Exception as e:
            return {'success': False, 'message': f'Set geofence failed: {str(e)}'}
    
    def check_pre_flight(self, args=None):
        """Run pre-flight safety checklist"""
        if not self.safety_manager:
            return {'success': False, 'message': 'Safety Manager not available'}
        
        if not self.pixhawk:
            return {'success': False, 'message': 'Pixhawk not available'}
        
        try:
            telemetry = self.pixhawk.get_telemetry()
            result = self.safety_manager.pre_flight_checklist(telemetry)
            
            return {
                'success': True,
                'data': result
            }
        except Exception as e:
            return {'success': False, 'message': f'Pre-flight check failed: {str(e)}'}
    
    def get_safety_status(self, args=None):
        """Get current safety status"""
        if not self.safety_manager:
            return {'success': False, 'message': 'Safety Manager not available'}
        
        try:
            status = self.safety_manager.get_safety_status()
            return {
                'success': True,
                'data': status
            }
        except Exception as e:
            return {'success': False, 'message': f'Get safety status failed: {str(e)}'}
    
    def shutdown(self):
        """Clean shutdown of all systems"""
        print("\n🛑 Shutting down Pi Controller...")
        
        # Stop safety monitoring
        if self.safety_manager:
            print("   Stopping safety monitoring...")
            self.safety_manager.stop_monitoring()
        
        # Stop camera
        if camera_active:
            self.stop_camera_stream()
        
        # Stop Pixhawk telemetry
        if self.pixhawk:
            print("   Disconnecting Pixhawk...")
            self.pixhawk.disconnect()
        
        self.is_running = False
        print("✅ Shutdown complete")

# Initialize controller
controller = PiController()

# Socket.IO event handlers
@sio.event
def connect():
    print(f"✅ Connected to server at {SERVER_URL}")
    print(f"   Registering as Pi ID: {PI_ID}")
    
    registration_data = {
        'pi_id': PI_ID,
        'timestamp': datetime.now().isoformat()
    }
    
    print(f"   Sending pi_register event with data: {registration_data}")
    sio.emit('pi_register', registration_data)
    print(f"   ✓ pi_register event emitted")
    
    # Send initial stats
    stats = controller.get_system_stats()
    print(f"   Sending initial system stats")
    sio.emit('system_stats', {'pi_id': PI_ID, 'stats': stats})
    print(f"   ✓ system_stats event emitted")

@sio.event
def disconnect():
    print("❌ Disconnected from server")
    print("   Will attempt to reconnect automatically...")

@sio.on('execute_command')
def handle_command(data):
    """Handle commands from server"""
    print(f"Received command: {data}")
    
    command = data.get('command')
    args = data.get('args', {})
    
    result = controller.execute_command(command, args)
    
    sio.emit('command_result', {
        'pi_id': PI_ID,
        'command': command,
        'result': result,
        'timestamp': datetime.now().isoformat()
    })

@sio.on('request_stats')
def handle_stats_request(data):
    """Send system stats"""
    stats = controller.get_system_stats()
    
    # Send via Socket.IO
    sio.emit('system_stats', {'pi_id': PI_ID, 'stats': stats})
    
    # Also send via MAVLink for dual-channel reliability
    if controller.mavlink_detection_sender:
        controller.mavlink_detection_sender.send_system_stats(stats)
        print(f"📡 System stats sent via both Socket.IO and MAVLink")

@sio.on('start_stream')
def handle_start_stream(data):
    """Start camera stream"""
    settings = data.get('settings', {})
    result = controller.start_camera_stream(settings)
    sio.emit('stream_status', {
        'pi_id': PI_ID,
        'status': 'started' if result['success'] else 'failed',
        'message': result['message']
    })

@sio.on('stop_stream')
def handle_stop_stream(data):
    """Stop camera stream"""
    result = controller.stop_camera_stream()
    sio.emit('stream_status', {
        'pi_id': PI_ID,
        'status': 'stopped',
        'message': result['message']
    })

@sio.on('start_detection')
def handle_start_detection(data):
    """Start yellow crop detection"""
    global streaming_active, camera_active
    
    if not controller.detector:
        sio.emit('detection_status', {
            'pi_id': PI_ID,
            'status': 'failed',
            'message': 'Detector not available'
        })
        return
    
    # Auto-start mission if not already active
    if not controller.mission_active:
        mission_id = f"auto_mission_{int(time.time())}"
        controller.current_mission_id = mission_id
        controller.mission_active = True
        controller.detection_count = 0
        print(f"🚀 Auto-started mission: {mission_id}")
        
        # Notify server about the new mission
        if sio.connected:
            sio.emit('mission_started', {
                'mission_id': mission_id,
                'drone_id': 1,
                'pi_id': PI_ID,
                'timestamp': datetime.now().isoformat(),
                'auto_started': True
            })
            print(f"   📡 Notified server about mission: {mission_id}")
    
    # Auto-start camera stream if not already streaming
    if not streaming_active and not camera_active:
        print("📹 Auto-starting camera for detection...")
        result = controller.start_camera_stream({})
        if result.get('success'):
            print("✅ Camera started successfully")
        else:
            print(f"❌ Camera start failed: {result.get('message')}")
            sio.emit('detection_status', {
                'pi_id': PI_ID,
                'status': 'failed',
                'message': f"Camera start failed: {result.get('message')}"
            })
            return
    elif camera_active and not streaming_active:
        # Camera exists but streaming stopped - restart streaming
        print("📹 Restarting camera stream...")
        streaming_active = True
        Thread(target=controller._stream_frames, daemon=True).start()
    elif streaming_active:
        print("📹 Camera already streaming")
    
    controller.detection_active = True
    print("🌾 Detection started")
    sio.emit('detection_status', {
        'pi_id': PI_ID,
        'status': 'active',
        'message': 'Detection started'
    })

@sio.on('stop_detection')
def handle_stop_detection(data):
    """Stop yellow crop detection"""
    global streaming_active
    
    controller.detection_active = False
    print("🌾 Detection stopped")
    
    # Also stop camera stream
    if streaming_active:
        print("📹 Stopping camera stream...")
        result = controller.stop_camera_stream()
        if result.get('success'):
            print("✅ Camera stopped successfully")
        else:
            print(f"⚠️ Camera stop: {result.get('message')}")
    
    # Stop mission
    if controller.mission_active:
        print(f"🏁 Stopping mission: {controller.current_mission_id}")
        controller.mission_active = False
        controller.current_mission_id = None
    
    sio.emit('detection_status', {
        'pi_id': PI_ID,
        'status': 'inactive',
        'message': 'Detection and camera stopped'
    })

@sio.on('get_detection_stats')
def handle_get_detection_stats(data):
    """Get detection statistics"""
    if controller.detector:
        stats = controller.detector.get_statistics()
        stats['detection_count'] = controller.detection_count
        stats['detection_active'] = controller.detection_active
        sio.emit('detection_stats', {
            'pi_id': PI_ID,
            'stats': stats
        })

@sio.on('test_pi_connection')
def handle_test_pi_connection(data):
    """Test Pi telemetry connection and return system status"""
    try:
        request_timestamp = data.get('timestamp', 0)
        current_timestamp = time.time() * 1000  # Convert to milliseconds
        latency = int(current_timestamp - request_timestamp) if request_timestamp else 0
        
        print(f"📡 Connection test request received (latency: {latency}ms)")
        
        # Gather system status
        response = {
            'status': 'ok',
            'pi_id': PI_ID,
            'latency': latency,
            'timestamp': current_timestamp,
            'pixhawk_connected': controller.pixhawk.connected if controller.pixhawk else False,
            'camera_enabled': CAMERA_ENABLED,
            'detection_enabled': controller.detection_enabled,
            'detection_active': controller.detection_active,
            'mission_active': controller.mission_active,
            'system_info': {
                'cpu_percent': psutil.cpu_percent(interval=0.1),
                'memory_percent': psutil.virtual_memory().percent,
                'disk_percent': psutil.disk_usage('/').percent
            }
        }
        
        # Add telemetry if available
        if controller.pixhawk and controller.pixhawk.connected:
            telemetry = controller.pixhawk.get_telemetry()
            response['telemetry'] = {
                'mode': telemetry.get('mode', 'UNKNOWN'),
                'armed': telemetry.get('armed', False),
                'gps_satellites': telemetry.get('satellites_visible', 0),
                'battery_voltage': telemetry.get('battery_voltage', 0.0)
            }
        
        sio.emit('pi_connection_test_result', response)
        print(f"✅ Connection test response sent (latency: {latency}ms)")
        
    except Exception as e:
        print(f"❌ Connection test failed: {e}")
        sio.emit('pi_connection_test_result', {
            'status': 'error',
            'pi_id': PI_ID,
            'message': str(e)
        })

# ========================================
#          Mission Management            |
# ========================================

@sio.on('mission_started')
def handle_mission_started(data):
    """Server notifies Pi that mission started"""
    controller.current_mission_id = data.get('mission_id')
    controller.mission_active = True
    controller.detection_count = 0  # Reset detection counter
    controller.periodic_capture_count = 0  # Reset periodic capture counter
    print(f"🚀 Mission started: {controller.current_mission_id}")
    
    # Auto-start periodic capture if enabled and camera active
    if controller.periodic_capture_enabled and controller.camera:
        controller.start_periodic_capture()
    
@sio.on('mission_stopped')
def handle_mission_stopped(data):
    """Server notifies Pi that mission stopped"""
    print(f"🏁 Mission stopped: {controller.current_mission_id}")
    
    # Stop periodic capture
    if controller.periodic_capture_active:
        controller.stop_periodic_capture()
    
    controller.current_mission_id = None
    controller.mission_active = False
    controller.detection_active = False  # Auto-stop detection

@sio.on('ping')
def handle_ping(data):
    """Respond to ping"""
    sio.emit('pong', {'pi_id': PI_ID, 'timestamp': time.time()})

@sio.on('request_telemetry')
def handle_telemetry_request(data):
    """Send current drone telemetry"""
    status = controller.get_pixhawk_status()
    sio.emit('drone_telemetry', {
        'pi_id': PI_ID,
        'telemetry': status.get('telemetry', {}),
        'connected': status.get('connected', False),
        'timestamp': datetime.now().isoformat()
    })

@sio.on('reconnect_pixhawk')
def handle_reconnect_pixhawk(data):
    """Reconnect to Pixhawk"""
    result = controller.reconnect_pixhawk()
    sio.emit('pixhawk_status', {
        'pi_id': PI_ID,
        'status': result,
        'timestamp': datetime.now().isoformat()
    })

# ============ MAVLINK COMMAND HANDLERS ============

@sio.on('drone_arm')
def handle_drone_arm(data):
    """ARM the drone"""
    args = data.get('args', {})
    result = controller.arm_vehicle(args)
    sio.emit('drone_command_result', {
        'pi_id': PI_ID,
        'command': 'arm',
        'result': result,
        'timestamp': datetime.now().isoformat()
    })

@sio.on('drone_disarm')
def handle_drone_disarm(data):
    """DISARM the drone"""
    args = data.get('args', {})
    result = controller.disarm_vehicle(args)
    sio.emit('drone_command_result', {
        'pi_id': PI_ID,
        'command': 'disarm',
        'result': result,
        'timestamp': datetime.now().isoformat()
    })

@sio.on('drone_set_mode')
def handle_drone_set_mode(data):
    """Change flight mode"""
    args = data.get('args', {})
    result = controller.set_flight_mode(args)
    sio.emit('drone_command_result', {
        'pi_id': PI_ID,
        'command': 'set_mode',
        'result': result,
        'timestamp': datetime.now().isoformat()
    })

@sio.on('drone_takeoff')
def handle_drone_takeoff(data):
    """Takeoff to specified altitude"""
    args = data.get('args', {})
    result = controller.takeoff_vehicle(args)
    sio.emit('drone_command_result', {
        'pi_id': PI_ID,
        'command': 'takeoff',
        'result': result,
        'timestamp': datetime.now().isoformat()
    })

@sio.on('drone_land')
def handle_drone_land(data):
    """Land the drone"""
    args = data.get('args', {})
    result = controller.land_vehicle(args)
    sio.emit('drone_command_result', {
        'pi_id': PI_ID,
        'command': 'land',
        'result': result,
        'timestamp': datetime.now().isoformat()
    })

@sio.on('drone_goto')
def handle_drone_goto(data):
    """Navigate to GPS location"""
    args = data.get('args', {})
    result = controller.goto_location(args)
    sio.emit('drone_command_result', {
        'pi_id': PI_ID,
        'command': 'goto',
        'result': result,
        'timestamp': datetime.now().isoformat()
    })

@sio.on('drone_upload_mission')
def handle_drone_upload_mission(data):
    """Upload multi-waypoint mission"""
    args = data.get('args', {})
    result = controller.upload_mission(args)
    sio.emit('drone_command_result', {
        'pi_id': PI_ID,
        'command': 'upload_mission',
        'result': result,
        'timestamp': datetime.now().isoformat()
    })

@sio.on('drone_start_mission')
def handle_drone_start_mission(data):
    """Start mission execution"""
    result = controller.start_mission()
    sio.emit('drone_command_result', {
        'pi_id': PI_ID,
        'command': 'start_mission',
        'result': result,
        'timestamp': datetime.now().isoformat()
    })

@sio.on('drone_clear_mission')
def handle_drone_clear_mission(data):
    """Clear mission"""
    result = controller.clear_mission()
    sio.emit('drone_command_result', {
        'pi_id': PI_ID,
        'command': 'clear_mission',
        'result': result,
        'timestamp': datetime.now().isoformat()
    })

@sio.on('drone_rtl')
def handle_drone_rtl(data):
    """Return to launch"""
    result = controller.return_to_launch()
    sio.emit('drone_command_result', {
        'pi_id': PI_ID,
        'command': 'rtl',
        'result': result,
        'timestamp': datetime.now().isoformat()
    })

@sio.on('upload_mission')
def handle_upload_mission(data):
    """Upload KML-generated mission to Pixhawk and launch autonomously (NIdar competition)"""
    try:
        mission_id = data.get('mission_id')
        mission_file = data.get('mission_file')
        mission_data = data.get('mission_data')
        
        print(f"\n{'='*60}")
        print(f"🚀 NIdar Competition: Autonomous Mission Launch")
        print(f"{'='*60}")
        print(f"📤 Mission ID: {mission_id}")
        print(f"📁 Mission file: {mission_file}")
        print(f"📍 Waypoints: {len(mission_data.get('waypoints', []))}")
        print(f"\n⚠️  Per NIdar rules 7.24-7.26:")
        print(f"   - Only KML upload and launch trigger allowed")
        print(f"   - NO manual intervention after launch")
        print(f"   - Drone flies autonomously until complete")
        print(f"{'='*60}\n")
        
        # Import mission uploader
        from modules.mission_uploader import MissionUploader
        
        # Get Pixhawk connection string
        pixhawk_connection = config.get('pixhawk', {}).get('connection_string', '/dev/serial0')
        
        # Create uploader and connect
        print("🔌 Connecting to Pixhawk...")
        uploader = MissionUploader(pixhawk_connection)
        
        if not uploader.connect():
            raise Exception("Failed to connect to Pixhawk")
        
        # Upload mission waypoints
        print("\n📤 Uploading mission waypoints...")
        waypoints = mission_data.get('waypoints', [])
        if not uploader.upload_mission(waypoints):
            raise Exception("Failed to upload mission waypoints")
        
        print(f"✅ Mission uploaded: {len(waypoints)} waypoints")
        
        # Notify mission uploaded
        sio.emit('mission_upload_status', {
            'success': True,
            'mission_id': mission_id,
            'waypoints_count': len(waypoints),
            'message': f'Mission uploaded: {len(waypoints)} waypoints'
        })
        
        # Wait a moment for confirmation
        time.sleep(2)
        
        # Execute autonomous launch sequence (NIdar competition compliance)
        print("\n🚀 Initiating autonomous launch sequence...")
        
        # Notify: Arming
        sio.emit('autonomous_launch_status', {
            'stage': 'arming',
            'message': 'Arming drone...'
        })
        
        if not uploader.autonomous_launch():
            raise Exception("Autonomous launch sequence failed")
        
        # Notify: Flying
        sio.emit('autonomous_launch_status', {
            'stage': 'flying',
            'message': 'Autonomous flight in progress'
        })
        
        print("\n✅ Autonomous mission launched successfully!")
        print(f"🚁 Drone flying autonomously")
        print(f"🌾 Detection system active")
        print(f"📍 Will return automatically when complete\n")
        
        # Close connection
        uploader.close()
        
    except Exception as e:
        print(f"\n❌ Mission launch failed: {e}\n")
        sio.emit('mission_upload_status', {
            'success': False,
            'error': str(e)
        })
        sio.emit('autonomous_launch_status', {
            'error': str(e)
        })

@sio.on('drone_emergency_stop')
def handle_drone_emergency_stop(data):
    """Emergency stop"""
    result = controller.emergency_stop()
    sio.emit('drone_command_result', {
        'pi_id': PI_ID,
        'command': 'emergency_stop',
        'result': result,
        'timestamp': datetime.now().isoformat()
    })

@sio.on('drone_get_status')
def handle_drone_get_status(data):
    """Get drone command status"""
    if controller.commander:
        status = controller.commander.get_status()
        sio.emit('drone_status', {
            'pi_id': PI_ID,
            'status': status,
            'timestamp': datetime.now().isoformat()
        })
    else:
        sio.emit('drone_status', {
            'pi_id': PI_ID,
            'error': 'Commander not available',
            'timestamp': datetime.now().isoformat()
        })

# ============ SAFETY COMMAND HANDLERS ============

@sio.on('safety_set_geofence')
def handle_safety_set_geofence(data):
    """Set geofence center"""
    args = data.get('args', {})
    result = controller.set_geofence(args)
    sio.emit('safety_command_result', {
        'pi_id': PI_ID,
        'command': 'set_geofence',
        'result': result,
        'timestamp': datetime.now().isoformat()
    })

@sio.on('safety_check_pre_flight')
def handle_safety_check_pre_flight(data):
    """Run pre-flight checklist"""
    result = controller.check_pre_flight()
    sio.emit('safety_command_result', {
        'pi_id': PI_ID,
        'command': 'check_pre_flight',
        'result': result,
        'timestamp': datetime.now().isoformat()
    })

@sio.on('safety_get_status')
def handle_safety_get_status(data):
    """Get safety status"""
    result = controller.get_safety_status()
    sio.emit('safety_status', {
        'pi_id': PI_ID,
        'status': result.get('data', {}),
        'timestamp': datetime.now().isoformat()
    })

def main():
    """Main function"""
    print(f"Starting Pi Controller for {PI_ID}")
    print(f"Connecting to {SERVER_URL}")
    
    try:
        # Connect to server
        sio.connect(SERVER_URL)
        
        # Send stats periodically
        stats_counter = 0
        while controller.is_running:
            time.sleep(10)
            stats_counter += 1
            stats = controller.get_system_stats()
            
            # Try Socket.IO first (WiFi)
            if sio.connected:
                sio.emit('system_stats', {'pi_id': PI_ID, 'stats': stats})
                print(f"📊 System stats sent via Socket.IO (WiFi)")
            # Fallback to MAVLink if out of WiFi range
            elif controller.mavlink_detection_sender:
                success = controller.mavlink_detection_sender.send_system_stats(stats)
                if success:
                    print(f"📡 System stats sent via MAVLink (out of WiFi range)")
                else:
                    print(f"⚠️  Failed to send system stats (no connectivity)")
            
            # Send system stats via MAVLink every 60 seconds (every 6th cycle) even when WiFi is available
            # This ensures GCS has dual-channel visibility for reliability monitoring
            if stats_counter % 6 == 0 and controller.mavlink_detection_sender:
                controller.mavlink_detection_sender.send_system_stats(stats)
                print(f"📡 System stats also sent via MAVLink (periodic dual transmission)")
    
    except KeyboardInterrupt:
        print("\nShutting down...")
    except Exception as e:
        print(f"Error: {e}")
    finally:
        controller.shutdown()
        if sio.connected:
            sio.disconnect()

if __name__ == '__main__':
    main()
