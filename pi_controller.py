#!/usr/bin/env python3
"""
Raspberry Pi Controller - Runs on the Pi to receive commands via Socket.IO
Handles camera, system control, WebRTC streaming, and drone telemetry
"""

# Python 3.13 compatibility fix for dronekit
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
    print(f"⚠️  Pixhawk telemetry module not available: {e}")
    PIXHAWK_MODULE_AVAILABLE = False
    PixhawkTelemetry = None

# Import MAVLink commander module
try:
    from modules.mavlink_commander import MAVLinkCommander
    MAVLINK_COMMANDER_AVAILABLE = True
except ImportError as e:
    print(f"⚠️  MAVLink commander module not available: {e}")
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
        self.pixhawk_enabled = PIXHAWK_ENABLED and PIXHAWK_MODULE_AVAILABLE
        
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
                config = self.camera.create_video_configuration(
                    main={"size": resolution, "format": "RGB888"},
                    controls={
                        "FrameRate": settings.get('framerate', 30),
                        "AeEnable": True,  # Auto exposure
                        "AwbEnable": False,  # Disable auto, use manual gains
                        "ColourGains": (1.5, 1.5),  # Manual white balance (Red, Blue) - reduces blue tint
                        "Saturation": 1.0,
                        "Contrast": 1.0,
                        "Brightness": 0.0,
                        "Sharpness": 1.0
                    }
                )
                self.camera.configure(config)
                self.camera.start()
                
                # Wait for camera auto exposure to settle
                print(f"Camera starting - waiting 2 seconds for exposure to settle...")
                time.sleep(2)
                print(f"Camera ready with manual white balance (daylight tuned)")
                
                camera_active = True
                streaming_active = True
                
                # Start streaming thread
                Thread(target=self._stream_frames, daemon=True).start()
                
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
        
        return {'success': True, 'message': 'Streaming stopped'}
    
    def _stream_frames(self):
        """Stream camera frames via Socket.IO"""
        global streaming_active
        import cv2
        
        try:
            while streaming_active:
                # Capture frame as numpy array
                frame = self.camera.capture_array()
                
                if not streaming_active:
                    break
                
                # Convert RGB to BGR and encode as JPEG
                frame_bgr = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
                _, buffer = cv2.imencode('.jpg', frame_bgr, [cv2.IMWRITE_JPEG_QUALITY, 75])
                frame_data = base64.b64encode(buffer).decode('utf-8')
                
                # Send frame to server
                sio.emit('camera_frame', {
                    'pi_id': PI_ID,
                    'frame': frame_data,
                    'timestamp': time.time()
                })
                
                time.sleep(0.033)  # ~30 fps
                
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
            return result
        except Exception as e:
            return {'success': False, 'message': f'Mission start failed: {str(e)}'}
    
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
    print(f"Connected to server at {SERVER_URL}")
    sio.emit('pi_register', {
        'pi_id': PI_ID,
        'timestamp': datetime.now().isoformat()
    })
    
    # Send initial stats
    stats = controller.get_system_stats()
    sio.emit('system_stats', {'pi_id': PI_ID, 'stats': stats})

@sio.event
def disconnect():
    print("Disconnected from server")

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
    sio.emit('system_stats', {'pi_id': PI_ID, 'stats': stats})

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
        while controller.is_running:
            time.sleep(10)
            if sio.connected:
                stats = controller.get_system_stats()
                sio.emit('system_stats', {'pi_id': PI_ID, 'stats': stats})
    
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
