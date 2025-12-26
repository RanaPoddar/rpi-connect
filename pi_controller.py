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
        self.pixhawk_enabled = PIXHAWK_ENABLED and PIXHAWK_MODULE_AVAILABLE
        
        # Initialize Pixhawk telemetry if enabled
        if self.pixhawk_enabled:
            print("🚁 Initializing Pixhawk telemetry...")
            self.pixhawk = PixhawkTelemetry(PIXHAWK_CONFIG)
            if self.pixhawk.connect():
                print("✅ Pixhawk telemetry initialized")
                # Start telemetry updates with callback
                self.pixhawk.start_telemetry_updates(callback=self._on_telemetry_update)
            else:
                print("❌ Failed to initialize Pixhawk telemetry")
                self.pixhawk = None
        else:
            print("ℹ️  Pixhawk telemetry disabled or module not available")
        
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
        except Exception as e:
            print(f"Error sending telemetry: {e}")
    
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
            
            elif command == 'capture_image':
                return self.capture_image()
            
            elif command == 'pixhawk_status':
                return {'success': True, 'data': self.get_pixhawk_status()}
            
            elif command == 'pixhawk_reconnect':
                return self.reconnect_pixhawk()
            
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
        global CAMERA_ENABLED
        
        if not CAMERA_ENABLED:
            return {'success': False, 'message': 'Camera disabled'}
        
        try:
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
                
                # Capture to numpy array and encode
                image_array = camera.capture_array()
                camera.stop()
                camera.close()
                
                # Convert RGB to BGR for JPEG encoding
                import cv2
                _, buffer = cv2.imencode('.jpg', cv2.cvtColor(image_array, cv2.COLOR_RGB2BGR), [cv2.IMWRITE_JPEG_QUALITY, 85])
                image_data = base64.b64encode(buffer).decode('utf-8')
                
                return {
                    'success': True,
                    'message': 'Image captured',
                    'image': image_data,
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
                        "AwbEnable": True,
                        "AwbMode": 4,  # Daylight mode (natural colors for outdoor/HQ camera)
                        "Saturation": 1.0,  # Normal saturation
                        "Contrast": 1.0,    # Normal contrast
                        "Brightness": 0.0,  # Normal brightness
                        "Sharpness": 1.0    # Normal sharpness
                    }
                )
                self.camera.configure(config)
                self.camera.start()
                
                # Wait for camera auto white balance to settle (critical for accurate colors)
                print(f"Camera starting - waiting 3 seconds for white balance to settle...")
                time.sleep(3)
                print(f"Camera ready with Daylight white balance mode")
                
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
    
    def shutdown(self):
        """Clean shutdown of all systems"""
        print("\n🛑 Shutting down Pi Controller...")
        
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
