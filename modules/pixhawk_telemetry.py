"""
Pixhawk Telemetry Module - Retrieves drone telemetry from Pixhawk flight controller
Runs on Raspberry Pi connected to Pixhawk via serial/USB
"""

import time
import threading
from typing import Dict, Optional, Callable
from datetime import datetime
import json

# Try to import dronekit/pymavlink
try:
    from dronekit import connect, VehicleMode, LocationGlobalRelative
    from pymavlink import mavutil
    MAVLINK_AVAILABLE = True
except ImportError:
    MAVLINK_AVAILABLE = False
    print("⚠️  DroneKit not available. Install with: pip install dronekit pymavlink")


class PixhawkTelemetry:
    """
    Manages communication with Pixhawk flight controller via MAVLink protocol.
    Retrieves real-time telemetry data including GPS, attitude, battery, and flight mode.
    """
    
    def __init__(self, config: dict = None):
        """
        Initialize Pixhawk telemetry interface
        
        Args:
            config: Configuration dictionary with connection parameters
        """
        self.config = config or {}
        
        # Connection parameters
        self.connection_string = self.config.get('connection_string', '/dev/ttyACM0')
        self.baud_rate = self.config.get('baud_rate', 57600)
        self.simulation_mode = self.config.get('simulation_mode', not MAVLINK_AVAILABLE)
        self.update_rate = self.config.get('update_rate', 1.0)  # Hz
        
        # Vehicle connection
        self.vehicle = None
        self.connected = False
        
        # Telemetry data storage
        self.telemetry_data = {
            'connected': False,
            'timestamp': None,
            'gps': {
                'lat': 0.0,
                'lon': 0.0,
                'alt': 0.0,
                'relative_alt': 0.0,
                'satellites': 0,
                'fix_type': 0,
                'hdop': 0.0
            },
            'attitude': {
                'roll': 0.0,
                'pitch': 0.0,
                'yaw': 0.0
            },
            'velocity': {
                'vx': 0.0,
                'vy': 0.0,
                'vz': 0.0,
                'groundspeed': 0.0,
                'airspeed': 0.0
            },
            'battery': {
                'voltage': 0.0,
                'current': 0.0,
                'level': 0,
                'remaining': 0
            },
            'flight_mode': 'UNKNOWN',
            'armed': False,
            'system_status': 'UNKNOWN',
            'ekf_ok': False,
            'heading': 0
        }
        
        # Background update thread
        self.update_thread = None
        self.running = False
        self.lock = threading.Lock()
        
        # Callbacks
        self.telemetry_callback = None
        
        # Simulation data for testing
        self.sim_counter = 0
        
        if not MAVLINK_AVAILABLE and not self.simulation_mode:
            print("⚠️  MAVLink libraries not available. Running in simulation mode.")
            self.simulation_mode = True
    
    def connect(self) -> bool:
        """
        Connect to Pixhawk flight controller
        
        Returns:
            bool: True if connection successful
        """
        if self.simulation_mode:
            print("🔌 Running in SIMULATION mode - generating simulated telemetry")
            self.connected = True
            with self.lock:
                self.telemetry_data['connected'] = True
            return True
        
        if not MAVLINK_AVAILABLE:
            print("❌ Cannot connect: DroneKit not available")
            return False
        
        try:
            print(f"🔌 Connecting to Pixhawk on {self.connection_string} at {self.baud_rate} baud...")
            
            # Attempt connection with timeout
            self.vehicle = connect(
                self.connection_string,
                baud=self.baud_rate,
                wait_ready=True,
                timeout=30,
                heartbeat_timeout=30
            )
            
            if self.vehicle:
                self.connected = True
                with self.lock:
                    self.telemetry_data['connected'] = True
                
                print("✅ Connected to Pixhawk successfully")
                print(f"   Autopilot: {self.vehicle.version}")
                print(f"   Vehicle Mode: {self.vehicle.mode.name}")
                
                # Set up message listeners
                self._setup_listeners()
                
                return True
            
        except Exception as e:
            print(f"❌ Failed to connect to Pixhawk: {e}")
            self.connected = False
            return False
        
        return False
    
    def _setup_listeners(self):
        """Set up MAVLink message listeners for real-time updates"""
        if not self.vehicle:
            return
        
        # Add attribute listeners for automatic updates
        @self.vehicle.on_attribute('location')
        def location_callback(vehicle, attr_name, value):
            self._update_gps_data()
        
        @self.vehicle.on_attribute('attitude')
        def attitude_callback(vehicle, attr_name, value):
            self._update_attitude_data()
        
        @self.vehicle.on_attribute('mode')
        def mode_callback(vehicle, attr_name, value):
            with self.lock:
                self.telemetry_data['flight_mode'] = str(value.name)
    
    def disconnect(self):
        """Disconnect from Pixhawk"""
        self.stop_telemetry_updates()
        
        if self.vehicle and not self.simulation_mode:
            try:
                self.vehicle.close()
                print("🔌 Disconnected from Pixhawk")
            except:
                pass
        
        self.connected = False
        with self.lock:
            self.telemetry_data['connected'] = False
    
    def start_telemetry_updates(self, callback: Optional[Callable] = None):
        """
        Start background thread for telemetry updates
        
        Args:
            callback: Optional callback function to receive telemetry updates
        """
        if self.running:
            print("⚠️  Telemetry updates already running")
            return
        
        self.telemetry_callback = callback
        self.running = True
        self.update_thread = threading.Thread(target=self._telemetry_update_loop, daemon=True)
        self.update_thread.start()
        print(f"📡 Started telemetry updates at {self.update_rate} Hz")
    
    def stop_telemetry_updates(self):
        """Stop background telemetry updates"""
        self.running = False
        if self.update_thread:
            self.update_thread.join(timeout=2)
        print("📡 Stopped telemetry updates")
    
    def _telemetry_update_loop(self):
        """Background loop for updating telemetry data"""
        interval = 1.0 / self.update_rate
        
        while self.running:
            try:
                if self.simulation_mode:
                    self._update_simulated_telemetry()
                else:
                    self._update_real_telemetry()
                
                # Call callback if registered
                if self.telemetry_callback:
                    telemetry_copy = self.get_telemetry()
                    self.telemetry_callback(telemetry_copy)
                
                time.sleep(interval)
                
            except Exception as e:
                print(f"❌ Telemetry update error: {e}")
                time.sleep(interval)
    
    def _update_real_telemetry(self):
        """Update telemetry from real Pixhawk connection"""
        if not self.vehicle:
            return
        
        with self.lock:
            self.telemetry_data['timestamp'] = datetime.now().isoformat()
            
            # GPS data
            if self.vehicle.location.global_relative_frame:
                loc = self.vehicle.location.global_relative_frame
                self.telemetry_data['gps']['lat'] = loc.lat or 0.0
                self.telemetry_data['gps']['lon'] = loc.lon or 0.0
                self.telemetry_data['gps']['alt'] = loc.alt or 0.0
            
            if self.vehicle.location.global_frame:
                self.telemetry_data['gps']['relative_alt'] = self.vehicle.location.global_frame.alt or 0.0
            
            # GPS quality
            if self.vehicle.gps_0:
                self.telemetry_data['gps']['satellites'] = self.vehicle.gps_0.satellites_visible or 0
                self.telemetry_data['gps']['fix_type'] = self.vehicle.gps_0.fix_type or 0
                self.telemetry_data['gps']['hdop'] = getattr(self.vehicle.gps_0, 'eph', 0) / 100.0
            
            # Attitude data
            if self.vehicle.attitude:
                self.telemetry_data['attitude']['roll'] = round(self.vehicle.attitude.roll, 3)
                self.telemetry_data['attitude']['pitch'] = round(self.vehicle.attitude.pitch, 3)
                self.telemetry_data['attitude']['yaw'] = round(self.vehicle.attitude.yaw, 3)
            
            # Velocity data
            if self.vehicle.velocity:
                vx, vy, vz = self.vehicle.velocity
                self.telemetry_data['velocity']['vx'] = vx or 0.0
                self.telemetry_data['velocity']['vy'] = vy or 0.0
                self.telemetry_data['velocity']['vz'] = vz or 0.0
            
            self.telemetry_data['velocity']['groundspeed'] = self.vehicle.groundspeed or 0.0
            self.telemetry_data['velocity']['airspeed'] = self.vehicle.airspeed or 0.0
            
            # Battery data
            if self.vehicle.battery:
                self.telemetry_data['battery']['voltage'] = self.vehicle.battery.voltage or 0.0
                self.telemetry_data['battery']['current'] = self.vehicle.battery.current or 0.0
                self.telemetry_data['battery']['level'] = self.vehicle.battery.level or 0
            
            # Flight status
            self.telemetry_data['flight_mode'] = str(self.vehicle.mode.name)
            self.telemetry_data['armed'] = self.vehicle.armed
            self.telemetry_data['system_status'] = str(self.vehicle.system_status.state)
            self.telemetry_data['ekf_ok'] = self.vehicle.ekf_ok
            self.telemetry_data['heading'] = self.vehicle.heading or 0
    
    def _update_gps_data(self):
        """Update GPS data from vehicle"""
        if not self.vehicle:
            return
        
        with self.lock:
            if self.vehicle.location.global_relative_frame:
                loc = self.vehicle.location.global_relative_frame
                self.telemetry_data['gps']['lat'] = loc.lat or 0.0
                self.telemetry_data['gps']['lon'] = loc.lon or 0.0
                self.telemetry_data['gps']['alt'] = loc.alt or 0.0
    
    def _update_attitude_data(self):
        """Update attitude data from vehicle"""
        if not self.vehicle:
            return
        
        with self.lock:
            if self.vehicle.attitude:
                self.telemetry_data['attitude']['roll'] = round(self.vehicle.attitude.roll, 3)
                self.telemetry_data['attitude']['pitch'] = round(self.vehicle.attitude.pitch, 3)
                self.telemetry_data['attitude']['yaw'] = round(self.vehicle.attitude.yaw, 3)
    
    def _update_simulated_telemetry(self):
        """Generate simulated telemetry data for testing"""
        import math
        
        self.sim_counter += 1
        t = self.sim_counter * 0.1
        
        with self.lock:
            self.telemetry_data['timestamp'] = datetime.now().isoformat()
            
            # Simulate GPS coordinates (circular path)
            center_lat = 40.7128
            center_lon = -74.0060
            radius = 0.001  # ~111 meters
            
            self.telemetry_data['gps']['lat'] = center_lat + radius * math.sin(t)
            self.telemetry_data['gps']['lon'] = center_lon + radius * math.cos(t)
            self.telemetry_data['gps']['alt'] = 50.0 + 10.0 * math.sin(t * 0.5)
            self.telemetry_data['gps']['relative_alt'] = self.telemetry_data['gps']['alt']
            self.telemetry_data['gps']['satellites'] = 12
            self.telemetry_data['gps']['fix_type'] = 3  # 3D fix
            self.telemetry_data['gps']['hdop'] = 0.8
            
            # Simulate attitude
            self.telemetry_data['attitude']['roll'] = 0.1 * math.sin(t * 2)
            self.telemetry_data['attitude']['pitch'] = 0.05 * math.cos(t * 2)
            self.telemetry_data['attitude']['yaw'] = (t % (2 * math.pi))
            
            # Simulate velocity
            self.telemetry_data['velocity']['groundspeed'] = 5.0 + 2.0 * math.sin(t * 0.5)
            self.telemetry_data['velocity']['airspeed'] = 5.5 + 2.0 * math.sin(t * 0.5)
            self.telemetry_data['velocity']['vx'] = 2.0 * math.cos(t)
            self.telemetry_data['velocity']['vy'] = 2.0 * math.sin(t)
            self.telemetry_data['velocity']['vz'] = 0.5 * math.sin(t * 0.5)
            
            # Simulate battery (slowly draining)
            battery_base = 12.6
            drain = (self.sim_counter * 0.0001) % 2.0
            self.telemetry_data['battery']['voltage'] = battery_base - drain
            self.telemetry_data['battery']['current'] = 15.0 + 5.0 * math.sin(t)
            self.telemetry_data['battery']['level'] = max(0, 100 - int(drain * 50))
            
            # Flight status
            modes = ['STABILIZE', 'GUIDED', 'AUTO', 'LOITER']
            mode_index = (self.sim_counter // 100) % len(modes)
            self.telemetry_data['flight_mode'] = modes[mode_index]
            self.telemetry_data['armed'] = True
            self.telemetry_data['system_status'] = 'ACTIVE'
            self.telemetry_data['ekf_ok'] = True
            self.telemetry_data['heading'] = int(math.degrees(t)) % 360
    
    def get_telemetry(self) -> Dict:
        """
        Get current telemetry data (thread-safe)
        
        Returns:
            dict: Current telemetry data
        """
        with self.lock:
            return self.telemetry_data.copy()
    
    def get_gps_location(self) -> tuple:
        """
        Get current GPS coordinates
        
        Returns:
            tuple: (latitude, longitude, altitude)
        """
        with self.lock:
            gps = self.telemetry_data['gps']
            return (gps['lat'], gps['lon'], gps['alt'])
    
    def is_connected(self) -> bool:
        """Check if connected to Pixhawk"""
        return self.connected
    
    def get_connection_status(self) -> Dict:
        """
        Get detailed connection status
        
        Returns:
            dict: Connection status information
        """
        return {
            'connected': self.connected,
            'simulation_mode': self.simulation_mode,
            'connection_string': self.connection_string,
            'mavlink_available': MAVLINK_AVAILABLE
        }


# Example usage
if __name__ == '__main__':
    """Test the Pixhawk telemetry module"""
    
    print("🚁 Testing Pixhawk Telemetry Module")
    print("-" * 50)
    
    # Configuration
    config = {
        'connection_string': '/dev/ttyACM0',
        'baud_rate': 57600,
        'simulation_mode': True,  # Set to False to connect to real Pixhawk
        'update_rate': 2.0  # 2 Hz for testing
    }
    
    # Create telemetry instance
    telemetry = PixhawkTelemetry(config)
    
    # Define callback for telemetry updates
    def on_telemetry_update(data):
        print(f"\n📡 Telemetry Update:")
        print(f"   GPS: {data['gps']['lat']:.6f}, {data['gps']['lon']:.6f}, {data['gps']['alt']:.1f}m")
        print(f"   Mode: {data['flight_mode']}, Armed: {data['armed']}")
        print(f"   Battery: {data['battery']['voltage']:.2f}V ({data['battery']['level']}%)")
        print(f"   Speed: {data['velocity']['groundspeed']:.1f} m/s")
    
    # Connect to Pixhawk
    if telemetry.connect():
        # Start telemetry updates
        telemetry.start_telemetry_updates(callback=on_telemetry_update)
        
        try:
            print("\n✅ Telemetry running. Press Ctrl+C to stop...")
            while True:
                time.sleep(1)
        
        except KeyboardInterrupt:
            print("\n\n🛑 Stopping telemetry...")
        finally:
            telemetry.disconnect()
            print("✅ Shutdown complete")
    else:
        print("❌ Failed to connect to Pixhawk")
