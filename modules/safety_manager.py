"""
Safety Manager Module - Drone flight safety features
Provides geofencing, battery monitoring, connection monitoring, and emergency procedures
"""

import time
import threading
import math
from typing import Dict, Optional, Callable, List, Tuple
from datetime import datetime


class SafetyManager:
    """
    Manages drone flight safety features including geofencing,
    battery monitoring, and emergency procedures.
    """
    
    def __init__(self, config: dict = None):
        """
        Initialize Safety Manager
        
        Args:
            config: Configuration dictionary with safety parameters
        """
        self.config = config or {}
        
        # Geofence configuration
        self.geofence_enabled = self.config.get('geofence_enabled', True)
        self.geofence_center = self.config.get('geofence_center', None)  # (lat, lon)
        self.geofence_radius = self.config.get('geofence_radius', 500)  # meters
        self.geofence_max_altitude = self.config.get('geofence_max_altitude', 120)  # meters
        self.geofence_min_altitude = self.config.get('geofence_min_altitude', 2)  # meters
        
        # Battery safety
        self.battery_rtl_threshold = self.config.get('battery_rtl_threshold', 20)  # percent
        self.battery_land_threshold = self.config.get('battery_land_threshold', 10)  # percent
        self.battery_critical_voltage = self.config.get('battery_critical_voltage', 10.5)  # volts (3S)
        
        # Connection monitoring
        self.connection_timeout = self.config.get('connection_timeout', 10)  # seconds
        self.telemetry_timeout = self.config.get('telemetry_timeout', 5)  # seconds
        
        # State tracking
        self.last_telemetry_time = time.time()
        self.last_command_time = time.time()
        self.violation_count = 0
        self.safety_violations = []
        self.battery_warnings_sent = []
        
        # Callbacks
        self.warning_callback = None
        self.emergency_callback = None
        
        # Monitoring thread
        self.monitoring_active = False
        self.monitor_thread = None
        self.monitor_lock = threading.Lock()
        
        print(f"🛡️  Safety Manager initialized")
        print(f"   Geofence: {'Enabled' if self.geofence_enabled else 'Disabled'}")
        if self.geofence_enabled and self.geofence_center:
            print(f"   - Center: {self.geofence_center}")
            print(f"   - Radius: {self.geofence_radius}m")
            print(f"   - Max altitude: {self.geofence_max_altitude}m")
        print(f"   Battery RTL: {self.battery_rtl_threshold}%")
        print(f"   Battery LAND: {self.battery_land_threshold}%")
    
    # ============ GEOFENCING ============
    
    @staticmethod
    def calculate_distance(lat1: float, lon1: float, lat2: float, lon2: float) -> float:
        """
        Calculate distance between two GPS coordinates
        
        Args:
            lat1, lon1: First coordinate
            lat2, lon2: Second coordinate
        
        Returns:
            float: Distance in meters
        """
        R = 6371000  # Earth's radius in meters
        
        phi1 = math.radians(lat1)
        phi2 = math.radians(lat2)
        delta_phi = math.radians(lat2 - lat1)
        delta_lambda = math.radians(lon2 - lon1)
        
        a = math.sin(delta_phi / 2) ** 2 + \
            math.cos(phi1) * math.cos(phi2) * math.sin(delta_lambda / 2) ** 2
        c = 2 * math.atan2(math.sqrt(a), math.sqrt(1 - a))
        
        return R * c
    
    def set_geofence_center(self, lat: float, lon: float):
        """
        Set geofence center point (usually home location)
        
        Args:
            lat: Latitude
            lon: Longitude
        """
        self.geofence_center = (lat, lon)
        print(f"🗺️  Geofence center set to ({lat:.6f}, {lon:.6f})")
    
    def check_geofence(self, lat: float, lon: float, altitude: float) -> Dict:
        """
        Check if position is within geofence boundaries
        
        Args:
            lat: Current latitude
            lon: Current longitude
            altitude: Current altitude (relative)
        
        Returns:
            dict: Geofence check result
        """
        if not self.geofence_enabled:
            return {
                'within_bounds': True,
                'message': 'Geofence disabled',
                'timestamp': datetime.now().isoformat()
            }
        
        violations = []
        
        # Check horizontal boundary
        if self.geofence_center:
            center_lat, center_lon = self.geofence_center
            distance = self.calculate_distance(center_lat, center_lon, lat, lon)
            
            if distance > self.geofence_radius:
                violations.append(f'Outside horizontal boundary ({distance:.1f}m from center)')
        
        # Check altitude boundaries
        if altitude > self.geofence_max_altitude:
            violations.append(f'Above max altitude ({altitude:.1f}m > {self.geofence_max_altitude}m)')
        
        if altitude < self.geofence_min_altitude and altitude > 0.5:  # Allow ground level
            violations.append(f'Below min altitude ({altitude:.1f}m < {self.geofence_min_altitude}m)')
        
        within_bounds = len(violations) == 0
        
        return {
            'within_bounds': within_bounds,
            'violations': violations,
            'distance_from_center': distance if self.geofence_center else 0,
            'altitude': altitude,
            'timestamp': datetime.now().isoformat()
        }
    
    def check_waypoint_safe(self, lat: float, lon: float, altitude: float) -> Dict:
        """
        Check if a waypoint is safe to navigate to
        
        Args:
            lat: Target latitude
            lon: Target longitude
            altitude: Target altitude
        
        Returns:
            dict: Safety check result
        """
        check = self.check_geofence(lat, lon, altitude)
        
        if not check['within_bounds']:
            return {
                'safe': False,
                'reason': 'Waypoint outside geofence',
                'violations': check['violations'],
                'timestamp': datetime.now().isoformat()
            }
        
        return {
            'safe': True,
            'message': 'Waypoint is safe',
            'timestamp': datetime.now().isoformat()
        }
    
    # ============ BATTERY MONITORING ============
    
    def check_battery_level(self, voltage: float, percentage: int, armed: bool) -> Dict:
        """
        Check battery level and determine if action is needed
        
        Args:
            voltage: Battery voltage
            percentage: Battery percentage (0-100)
            armed: Whether vehicle is armed
        
        Returns:
            dict: Battery check result with recommended action
        """
        action_required = None
        severity = 'normal'
        messages = []
        
        # Critical voltage check
        if voltage > 0 and voltage < self.battery_critical_voltage:
            action_required = 'LAND_IMMEDIATE'
            severity = 'critical'
            messages.append(f'Critical voltage: {voltage:.2f}V < {self.battery_critical_voltage}V')
        
        # Percentage checks (only if armed and flying)
        if armed and percentage > 0:
            if percentage <= self.battery_land_threshold:
                if action_required != 'LAND_IMMEDIATE':
                    action_required = 'LAND_NOW'
                severity = 'critical'
                messages.append(f'Battery critical: {percentage}% <= {self.battery_land_threshold}%')
            
            elif percentage <= self.battery_rtl_threshold:
                if action_required is None:
                    action_required = 'RTL'
                severity = 'warning'
                messages.append(f'Battery low: {percentage}% <= {self.battery_rtl_threshold}%')
        
        return {
            'action_required': action_required,
            'severity': severity,
            'voltage': voltage,
            'percentage': percentage,
            'messages': messages,
            'timestamp': datetime.now().isoformat()
        }
    
    # ============ CONNECTION MONITORING ============
    
    def update_telemetry_timestamp(self):
        """Update last telemetry received timestamp"""
        self.last_telemetry_time = time.time()
    
    def update_command_timestamp(self):
        """Update last command sent timestamp"""
        self.last_command_time = time.time()
    
    def check_connection_health(self) -> Dict:
        """
        Check connection health status
        
        Returns:
            dict: Connection health information
        """
        current_time = time.time()
        telemetry_age = current_time - self.last_telemetry_time
        command_age = current_time - self.last_command_time
        
        telemetry_timeout = telemetry_age > self.telemetry_timeout
        connection_timeout = telemetry_age > self.connection_timeout
        
        status = 'healthy'
        action_required = None
        messages = []
        
        if connection_timeout:
            status = 'connection_lost'
            action_required = 'RTL'
            messages.append(f'Connection lost for {telemetry_age:.1f}s')
        elif telemetry_timeout:
            status = 'degraded'
            messages.append(f'Telemetry delayed: {telemetry_age:.1f}s')
        
        return {
            'status': status,
            'action_required': action_required,
            'telemetry_age': telemetry_age,
            'command_age': command_age,
            'messages': messages,
            'timestamp': datetime.now().isoformat()
        }
    
    # ============ PRE-FLIGHT CHECKS ============
    
    def pre_flight_checklist(self, telemetry: dict) -> Dict:
        """
        Comprehensive pre-flight safety checklist
        
        Args:
            telemetry: Current telemetry data
        
        Returns:
            dict: Pre-flight check results
        """
        checks = {}
        all_passed = True
        warnings = []
        
        # GPS check
        gps = telemetry.get('gps', {})
        gps_fix = gps.get('fix_type', 0) >= 3
        gps_sats = gps.get('satellites', 0)
        checks['gps_fix'] = gps_fix and gps_sats >= 8
        
        if not checks['gps_fix']:
            all_passed = False
            warnings.append(f'GPS not ready (fix: {gps_fix}, sats: {gps_sats})')
        
        # Battery check
        battery = telemetry.get('battery', {})
        battery_ok = battery.get('voltage', 0) > self.battery_critical_voltage
        checks['battery'] = battery_ok
        
        if not battery_ok:
            all_passed = False
            warnings.append(f'Battery voltage too low: {battery.get("voltage", 0):.2f}V')
        
        # EKF check
        ekf_ok = telemetry.get('ekf_ok', False)
        checks['ekf'] = ekf_ok
        
        if not ekf_ok:
            all_passed = False
            warnings.append('EKF not healthy')
        
        # Geofence check (if center is set)
        if self.geofence_enabled and self.geofence_center:
            checks['geofence'] = True
        else:
            checks['geofence'] = not self.geofence_enabled
            if self.geofence_enabled:
                warnings.append('Geofence enabled but center not set')
        
        # Connection check
        connected = telemetry.get('connected', False)
        checks['connection'] = connected
        
        if not connected:
            all_passed = False
            warnings.append('Not connected to flight controller')
        
        return {
            'passed': all_passed,
            'checks': checks,
            'warnings': warnings,
            'ready_to_arm': all_passed,
            'timestamp': datetime.now().isoformat()
        }
    
    # ============ MONITORING & CALLBACKS ============
    
    def set_warning_callback(self, callback: Callable):
        """Set callback for safety warnings"""
        self.warning_callback = callback
    
    def set_emergency_callback(self, callback: Callable):
        """Set callback for emergency situations"""
        self.emergency_callback = callback
    
    def start_monitoring(self, get_telemetry_func: Callable):
        """
        Start continuous safety monitoring
        
        Args:
            get_telemetry_func: Function to get current telemetry
        """
        if self.monitoring_active:
            print("⚠️  Safety monitoring already active")
            return
        
        self.monitoring_active = True
        self.monitor_thread = threading.Thread(
            target=self._monitoring_loop,
            args=(get_telemetry_func,),
            daemon=True
        )
        self.monitor_thread.start()
        print("🛡️  Safety monitoring started")
    
    def stop_monitoring(self):
        """Stop safety monitoring"""
        self.monitoring_active = False
        if self.monitor_thread:
            self.monitor_thread.join(timeout=2)
        print("🛡️  Safety monitoring stopped")
    
    def _monitoring_loop(self, get_telemetry_func: Callable):
        """Background monitoring loop"""
        
        while self.monitoring_active:
            try:
                telemetry = get_telemetry_func()
                
                if not telemetry or not telemetry.get('connected'):
                    time.sleep(1)
                    continue
                
                # Update telemetry timestamp
                self.update_telemetry_timestamp()
                
                # Check geofence
                if self.geofence_enabled and telemetry.get('armed', False):
                    gps = telemetry.get('gps', {})
                    lat = gps.get('lat', 0)
                    lon = gps.get('lon', 0)
                    alt = gps.get('relative_alt', 0)
                    
                    if lat != 0 and lon != 0:
                        geo_check = self.check_geofence(lat, lon, alt)
                        
                        if not geo_check['within_bounds']:
                            self._handle_violation('geofence', geo_check)
                
                # Check battery
                battery = telemetry.get('battery', {})
                voltage = battery.get('voltage', 0)
                level = battery.get('level', 0)
                armed = telemetry.get('armed', False)
                
                battery_check = self.check_battery_level(voltage, level, armed)
                
                if battery_check['action_required']:
                    self._handle_battery_warning(battery_check)
                
                # Check connection
                conn_check = self.check_connection_health()
                
                if conn_check['action_required']:
                    self._handle_connection_issue(conn_check)
                
                time.sleep(1)  # Check every second
                
            except Exception as e:
                print(f"⚠️  Safety monitoring error: {e}")
                time.sleep(1)
    
    def _handle_violation(self, violation_type: str, details: dict):
        """Handle safety violation"""
        with self.monitor_lock:
            self.violation_count += 1
            
            violation = {
                'type': violation_type,
                'details': details,
                'count': self.violation_count,
                'timestamp': datetime.now().isoformat()
            }
            
            self.safety_violations.append(violation)
            
            # Keep only last 100 violations
            if len(self.safety_violations) > 100:
                self.safety_violations.pop(0)
            
            print(f"⚠️  SAFETY VIOLATION: {violation_type}")
            print(f"   Details: {details}")
            
            # Call warning callback
            if self.warning_callback:
                self.warning_callback(violation)
            
            # If critical, call emergency callback
            if violation_type == 'geofence' and self.emergency_callback:
                self.emergency_callback({
                    'action': 'RTL',
                    'reason': 'Geofence violation',
                    'details': details
                })
    
    def _handle_battery_warning(self, battery_check: dict):
        """Handle battery warning"""
        action = battery_check['action_required']
        
        # Avoid duplicate warnings
        if action in self.battery_warnings_sent:
            return
        
        self.battery_warnings_sent.append(action)
        
        print(f"🔋 BATTERY WARNING: {action}")
        for msg in battery_check['messages']:
            print(f"   {msg}")
        
        if self.warning_callback:
            self.warning_callback({
                'type': 'battery',
                'severity': battery_check['severity'],
                'action': action,
                'details': battery_check
            })
        
        # Emergency action for critical battery
        if action in ['LAND_IMMEDIATE', 'LAND_NOW'] and self.emergency_callback:
            self.emergency_callback({
                'action': 'LAND',
                'reason': 'Critical battery level',
                'details': battery_check
            })
    
    def _handle_connection_issue(self, conn_check: dict):
        """Handle connection issue"""
        print(f"📡 CONNECTION ISSUE: {conn_check['status']}")
        for msg in conn_check['messages']:
            print(f"   {msg}")
        
        if self.warning_callback:
            self.warning_callback({
                'type': 'connection',
                'details': conn_check
            })
        
        # Emergency RTL on connection loss
        if conn_check['status'] == 'connection_lost' and self.emergency_callback:
            self.emergency_callback({
                'action': 'RTL',
                'reason': 'Connection lost',
                'details': conn_check
            })
    
    # ============ STATUS & REPORTING ============
    
    def get_safety_status(self) -> Dict:
        """Get current safety status"""
        return {
            'monitoring_active': self.monitoring_active,
            'geofence_enabled': self.geofence_enabled,
            'geofence_center': self.geofence_center,
            'violation_count': self.violation_count,
            'recent_violations': self.safety_violations[-10:] if self.safety_violations else [],
            'connection_health': self.check_connection_health(),
            'timestamp': datetime.now().isoformat()
        }
    
    def get_violations(self, limit: int = 10) -> List[Dict]:
        """Get recent safety violations"""
        return self.safety_violations[-limit:]
    
    def clear_violations(self):
        """Clear violation history"""
        self.safety_violations.clear()
        self.violation_count = 0
        self.battery_warnings_sent.clear()
        print("🗑️  Violation history cleared")


# Example usage
if __name__ == '__main__':
    """Test the Safety Manager module"""
    
    print("🛡️  Testing Safety Manager Module")
    print("-" * 50)
    
    # Configuration
    config = {
        'geofence_enabled': True,
        'geofence_center': (40.7128, -74.0060),
        'geofence_radius': 100,  # 100m radius
        'geofence_max_altitude': 50,
        'battery_rtl_threshold': 25,
        'battery_land_threshold': 15
    }
    
    # Create safety manager
    safety = SafetyManager(config)
    
    # Test geofence check
    print("\n1️⃣  Testing geofence (within bounds)...")
    result = safety.check_geofence(40.7128, -74.0060, 20)
    print(f"   Within bounds: {result['within_bounds']}")
    
    print("\n2️⃣  Testing geofence (outside bounds)...")
    result = safety.check_geofence(40.7138, -74.0060, 20)  # ~1km north
    print(f"   Within bounds: {result['within_bounds']}")
    if result['violations']:
        print(f"   Violations: {result['violations']}")
    
    print("\n3️⃣  Testing battery check (normal)...")
    result = safety.check_battery_level(12.4, 80, True)
    print(f"   Action required: {result['action_required']}")
    print(f"   Severity: {result['severity']}")
    
    print("\n4️⃣  Testing battery check (low)...")
    result = safety.check_battery_level(11.8, 18, True)
    print(f"   Action required: {result['action_required']}")
    print(f"   Severity: {result['severity']}")
    
    print("\n5️⃣  Testing battery check (critical)...")
    result = safety.check_battery_level(10.2, 8, True)
    print(f"   Action required: {result['action_required']}")
    print(f"   Severity: {result['severity']}")
    
    print("\n6️⃣  Testing pre-flight checklist...")
    test_telemetry = {
        'connected': True,
        'gps': {'fix_type': 3, 'satellites': 12},
        'battery': {'voltage': 12.6},
        'ekf_ok': True
    }
    result = safety.pre_flight_checklist(test_telemetry)
    print(f"   All checks passed: {result['passed']}")
    print(f"   Ready to arm: {result['ready_to_arm']}")
    
    print("\n✅ All safety tests completed!")
