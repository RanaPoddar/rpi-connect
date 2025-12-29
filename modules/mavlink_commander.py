"""
MAVLink Commander Module - Send commands to Pixhawk flight controller
Provides drone control capabilities: ARM/DISARM, mode changes, and flight commands
"""

import time
import threading
import math
import json
from typing import Dict, Optional, Tuple, List
from datetime import datetime

# Try to import dronekit/pymavlink
try:
    from dronekit import connect, VehicleMode, LocationGlobalRelative, Command
    from pymavlink import mavutil
    MAVLINK_AVAILABLE = True
except ImportError:
    MAVLINK_AVAILABLE = False
    print("  DroneKit not available. Install with: pip install dronekit pymavlink")


class MAVLinkCommander:
    """
    Manages MAVLink commands to Pixhawk flight controller.
    Provides high-level interface for drone control operations.
    """
    
    # Available flight modes
    FLIGHT_MODES = [
        'STABILIZE', 'ACRO', 'ALT_HOLD', 'AUTO', 'GUIDED', 
        'LOITER', 'RTL', 'CIRCLE', 'LAND', 'DRIFT', 'SPORT',
        'FLIP', 'AUTOTUNE', 'POSHOLD', 'BRAKE', 'THROW',
        'AVOID_ADSB', 'GUIDED_NOGPS', 'SMART_RTL'
    ]
    
    def __init__(self, vehicle=None, simulation_mode: bool = False):
        """
        Initialize MAVLink Commander
        
        Args:
            vehicle: DroneKit vehicle instance (can be None if not connected)
            simulation_mode: If True, simulate commands without real vehicle
        """
        self.vehicle = vehicle
        self.simulation_mode = simulation_mode
        self.command_lock = threading.Lock()
        
        # Command history
        self.command_history = []
        self.max_history = 50
        
        # Simulated state (for simulation mode)
        self.sim_armed = False
        self.sim_mode = 'STABILIZE'
        self.sim_altitude = 0.0
        self.sim_takeoff_complete = False
        self.sim_lat = 40.7128  # Default NYC coordinates for simulation
        self.sim_lon = -74.0060
        
        print(f"🎮 MAVLink Commander initialized (simulation: {simulation_mode})")
    
    def set_vehicle(self, vehicle):
        """
        Set or update the vehicle instance
        
        Args:
            vehicle: DroneKit vehicle instance
        """
        self.vehicle = vehicle
        if vehicle:
            self.simulation_mode = False
            print("🎮 Vehicle connected to commander")
    
    def _log_command(self, command: str, result: dict):
        """Log command execution"""
        log_entry = {
            'timestamp': datetime.now().isoformat(),
            'command': command,
            'result': result
        }
        self.command_history.append(log_entry)
        
        # Maintain max history size
        if len(self.command_history) > self.max_history:
            self.command_history.pop(0)
    
    def _check_vehicle_ready(self) -> Tuple[bool, str]:
        """
        Check if vehicle is ready for commands
        
        Returns:
            tuple: (ready: bool, message: str)
        """
        if self.simulation_mode:
            return True, "Simulation mode"
        
        if not MAVLINK_AVAILABLE:
            return False, "MAVLink libraries not available"
        
        if not self.vehicle:
            return False, "No vehicle connected"
        
        return True, "Vehicle ready"
    
    # ============ ARM/DISARM COMMANDS ============
    
    def arm(self, force: bool = False) -> Dict:
        """
        ARM the vehicle
        
        Args:
            force: If True, skip pre-arm checks (use with caution!)
        
        Returns:
            dict: Command result with success status and message
        """
        with self.command_lock:
            result = self._arm_internal(force)
            self._log_command('ARM', result)
            return result
    
    def _arm_internal(self, force: bool = False) -> Dict:
        """Internal ARM implementation"""
        
        # Check vehicle readiness
        ready, msg = self._check_vehicle_ready()
        if not ready:
            return {
                'success': False,
                'message': f'Cannot arm: {msg}',
                'timestamp': datetime.now().isoformat()
            }
        
        # Simulation mode
        if self.simulation_mode:
            self.sim_armed = True
            print("🔴 [SIM] Vehicle ARMED")
            return {
                'success': True,
                'message': 'Vehicle armed (simulated)',
                'armed': True,
                'timestamp': datetime.now().isoformat()
            }
        
        # Real vehicle
        try:
            # Check if already armed
            if self.vehicle.armed:
                print("⚠️  Vehicle already armed")
                return {
                    'success': True,
                    'message': 'Vehicle already armed',
                    'armed': True,
                    'timestamp': datetime.now().isoformat()
                }
            
            # Pre-arm checks (unless forced)
            if not force:
                checks = self.get_prearm_checks()
                if not checks['armable']:
                    return {
                        'success': False,
                        'message': 'Pre-arm checks failed',
                        'checks': checks,
                        'timestamp': datetime.now().isoformat()
                    }
            
            print(" Arming vehicle...")
            self.vehicle.armed = True
            
            # Wait for arming confirmation (max 10 seconds)
            timeout = 10
            start_time = time.time()
            while not self.vehicle.armed:
                if time.time() - start_time > timeout:
                    return {
                        'success': False,
                        'message': 'Arming timeout - vehicle did not arm',
                        'armed': False,
                        'timestamp': datetime.now().isoformat()
                    }
                time.sleep(0.5)
            
            print(" Vehicle ARMED successfully")
            return {
                'success': True,
                'message': 'Vehicle armed successfully',
                'armed': True,
                'timestamp': datetime.now().isoformat()
            }
            
        except Exception as e:
            print(f"Arming failed: {e}")
            return {
                'success': False,
                'message': f'Arming failed: {str(e)}',
                'armed': False,
                'timestamp': datetime.now().isoformat()
            }
    
    def disarm(self, force: bool = False) -> Dict:
        """
        DISARM the vehicle
        
        Args:
            force: If True, force disarm even in flight (DANGEROUS!)
        
        Returns:
            dict: Command result with success status and message
        """
        with self.command_lock:
            result = self._disarm_internal(force)
            self._log_command('DISARM', result)
            return result
    
    def _disarm_internal(self, force: bool = False) -> Dict:
        """Internal DISARM implementation"""
        
        # Check vehicle readiness
        ready, msg = self._check_vehicle_ready()
        if not ready:
            return {
                'success': False,
                'message': f'Cannot disarm: {msg}',
                'timestamp': datetime.now().isoformat()
            }
        
        # Simulation mode
        if self.simulation_mode:
            self.sim_armed = False
            print(" [SIM] Vehicle DISARMED")
            return {
                'success': True,
                'message': 'Vehicle disarmed (simulated)',
                'armed': False,
                'timestamp': datetime.now().isoformat()
            }
        
        # Real vehicle
        try:
            # Check if already disarmed
            if not self.vehicle.armed:
                print("  Vehicle already disarmed")
                return {
                    'success': True,
                    'message': 'Vehicle already disarmed',
                    'armed': False,
                    'timestamp': datetime.now().isoformat()
                }
            
            # Safety check - don't disarm in flight unless forced
            if not force:
                altitude = self.vehicle.location.global_relative_frame.alt
                if altitude and altitude > 1.0:  # More than 1 meter above ground
                    return {
                        'success': False,
                        'message': f'Cannot disarm in flight (altitude: {altitude:.1f}m). Use force=True to override.',
                        'armed': True,
                        'timestamp': datetime.now().isoformat()
                    }
            
            print("Disarming vehicle...")
            self.vehicle.armed = False
            
            # Wait for disarming confirmation (max 5 seconds)
            timeout = 5
            start_time = time.time()
            while self.vehicle.armed:
                if time.time() - start_time > timeout:
                    return {
                        'success': False,
                        'message': 'Disarming timeout - vehicle did not disarm',
                        'armed': True,
                        'timestamp': datetime.now().isoformat()
                    }
                time.sleep(0.5)
            
            print(" Vehicle DISARMED successfully")
            return {
                'success': True,
                'message': 'Vehicle disarmed successfully',
                'armed': False,
                'timestamp': datetime.now().isoformat()
            }
            
        except Exception as e:
            print(f" Disarming failed: {e}")
            return {
                'success': False,
                'message': f'Disarming failed: {str(e)}',
                'armed': True,
                'timestamp': datetime.now().isoformat()
            }
    
    # ============ FLIGHT MODE COMMANDS ============
    
    def set_mode(self, mode: str) -> Dict:
        """
        Change flight mode
        
        Args:
            mode: Flight mode name (e.g., 'GUIDED', 'AUTO', 'RTL', 'LAND')
        
        Returns:
            dict: Command result with success status and message
        """
        with self.command_lock:
            result = self._set_mode_internal(mode)
            self._log_command(f'SET_MODE_{mode}', result)
            return result
    
    def _set_mode_internal(self, mode: str) -> Dict:
        """Internal mode change implementation"""
        
        # Validate mode
        mode = mode.upper()
        if mode not in self.FLIGHT_MODES:
            return {
                'success': False,
                'message': f'Invalid flight mode: {mode}. Valid modes: {", ".join(self.FLIGHT_MODES[:10])}...',
                'timestamp': datetime.now().isoformat()
            }
        
        # Check vehicle readiness
        ready, msg = self._check_vehicle_ready()
        if not ready:
            return {
                'success': False,
                'message': f'Cannot change mode: {msg}',
                'timestamp': datetime.now().isoformat()
            }
        
        # Simulation mode
        if self.simulation_mode:
            self.sim_mode = mode
            print(f" [SIM] Mode changed to {mode}")
            return {
                'success': True,
                'message': f'Mode changed to {mode} (simulated)',
                'mode': mode,
                'timestamp': datetime.now().isoformat()
            }
        
        # Real vehicle
        try:
            current_mode = self.vehicle.mode.name
            
            # Check if already in this mode
            if current_mode == mode:
                print(f"⚠️  Already in {mode} mode")
                return {
                    'success': True,
                    'message': f'Already in {mode} mode',
                    'mode': mode,
                    'timestamp': datetime.now().isoformat()
                }
            
            print(f" Changing mode from {current_mode} to {mode}...")
            self.vehicle.mode = VehicleMode(mode)
            
            # Wait for mode change confirmation (max 5 seconds)
            timeout = 5
            start_time = time.time()
            while self.vehicle.mode.name != mode:
                if time.time() - start_time > timeout:
                    return {
                        'success': False,
                        'message': f'Mode change timeout - still in {self.vehicle.mode.name}',
                        'mode': self.vehicle.mode.name,
                        'timestamp': datetime.now().isoformat()
                    }
                time.sleep(0.3)
            
            print(f" Mode changed to {mode} successfully")
            return {
                'success': True,
                'message': f'Mode changed to {mode} successfully',
                'mode': mode,
                'timestamp': datetime.now().isoformat()
            }
            
        except Exception as e:
            print(f" Mode change failed: {e}")
            return {
                'success': False,
                'message': f'Mode change failed: {str(e)}',
                'mode': self.vehicle.mode.name if self.vehicle else 'UNKNOWN',
                'timestamp': datetime.now().isoformat()
            }
    
    # ============ PRE-ARM CHECKS ============
    
    def get_prearm_checks(self) -> Dict:
        """
        Get pre-arm check status
        
        Returns:
            dict: Pre-arm check results
        """
        if self.simulation_mode:
            return {
                'armable': True,
                'simulation': True,
                'checks': {
                    'gps': True,
                    'ekf': True,
                    'battery': True,
                    'mode': True
                },
                'message': 'Simulation mode - all checks passed',
                'timestamp': datetime.now().isoformat()
            }
        
        if not self.vehicle:
            return {
                'armable': False,
                'checks': {},
                'message': 'No vehicle connected',
                'timestamp': datetime.now().isoformat()
            }
        
        try:
            checks = {
                'armable': self.vehicle.is_armable,
                'gps_fix': self.vehicle.gps_0.fix_type >= 2 if self.vehicle.gps_0 else False,
                'gps_satellites': self.vehicle.gps_0.satellites_visible if self.vehicle.gps_0 else 0,
                'ekf_ok': self.vehicle.ekf_ok,
                'battery_voltage': self.vehicle.battery.voltage if self.vehicle.battery else 0,
                'mode': self.vehicle.mode.name,
                'system_status': self.vehicle.system_status.state
            }
            
            # Determine if safe to arm
            safe_to_arm = (
                checks['armable'] and
                checks['gps_fix'] and
                checks['gps_satellites'] >= 6 and
                checks['ekf_ok'] and
                checks['battery_voltage'] > 10.5  # Minimum voltage for 3S LiPo
            )
            
            return {
                'armable': safe_to_arm,
                'checks': checks,
                'message': 'Ready to arm' if safe_to_arm else 'Not ready to arm - check conditions',
                'timestamp': datetime.now().isoformat()
            }
            
        except Exception as e:
            return {
                'armable': False,
                'checks': {},
                'message': f'Error checking pre-arm status: {str(e)}',
                'timestamp': datetime.now().isoformat()
            }
    
    # ============ STATUS & INFO ============
    
    def get_status(self) -> Dict:
        """
        Get current vehicle status
        
        Returns:
            dict: Vehicle status information
        """
        if self.simulation_mode:
            return {
                'connected': True,
                'simulation': True,
                'armed': self.sim_armed,
                'mode': self.sim_mode,
                'armable': True,
                'timestamp': datetime.now().isoformat()
            }
        
        if not self.vehicle:
            return {
                'connected': False,
                'simulation': False,
                'armed': False,
                'mode': 'UNKNOWN',
                'armable': False,
                'message': 'No vehicle connected',
                'timestamp': datetime.now().isoformat()
            }
        
        try:
            return {
                'connected': True,
                'simulation': False,
                'armed': self.vehicle.armed,
                'mode': self.vehicle.mode.name,
                'armable': self.vehicle.is_armable,
                'system_status': self.vehicle.system_status.state,
                'ekf_ok': self.vehicle.ekf_ok,
                'gps_satellites': self.vehicle.gps_0.satellites_visible if self.vehicle.gps_0 else 0,
                'battery_voltage': self.vehicle.battery.voltage if self.vehicle.battery else 0,
                'timestamp': datetime.now().isoformat()
            }
        except Exception as e:
            return {
                'connected': False,
                'error': str(e),
                'timestamp': datetime.now().isoformat()
            }
    
    def get_command_history(self, limit: int = 10) -> list:
        """
        Get recent command history
        
        Args:
            limit: Maximum number of commands to return
        
        Returns:
            list: Recent command history
        """
        return self.command_history[-limit:]
    
    def clear_command_history(self):
        """Clear command history"""
        self.command_history.clear()
        print("🗑️  Command history cleared")
    
    # ============ CONVENIENCE METHODS ============
    
    def emergency_stop(self) -> Dict:
        """
        Emergency stop - change to LAND mode if armed
        
        Returns:
            dict: Command result
        """
        print(" EMERGENCY STOP INITIATED")
        
        status = self.get_status()
        if status['armed']:
            return self.set_mode('LAND')
        else:
            return {
                'success': True,
                'message': 'Vehicle already disarmed',
                'timestamp': datetime.now().isoformat()
            }
    
    def return_to_launch(self) -> Dict:
        """
        Return to launch point (RTL mode)
        
        Returns:
            dict: Command result
        """
        return self.set_mode('RTL')
    
    def hold_position(self) -> Dict:
        """
        Hold current position (LOITER mode)
        
        Returns:
            dict: Command result
        """
        return self.set_mode('LOITER')
    
    # ============ TAKEOFF & LANDING COMMANDS ============
    
    def takeoff(self, altitude: float, timeout: int = 60) -> Dict:
        """
        Automated takeoff to specified altitude
        
        Args:
            altitude: Target altitude in meters (relative to home)
            timeout: Maximum time to wait for takeoff completion (seconds)
        
        Returns:
            dict: Command result with success status and message
        """
        with self.command_lock:
            result = self._takeoff_internal(altitude, timeout)
            self._log_command(f'TAKEOFF_{altitude}m', result)
            return result
    
    def _takeoff_internal(self, altitude: float, timeout: int) -> Dict:
        """Internal takeoff implementation"""
        
        # Validate altitude (for safety check )
        if altitude < 2.0:
            return {
                'success': False,
                'message': 'Altitude must be at least 2 meters for safety',
                'timestamp': datetime.now().isoformat()
            }
        
        if altitude > 100.0:
            return {
                'success': False,
                'message': 'Altitude exceeds maximum allowed (100m)',
                'timestamp': datetime.now().isoformat()
            }
        
        # Check vehicle readiness
        ready, msg = self._check_vehicle_ready()
        if not ready:
            return {
                'success': False,
                'message': f'Cannot takeoff: {msg}',
                'timestamp': datetime.now().isoformat()
            }
        
        # Simulation mode
        if self.simulation_mode:
            if not self.sim_armed:
                return {
                    'success': False,
                    'message': 'Cannot takeoff: Vehicle not armed',
                    'timestamp': datetime.now().isoformat()
                }
            
            print(f" [SIM] Taking off to {altitude}m...")
            time.sleep(2)  # Simulate takeoff time
            self.sim_altitude = altitude
            self.sim_takeoff_complete = True
            print(f" [SIM] Takeoff complete - altitude: {altitude}m")
            
            return {
                'success': True,
                'message': f'Takeoff complete to {altitude}m (simulated)',
                'altitude': altitude,
                'timestamp': datetime.now().isoformat()
            }
        
        # Real vehicle (when we got pi connected with pixhawk (when isuue will be fixed))
        try:
            # Check if armed
            if not self.vehicle.armed:
                return {
                    'success': False,
                    'message': 'Cannot takeoff: Vehicle not armed. Arm first.',
                    'timestamp': datetime.now().isoformat()
                }
            
            # Check pre-flight conditions
            prearm = self.get_prearm_checks()
            if not prearm['armable']:
                return {
                    'success': False,
                    'message': 'Pre-flight checks failed',
                    'checks': prearm,
                    'timestamp': datetime.now().isoformat()
                }
            
            # Set mode to GUIDED (required for takeoff command)
            current_mode = self.vehicle.mode.name
            if current_mode != 'GUIDED':
                print(f"Switching to GUIDED mode for takeoff...")
                mode_result = self.set_mode('GUIDED')
                if not mode_result['success']:
                    return {
                        'success': False,
                        'message': f'Failed to switch to GUIDED mode: {mode_result["message"]}',
                        'timestamp': datetime.now().isoformat()
                    }
            
            print(f"Taking off to {altitude} meters...")
            self.vehicle.simple_takeoff(altitude)
            
            # Wait for takeoff to reach target altitude
            start_time = time.time()
            while time.time() - start_time < timeout:
                current_alt = self.vehicle.location.global_relative_frame.alt
                
                if current_alt is None:
                    time.sleep(0.5)
                    continue
                
                print(f"   Altitude: {current_alt:.1f}m / {altitude}m", end='\r')
                
                # Check if reached target altitude (within 95%)
                if current_alt >= altitude * 0.95:
                    print(f"\n Takeoff complete - reached {current_alt:.1f}m")
                    return {
                        'success': True,
                        'message': f'Takeoff complete - reached {current_alt:.1f}m',
                        'target_altitude': altitude,
                        'actual_altitude': current_alt,
                        'timestamp': datetime.now().isoformat()
                    }
                
                time.sleep(0.5)
            
            # Timeout reached
            current_alt = self.vehicle.location.global_relative_frame.alt or 0
            print(f"\n  Takeoff timeout - reached {current_alt:.1f}m of {altitude}m")
            return {
                'success': False,
                'message': f'Takeoff timeout - only reached {current_alt:.1f}m of {altitude}m target',
                'target_altitude': altitude,
                'actual_altitude': current_alt,
                'timestamp': datetime.now().isoformat()
            }
            
        except Exception as e:
            print(f" Takeoff failed: {e}")
            return {
                'success': False,
                'message': f'Takeoff failed: {str(e)}',
                'timestamp': datetime.now().isoformat()
            }
    
    def land(self, timeout: int = 60) -> Dict:
        """
        Initiate landing sequence
        
        Args:
            timeout: Maximum time to wait for landing completion (seconds)
        
        Returns:
            dict: Command result with success status and message
        """
        with self.command_lock:
            result = self._land_internal(timeout)
            self._log_command('LAND', result)
            return result
    
    def _land_internal(self, timeout: int) -> Dict:
        """Internal landing implementation"""
        
        # Check vehicle readiness
        ready, msg = self._check_vehicle_ready()
        if not ready:
            return {
                'success': False,
                'message': f'Cannot land: {msg}',
                'timestamp': datetime.now().isoformat()
            }
        
        # Simulation mode
        if self.simulation_mode:
            if not self.sim_armed:
                return {
                    'success': False,
                    'message': 'Vehicle already disarmed/landed',
                    'timestamp': datetime.now().isoformat()
                }
            
            print(f"🛬 [SIM] Landing...")
            time.sleep(2)  # Simulate landing time
            self.sim_altitude = 0.0
            self.sim_armed = False
            self.sim_takeoff_complete = False
            print(f"✅ [SIM] Landing complete")
            
            return {
                'success': True,
                'message': 'Landing complete (simulated)',
                'timestamp': datetime.now().isoformat()
            }
        
        # Real vehicle
        try:
            # Check if already on ground
            if not self.vehicle.armed:
                return {
                    'success': True,
                    'message': 'Vehicle already disarmed/landed',
                    'timestamp': datetime.now().isoformat()
                }
            
            # Set mode to LAND
            print(f"🛬 Initiating landing sequence...")
            mode_result = self.set_mode('LAND')
            if not mode_result['success']:
                return {
                    'success': False,
                    'message': f'Failed to switch to LAND mode: {mode_result["message"]}',
                    'timestamp': datetime.now().isoformat()
                }
            
            # Wait for landing to complete
            start_time = time.time()
            while time.time() - start_time < timeout:
                current_alt = self.vehicle.location.global_relative_frame.alt
                armed = self.vehicle.armed
                
                if current_alt is not None:
                    print(f"   Altitude: {current_alt:.1f}m, Armed: {armed}", end='\r')
                
                # Check if landed (disarmed automatically)
                if not armed:
                    print(f"\n✅ Landing complete - vehicle disarmed")
                    return {
                        'success': True,
                        'message': 'Landing complete - vehicle disarmed',
                        'timestamp': datetime.now().isoformat()
                    }
                
                # Check if very close to ground
                if current_alt is not None and current_alt < 0.5:
                    print(f"\n✅ Landing complete - on ground")
                    return {
                        'success': True,
                        'message': 'Landing complete - on ground',
                        'timestamp': datetime.now().isoformat()
                    }
                
                time.sleep(0.5)
            
            # Timeout reached but may still be landing
            current_alt = self.vehicle.location.global_relative_frame.alt or 0
            armed = self.vehicle.armed
            
            if not armed or current_alt < 0.5:
                print(f"\n✅ Landing complete")
                return {
                    'success': True,
                    'message': 'Landing complete',
                    'timestamp': datetime.now().isoformat()
                }
            
            print(f"\n⚠️  Landing timeout - still at {current_alt:.1f}m")
            return {
                'success': False,
                'message': f'Landing timeout - still at {current_alt:.1f}m',
                'altitude': current_alt,
                'armed': armed,
                'timestamp': datetime.now().isoformat()
            }
            
        except Exception as e:
            print(f"❌ Landing failed: {e}")
            return {
                'success': False,
                'message': f'Landing failed: {str(e)}',
                'timestamp': datetime.now().isoformat()
            }
    
    def get_altitude(self) -> Dict:
        """
        Get current altitude information
        
        Returns:
            dict: Altitude data (relative and absolute)
        """
        if self.simulation_mode:
            return {
                'relative_altitude': self.sim_altitude,
                'absolute_altitude': self.sim_altitude + 100.0,  # Fake home altitude
                'simulation': True,
                'timestamp': datetime.now().isoformat()
            }
        
        if not self.vehicle:
            return {
                'error': 'No vehicle connected',
                'timestamp': datetime.now().isoformat()
            }
        
        try:
            loc_rel = self.vehicle.location.global_relative_frame
            loc_abs = self.vehicle.location.global_frame
            
            return {
                'relative_altitude': loc_rel.alt if loc_rel else 0.0,
                'absolute_altitude': loc_abs.alt if loc_abs else 0.0,
                'timestamp': datetime.now().isoformat()
            }
        except Exception as e:
            return {
                'error': str(e),
                'timestamp': datetime.now().isoformat()
            }
    
    # ============ WAYPOINT NAVIGATION ============
    
    @staticmethod
    def get_distance_meters(lat1: float, lon1: float, lat2: float, lon2: float) -> float:
        """
        Calculate distance between two GPS coordinates using Haversine formula
        
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
    
    @staticmethod
    def get_bearing(lat1: float, lon1: float, lat2: float, lon2: float) -> float:
        """
        Calculate bearing from point 1 to point 2
        
        Args:
            lat1, lon1: Start coordinate
            lat2, lon2: End coordinate
        
        Returns:
            float: Bearing in degrees (0-360)
        """
        phi1 = math.radians(lat1)
        phi2 = math.radians(lat2)
        delta_lambda = math.radians(lon2 - lon1)
        
        y = math.sin(delta_lambda) * math.cos(phi2)
        x = math.cos(phi1) * math.sin(phi2) - \
            math.sin(phi1) * math.cos(phi2) * math.cos(delta_lambda)
        
        bearing = math.degrees(math.atan2(y, x))
        return (bearing + 360) % 360
    
    def goto_location(self, lat: float, lon: float, altitude: float = None, 
                      groundspeed: float = None, timeout: int = 120) -> Dict:
        """
        Navigate to a specific GPS location
        
        Args:
            lat: Target latitude
            lon: Target longitude
            altitude: Target altitude in meters (None = maintain current)
            groundspeed: Target groundspeed in m/s (None = use default)
            timeout: Maximum time to reach waypoint (seconds)
        
        Returns:
            dict: Command result with success status and message
        """
        with self.command_lock:
            result = self._goto_location_internal(lat, lon, altitude, groundspeed, timeout)
            self._log_command(f'GOTO_({lat:.6f},{lon:.6f})', result)
            return result
    
    def _goto_location_internal(self, lat: float, lon: float, altitude: float, 
                                groundspeed: float, timeout: int) -> Dict:
        """Internal goto location implementation"""
        
        # Validate coordinates
        if not (-90 <= lat <= 90):
            return {
                'success': False,
                'message': f'Invalid latitude: {lat} (must be -90 to 90)',
                'timestamp': datetime.now().isoformat()
            }
        
        if not (-180 <= lon <= 180):
            return {
                'success': False,
                'message': f'Invalid longitude: {lon} (must be -180 to 180)',
                'timestamp': datetime.now().isoformat()
            }
        
        # Check vehicle readiness
        ready, msg = self._check_vehicle_ready()
        if not ready:
            return {
                'success': False,
                'message': f'Cannot navigate: {msg}',
                'timestamp': datetime.now().isoformat()
            }
        
        # Simulation mode
        if self.simulation_mode:
            if not self.sim_armed:
                return {
                    'success': False,
                    'message': 'Cannot navigate: Vehicle not armed',
                    'timestamp': datetime.now().isoformat()
                }
            
            # Calculate distance
            distance = self.get_distance_meters(self.sim_lat, self.sim_lon, lat, lon)
            bearing = self.get_bearing(self.sim_lat, self.sim_lon, lat, lon)
            
            print(f"  [SIM] Navigating to ({lat:.6f}, {lon:.6f})")
            print(f"    Distance: {distance:.1f}m, Bearing: {bearing:.0f}°")
            time.sleep(2)  # Simulate travel time
            
            self.sim_lat = lat
            self.sim_lon = lon
            if altitude is not None:
                self.sim_altitude = altitude
            
            print(f"✅ [SIM] Arrived at destination")
            
            return {
                'success': True,
                'message': f'Arrived at destination (simulated)',
                'distance_traveled': distance,
                'timestamp': datetime.now().isoformat()
            }
        
        # Real vehicle
        try:
            # Check if armed
            if not self.vehicle.armed:
                return {
                    'success': False,
                    'message': 'Cannot navigate: Vehicle not armed',
                    'timestamp': datetime.now().isoformat()
                }
            
            # Ensure in GUIDED mode
            if self.vehicle.mode.name != 'GUIDED':
                print(f" Switching to GUIDED mode...")
                mode_result = self.set_mode('GUIDED')
                if not mode_result['success']:
                    return {
                        'success': False,
                        'message': f'Failed to switch to GUIDED mode: {mode_result["message"]}',
                        'timestamp': datetime.now().isoformat()
                    }
            
            # Get current location
            current_loc = self.vehicle.location.global_relative_frame
            if not current_loc or current_loc.lat is None:
                return {
                    'success': False,
                    'message': 'Cannot get current location - GPS not ready',
                    'timestamp': datetime.now().isoformat()
                }
            
            # Calculate initial distance
            start_distance = self.get_distance_meters(
                current_loc.lat, current_loc.lon, lat, lon
            )
            bearing = self.get_bearing(current_loc.lat, current_loc.lon, lat, lon)
            
            print(f"   Navigating to ({lat:.6f}, {lon:.6f})")
            print(f"    Distance: {start_distance:.1f}m, Bearing: {bearing:.0f}°")
            
            # Set groundspeed if specified
            if groundspeed is not None:
                self.vehicle.groundspeed = groundspeed
                print(f"    Groundspeed set to {groundspeed} m/s")
            
            # Use altitude if specified, otherwise maintain current
            target_alt = altitude if altitude is not None else current_loc.alt
            
            # Create target location
            target_location = LocationGlobalRelative(lat, lon, target_alt)
            
            # Send goto command
            self.vehicle.simple_goto(target_location)
            
            # Monitor progress
            start_time = time.time()
            arrival_threshold = 2.0  # Within 2 meters = arrived
            
            while time.time() - start_time < timeout:
                current_loc = self.vehicle.location.global_relative_frame
                
                if not current_loc or current_loc.lat is None:
                    time.sleep(0.5)
                    continue
                
                # Calculate remaining distance
                remaining_distance = self.get_distance_meters(
                    current_loc.lat, current_loc.lon, lat, lon
                )
                
                print(f"    Distance remaining: {remaining_distance:.1f}m", end='\r')
                
                # Check if arrived
                if remaining_distance <= arrival_threshold:
                    print(f"\n✅ Arrived at destination ({remaining_distance:.1f}m from target)")
                    return {
                        'success': True,
                        'message': f'Arrived at destination ({remaining_distance:.1f}m from target)',
                        'distance_traveled': start_distance,
                        'final_distance': remaining_distance,
                        'time_elapsed': time.time() - start_time,
                        'timestamp': datetime.now().isoformat()
                    }
                
                time.sleep(0.5)
            
            # Timeout reached
            current_loc = self.vehicle.location.global_relative_frame
            remaining_distance = self.get_distance_meters(
                current_loc.lat, current_loc.lon, lat, lon
            ) if current_loc else 0
            
            print(f"\n⚠️  Navigation timeout - {remaining_distance:.1f}m from target")
            return {
                'success': False,
                'message': f'Navigation timeout - still {remaining_distance:.1f}m from target',
                'distance_traveled': start_distance - remaining_distance,
                'remaining_distance': remaining_distance,
                'timestamp': datetime.now().isoformat()
            }
            
        except Exception as e:
            print(f"❌ Navigation failed: {e}")
            return {
                'success': False,
                'message': f'Navigation failed: {str(e)}',
                'timestamp': datetime.now().isoformat()
            }
    
    def get_current_location(self) -> Dict:
        """
        Get current GPS location
        
        Returns:
            dict: Current location data
        """
        if self.simulation_mode:
            return {
                'latitude': self.sim_lat,
                'longitude': self.sim_lon,
                'altitude': self.sim_altitude,
                'simulation': True,
                'timestamp': datetime.now().isoformat()
            }
        
        if not self.vehicle:
            return {
                'error': 'No vehicle connected',
                'timestamp': datetime.now().isoformat()
            }
        
        try:
            loc = self.vehicle.location.global_relative_frame
            if not loc or loc.lat is None:
                return {
                    'error': 'GPS location not available',
                    'timestamp': datetime.now().isoformat()
                }
            
            return {
                'latitude': loc.lat,
                'longitude': loc.lon,
                'altitude': loc.alt,
                'timestamp': datetime.now().isoformat()
            }
        except Exception as e:
            return {
                'error': str(e),
                'timestamp': datetime.now().isoformat()
            }
    
    def calculate_distance_to_waypoint(self, lat: float, lon: float) -> Dict:
        """
        Calculate distance and bearing to a waypoint
        
        Args:
            lat: Target latitude
            lon: Target longitude
        
        Returns:
            dict: Distance and bearing information
        """
        current = self.get_current_location()
        
        if 'error' in current:
            return {
                'error': current['error'],
                'timestamp': datetime.now().isoformat()
            }
        
        distance = self.get_distance_meters(
            current['latitude'], current['longitude'], lat, lon
        )
        bearing = self.get_bearing(
            current['latitude'], current['longitude'], lat, lon
        )
        
        return {
            'distance_meters': distance,
            'bearing_degrees': bearing,
            'from': {
                'lat': current['latitude'],
                'lon': current['longitude']
            },
            'to': {
                'lat': lat,
                'lon': lon
            },
            'timestamp': datetime.now().isoformat()
        }
    
    # ============ MISSION MANAGEMENT ============
    
    def upload_mission(self, waypoints: List[Dict], takeoff_alt: float = None) -> Dict:
        """
        Upload a mission with multiple waypoints
        
        Args:
            waypoints: List of waypoint dictionaries with keys:
                      - lat: latitude
                      - lon: longitude
                      - alt: altitude (optional, uses first waypoint alt if not specified)
                      - action: 'waypoint' or 'land' (optional, default 'waypoint')
            takeoff_alt: Takeoff altitude in meters (None = don't add takeoff command)
        
        Returns:
            dict: Upload result with success status and message
        
        Example waypoint format:
            [
                {'lat': 40.7128, 'lon': -74.0060, 'alt': 10},
                {'lat': 40.7129, 'lon': -74.0061, 'alt': 10},
                {'lat': 40.7130, 'lon': -74.0062, 'alt': 10, 'action': 'land'}
            ]
        """
        with self.command_lock:
            result = self._upload_mission_internal(waypoints, takeoff_alt)
            self._log_command(f'UPLOAD_MISSION_{len(waypoints)}_WP', result)
            return result
    
    def _upload_mission_internal(self, waypoints: List[Dict], takeoff_alt: float) -> Dict:
        """Internal mission upload implementation"""
        
        # Validate waypoints
        if not waypoints or len(waypoints) == 0:
            return {
                'success': False,
                'message': 'Waypoint list is empty',
                'timestamp': datetime.now().isoformat()
            }
        
        # Validate each waypoint
        for i, wp in enumerate(waypoints):
            if 'lat' not in wp or 'lon' not in wp:
                return {
                    'success': False,
                    'message': f'Waypoint {i} missing lat/lon',
                    'timestamp': datetime.now().isoformat()
                }
            
            if not (-90 <= wp['lat'] <= 90):
                return {
                    'success': False,
                    'message': f'Waypoint {i} has invalid latitude: {wp["lat"]}',
                    'timestamp': datetime.now().isoformat()
                }
            
            if not (-180 <= wp['lon'] <= 180):
                return {
                    'success': False,
                    'message': f'Waypoint {i} has invalid longitude: {wp["lon"]}',
                    'timestamp': datetime.now().isoformat()
                }
        
        # Check vehicle readiness
        ready, msg = self._check_vehicle_ready()
        if not ready:
            return {
                'success': False,
                'message': f'Cannot upload mission: {msg}',
                'timestamp': datetime.now().isoformat()
            }
        
        # Simulation mode
        if self.simulation_mode:
            print(f"📋 [SIM] Uploading mission with {len(waypoints)} waypoints...")
            time.sleep(1)
            print(f"✅ [SIM] Mission uploaded successfully")
            
            return {
                'success': True,
                'message': f'Mission uploaded with {len(waypoints)} waypoints (simulated)',
                'waypoint_count': len(waypoints),
                'timestamp': datetime.now().isoformat()
            }
        
        # Real vehicle
        try:
            print(f"📋 Uploading mission with {len(waypoints)} waypoints...")
            
            # Get commands object
            cmds = self.vehicle.commands
            
            # Clear existing mission
            cmds.clear()
            
            # Add home location (required as first waypoint)
            # Use current location as home if available
            home_location = self.vehicle.location.global_frame
            if home_location and home_location.lat:
                home_cmd = Command(
                    0, 0, 0,
                    mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
                    mavutil.mavlink.MAV_CMD_NAV_WAYPOINT,
                    0, 1,  # current=0, autocontinue=1
                    0, 0, 0, 0,  # param1-4
                    home_location.lat, home_location.lon, 0  # x, y, z
                )
                cmds.add(home_cmd)
            
            # Add takeoff command if specified
            if takeoff_alt is not None:
                takeoff_cmd = Command(
                    0, 0, 0,
                    mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
                    mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
                    0, 0,
                    0, 0, 0, 0,  # param1-4 (pitch, empty, empty, yaw)
                    0, 0, takeoff_alt  # lat, lon, alt
                )
                cmds.add(takeoff_cmd)
            
            # Add waypoints
            for i, wp in enumerate(waypoints):
                altitude = wp.get('alt', waypoints[0].get('alt', 10))
                action = wp.get('action', 'waypoint')
                
                if action == 'land':
                    # Add land command
                    cmd = Command(
                        0, 0, 0,
                        mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
                        mavutil.mavlink.MAV_CMD_NAV_LAND,
                        0, 0,
                        0, 0, 0, 0,  # param1-4
                        wp['lat'], wp['lon'], 0
                    )
                else:
                    # Add waypoint command
                    cmd = Command(
                        0, 0, 0,
                        mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
                        mavutil.mavlink.MAV_CMD_NAV_WAYPOINT,
                        0, 0,
                        0, 0, 0, 0,  # param1-4 (hold, accept radius, pass radius, yaw)
                        wp['lat'], wp['lon'], altitude
                    )
                
                cmds.add(cmd)
                print(f"   Added waypoint {i+1}: ({wp['lat']:.6f}, {wp['lon']:.6f}, {altitude}m)")
            
            # Upload commands to vehicle
            cmds.upload()
            print(f"✅ Mission uploaded successfully - {len(waypoints)} waypoints")
            
            return {
                'success': True,
                'message': f'Mission uploaded with {len(waypoints)} waypoints',
                'waypoint_count': len(waypoints),
                'timestamp': datetime.now().isoformat()
            }
            
        except Exception as e:
            print(f"❌ Mission upload failed: {e}")
            return {
                'success': False,
                'message': f'Mission upload failed: {str(e)}',
                'timestamp': datetime.now().isoformat()
            }
    
    def clear_mission(self) -> Dict:
        """
        Clear the current mission
        
        Returns:
            dict: Clear result
        """
        with self.command_lock:
            result = self._clear_mission_internal()
            self._log_command('CLEAR_MISSION', result)
            return result
    
    def _clear_mission_internal(self) -> Dict:
        """Internal mission clear implementation"""
        
        ready, msg = self._check_vehicle_ready()
        if not ready:
            return {
                'success': False,
                'message': f'Cannot clear mission: {msg}',
                'timestamp': datetime.now().isoformat()
            }
        
        if self.simulation_mode:
            print("🗑️  [SIM] Mission cleared")
            return {
                'success': True,
                'message': 'Mission cleared (simulated)',
                'timestamp': datetime.now().isoformat()
            }
        
        try:
            print("🗑️  Clearing mission...")
            self.vehicle.commands.clear()
            self.vehicle.commands.upload()
            print("✅ Mission cleared successfully")
            
            return {
                'success': True,
                'message': 'Mission cleared successfully',
                'timestamp': datetime.now().isoformat()
            }
            
        except Exception as e:
            print(f"❌ Mission clear failed: {e}")
            return {
                'success': False,
                'message': f'Mission clear failed: {str(e)}',
                'timestamp': datetime.now().isoformat()
            }
    
    def start_mission(self) -> Dict:
        """
        Start the uploaded mission (switch to AUTO mode)
        
        Returns:
            dict: Start result
        """
        with self.command_lock:
            result = self._start_mission_internal()
            self._log_command('START_MISSION', result)
            return result
    
    def _start_mission_internal(self) -> Dict:
        """Internal mission start implementation"""
        
        ready, msg = self._check_vehicle_ready()
        if not ready:
            return {
                'success': False,
                'message': f'Cannot start mission: {msg}',
                'timestamp': datetime.now().isoformat()
            }
        
        if self.simulation_mode:
            self.sim_mode = 'AUTO'
            print("🚀 [SIM] Mission started (AUTO mode)")
            return {
                'success': True,
                'message': 'Mission started (simulated)',
                'timestamp': datetime.now().isoformat()
            }
        
        try:
            # Check if armed
            if not self.vehicle.armed:
                return {
                    'success': False,
                    'message': 'Cannot start mission: Vehicle not armed',
                    'timestamp': datetime.now().isoformat()
                }
            
            # Check if mission exists
            if self.vehicle.commands.count == 0:
                return {
                    'success': False,
                    'message': 'No mission uploaded. Upload mission first.',
                    'timestamp': datetime.now().isoformat()
                }
            
            print("🚀 Starting mission (switching to AUTO mode)...")
            result = self.set_mode('AUTO')
            
            if result['success']:
                print("✅ Mission started successfully")
                return {
                    'success': True,
                    'message': 'Mission started in AUTO mode',
                    'timestamp': datetime.now().isoformat()
                }
            else:
                return result
            
        except Exception as e:
            print(f"❌ Mission start failed: {e}")
            return {
                'success': False,
                'message': f'Mission start failed: {str(e)}',
                'timestamp': datetime.now().isoformat()
            }
    
    def get_mission_progress(self) -> Dict:
        """
        Get current mission progress
        
        Returns:
            dict: Mission progress information
        """
        if self.simulation_mode:
            return {
                'mode': self.sim_mode,
                'current_waypoint': 1,
                'total_waypoints': 3,
                'simulation': True,
                'timestamp': datetime.now().isoformat()
            }
        
        if not self.vehicle:
            return {
                'error': 'No vehicle connected',
                'timestamp': datetime.now().isoformat()
            }
        
        try:
            return {
                'mode': self.vehicle.mode.name,
                'current_waypoint': self.vehicle.commands.next,
                'total_waypoints': self.vehicle.commands.count,
                'armed': self.vehicle.armed,
                'timestamp': datetime.now().isoformat()
            }
        except Exception as e:
            return {
                'error': str(e),
                'timestamp': datetime.now().isoformat()
            }
    
    def pause_mission(self) -> Dict:
        """
        Pause the current mission (switch to LOITER)
        
        Returns:
            dict: Pause result
        """
        return self.set_mode('LOITER')
    
    def resume_mission(self) -> Dict:
        """
        Resume the paused mission (switch back to AUTO)
        
        Returns:
            dict: Resume result
        """
        return self.set_mode('AUTO')
    
    def save_mission_to_file(self, waypoints: List[Dict], filename: str) -> Dict:
        """
        Save mission waypoints to JSON file
        
        Args:
            waypoints: List of waypoint dictionaries
            filename: Output filename (e.g., 'mission.json')
        
        Returns:
            dict: Save result
        """
        try:
            mission_data = {
                'version': '1.0',
                'created': datetime.now().isoformat(),
                'waypoint_count': len(waypoints),
                'waypoints': waypoints
            }
            
            with open(filename, 'w') as f:
                json.dump(mission_data, f, indent=2)
            
            print(f"💾 Mission saved to {filename}")
            return {
                'success': True,
                'message': f'Mission saved to {filename}',
                'filename': filename,
                'timestamp': datetime.now().isoformat()
            }
            
        except Exception as e:
            return {
                'success': False,
                'message': f'Failed to save mission: {str(e)}',
                'timestamp': datetime.now().isoformat()
            }
    
    def load_mission_from_file(self, filename: str) -> Dict:
        """
        Load mission waypoints from JSON file
        
        Args:
            filename: Input filename (e.g., 'mission.json')
        
        Returns:
            dict: Load result with waypoints
        """
        try:
            with open(filename, 'r') as f:
                mission_data = json.load(f)
            
            waypoints = mission_data.get('waypoints', [])
            
            print(f"📂 Mission loaded from {filename} - {len(waypoints)} waypoints")
            return {
                'success': True,
                'message': f'Mission loaded from {filename}',
                'waypoints': waypoints,
                'waypoint_count': len(waypoints),
                'timestamp': datetime.now().isoformat()
            }
            
        except Exception as e:
            return {
                'success': False,
                'message': f'Failed to load mission: {str(e)}',
                'timestamp': datetime.now().isoformat()
            }


# Example usage and testing
if __name__ == '__main__':
    """Test the MAVLink Commander module"""
    
    print("🚁 Testing MAVLink Commander Module")
    print("-" * 50)
    
    # Create commander in simulation mode
    commander = MAVLinkCommander(simulation_mode=True)
    
    # Test pre-arm checks
    print("\n1️⃣  Testing pre-arm checks...")
    checks = commander.get_prearm_checks()
    print(f"   Armable: {checks['armable']}")
    print(f"   Message: {checks['message']}")
    
    # Test arming
    print("\n2️⃣  Testing ARM command...")
    result = commander.arm()
    print(f"   Success: {result['success']}")
    print(f"   Message: {result['message']}")
    
    # Test status
    print("\n3️⃣  Checking status...")
    status = commander.get_status()
    print(f"   Armed: {status['armed']}")
    print(f"   Mode: {status['mode']}")
    
    # Test mode change
    print("\n4️⃣  Testing mode change to GUIDED...")
    result = commander.set_mode('GUIDED')
    print(f"   Success: {result['success']}")
    print(f"   Message: {result['message']}")
    
    # Test takeoff
    print("\n5️⃣  Testing TAKEOFF command to 10m...")
    result = commander.takeoff(10.0)
    print(f"   Success: {result['success']}")
    print(f"   Message: {result['message']}")
    
    # Check altitude
    print("\n6️⃣  Checking altitude...")
    alt_info = commander.get_altitude()
    print(f"   Relative altitude: {alt_info.get('relative_altitude', 0):.1f}m")
    
    # Test navigation - get current location
    print("\n7️⃣  Getting current location...")
    location = commander.get_current_location()
    print(f"   Current position: ({location.get('latitude', 0):.6f}, {location.get('longitude', 0):.6f})")
    
    # Test navigation - calculate distance to waypoint
    print("\n8️⃣  Calculating distance to waypoint...")
    target_lat = location.get('latitude', 0) + 0.001  # ~111m north
    target_lon = location.get('longitude', 0) + 0.001  # ~111m east
    distance_info = commander.calculate_distance_to_waypoint(target_lat, target_lon)
    print(f"   Distance: {distance_info.get('distance_meters', 0):.1f}m")
    print(f"   Bearing: {distance_info.get('bearing_degrees', 0):.0f}°")
    
    # Test navigation - goto location
    print("\n9️⃣  Testing GOTO waypoint...")
    result = commander.goto_location(target_lat, target_lon, altitude=10.0)
    print(f"   Success: {result['success']}")
    print(f"   Message: {result['message']}")
    
    # Verify arrival
    print("\n🔟 Verifying arrival at waypoint...")
    new_location = commander.get_current_location()
    print(f"   New position: ({new_location.get('latitude', 0):.6f}, {new_location.get('longitude', 0):.6f})")
    
    # Return to starting position
    print("\n1️⃣1️⃣  Returning to start position...")
    result = commander.goto_location(location.get('latitude', 0), location.get('longitude', 0), altitude=10.0)
    print(f"   Success: {result['success']}")
    
    # Test mission creation and upload
    print("\n1️⃣2️⃣  Creating multi-waypoint mission...")
    start_lat = location.get('latitude', 0)
    start_lon = location.get('longitude', 0)
    mission_waypoints = [
        {'lat': start_lat + 0.0005, 'lon': start_lon + 0.0005, 'alt': 15},
        {'lat': start_lat + 0.0010, 'lon': start_lon + 0.0005, 'alt': 15},
        {'lat': start_lat + 0.0010, 'lon': start_lon + 0.0010, 'alt': 15},
        {'lat': start_lat + 0.0005, 'lon': start_lon + 0.0010, 'alt': 15},
    ]
    print(f"   Created {len(mission_waypoints)} waypoints (square pattern)")
    
    # Upload mission
    print("\n1️⃣3️⃣  Uploading mission to vehicle...")
    result = commander.upload_mission(mission_waypoints, takeoff_alt=15.0)
    print(f"   Success: {result['success']}")
    print(f"   Message: {result['message']}")
    
    # Get mission progress
    print("\n1️⃣4️⃣  Checking mission status...")
    progress = commander.get_mission_progress()
    print(f"   Total waypoints: {progress.get('total_waypoints', 0)}")
    print(f"   Current waypoint: {progress.get('current_waypoint', 0)}")
    
    # Save mission to file
    print("\n1️⃣5️⃣  Saving mission to file...")
    result = commander.save_mission_to_file(mission_waypoints, '/tmp/test_mission.json')
    print(f"   Success: {result['success']}")
    
    # Load mission from file
    print("\n1️⃣6️⃣  Loading mission from file...")
    result = commander.load_mission_from_file('/tmp/test_mission.json')
    print(f"   Success: {result['success']}")
    print(f"   Loaded {result.get('waypoint_count', 0)} waypoints")
    
    # Test landing
    print("\n1️⃣7️⃣  Testing LAND command...")
    result = commander.land()
    print(f"   Success: {result['success']}")
    print(f"   Message: {result['message']}")
    
    # Test disarming
    print("\n1️⃣8️⃣  Verifying vehicle disarmed after landing...")
    status = commander.get_status()
    print(f"   Armed: {status['armed']}")
    
    # Show command history (last 10)
    print("\n1️⃣9️⃣  Command history (last 10):")
    for cmd in commander.get_command_history(limit=10):
        cmd_name = cmd['command'][:30]  # Truncate long command names
        print(f"   {cmd_name}: {cmd['result']['success']}")
    
    print("\n✅ All tests completed!")
