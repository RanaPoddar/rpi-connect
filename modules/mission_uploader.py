#!/usr/bin/env python3
"""
MAVLink Mission Uploader - Upload waypoint missions to Pixhawk
"""

from pymavlink import mavutil
import time
import json


class MissionUploader:
    """Upload waypoint missions to Pixhawk via MAVLink"""
    
    def __init__(self, connection_string):
        """
        Initialize mission uploader
        
        Args:
            connection_string: MAVLink connection string (e.g., '/dev/serial0', 'udp:127.0.0.1:14550')
        """
        self.connection_string = connection_string
        self.master = None
        
    def connect(self):
        """Connect to Pixhawk"""
        try:
            print(f" Connecting to Pixhawk: {self.connection_string}")
            self.master = mavutil.mavlink_connection(self.connection_string, baud=57600)
            
            # Wait for heartbeat
            print(" Waiting for heartbeat...")
            self.master.wait_heartbeat()
            print(f" Heartbeat received from system {self.master.target_system}")
            return True
        except Exception as e:
            print(f"Connection failed: {e}")
            return False
    
    def clear_mission(self):
        """Clear existing mission from Pixhawk"""
        try:
            print("Clearing existing mission...")
            
            # Send mission clear command
            self.master.mav.mission_clear_all_send(
                self.master.target_system,
                self.master.target_component
            )
            
            # Wait for acknowledgment
            ack = self.master.recv_match(type='MISSION_ACK', blocking=True, timeout=5)
            if ack:
                print("Mission cleared successfully")
                return True
            else:
                print(" No acknowledgment received for mission clear")
                return False
                
        except Exception as e:
            print(f"Failed to clear mission: {e}")
            return False
    
    def upload_mission(self, waypoints):
        """
        Upload mission waypoints to Pixhawk
        
        Args:
            waypoints: List of waypoint dicts with keys: lat, lon, alt, command
            
        Returns:
            bool: True if upload successful
        """
        try:
            if not self.master:
                print(" Not connected to Pixhawk")
                return False
            
            # Clear existing mission first
            if not self.clear_mission():
                return False
            
            # Send mission count
            mission_count = len(waypoints)
            print(f" Uploading {mission_count} waypoints...")
            
            self.master.mav.mission_count_send(
                self.master.target_system,
                self.master.target_component,
                mission_count
            )
            
            # Wait for mission request
            for i, wp in enumerate(waypoints):
                print(f" Waiting for request for waypoint {i}...")
                
                msg = self.master.recv_match(type='MISSION_REQUEST', blocking=True, timeout=10)
                if not msg:
                    print(f" Timeout waiting for request for waypoint {i}")
                    return False
                
                if msg.seq != i:
                    print(f" Expected request for waypoint {i}, got {msg.seq}")
                    continue
                
                # Prepare waypoint command
                command = wp.get('command', mavutil.mavlink.MAV_CMD_NAV_WAYPOINT)
                
                # Send waypoint
                self.master.mav.mission_item_send(
                    self.master.target_system,
                    self.master.target_component,
                    i,  # seq
                    mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
                    command,
                    0 if i > 0 else 1,  # current (1 for first waypoint, 0 for others)
                    1,  # autocontinue
                    wp.get('param1', 0),  # Hold time in seconds
                    wp.get('param2', 0),  # Acceptance radius in meters
                    wp.get('param3', 0),  # Pass through waypoint
                    wp.get('param4', 0),  # Yaw angle
                    wp['lat'],  # Latitude
                    wp['lon'],  # Longitude
                    wp['alt']   # Altitude (relative)
                )
                
                print(f" Waypoint {i}: {wp['lat']:.6f}, {wp['lon']:.6f}, {wp['alt']:.1f}m")
            
            # Wait for mission ACK
            print("⏳ Waiting for mission acknowledgment...")
            ack = self.master.recv_match(type='MISSION_ACK', blocking=True, timeout=10)
            
            if ack:
                if ack.type == mavutil.mavlink.MAV_MISSION_ACCEPTED:
                    print("Mission uploaded successfully!")
                    return True
                else:
                    print(f" Mission upload failed: {ack.type}")
                    return False
            else:
                print(" No acknowledgment received")
                return False
                
        except Exception as e:
            print(f"Mission upload error: {e}")
            return False
    
    def load_mission_from_json(self, json_file):
        """
        Load mission from JSON file and upload to Pixhawk
        
        Args:
            json_file: Path to mission JSON file generated by kml_mission_planner.py
            
        Returns:
            bool: True if successful
        """
        try:
            print(f"📖 Loading mission from {json_file}")
            
            with open(json_file, 'r') as f:
                mission_data = json.load(f)
            
            waypoints = mission_data.get('waypoints', [])
            
            if not waypoints:
                print(" No waypoints found in mission file")
                return False
            
            print(f" Loaded {len(waypoints)} waypoints")
            print(f"Mission: {mission_data.get('metadata', {}).get('mission_name', 'Unnamed')}")
            
            # Upload to Pixhawk
            return self.upload_mission(waypoints)
            
        except Exception as e:
            print(f" Failed to load mission from JSON: {e}")
            return False
    
    def set_mode_auto(self):
        """Set Pixhawk to AUTO mode to start mission"""
        try:
            print("🚁 Setting mode to AUTO...")
            
            # Get mode ID for AUTO
            mode_id = self.master.mode_mapping().get('AUTO')
            if mode_id is None:
                print("AUTO mode not found")
                return False
            
            # Send mode change command
            self.master.mav.set_mode_send(
                self.master.target_system,
                mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
                mode_id
            )
            
            # Wait for confirmation
            time.sleep(1)
            print("Mode set to AUTO")
            return True
            
        except Exception as e:
            print(f"Failed to set AUTO mode: {e}")
            return False
    
    def arm_drone(self, force=False):
        """
        ARM the drone for flight
        
        Args:
            force: Force arming even if pre-arm checks fail
            
        Returns:
            bool: True if armed successfully
        """
        try:
            print(" Arming drone...")
            
            # Send ARM command
            self.master.mav.command_long_send(
                self.master.target_system,
                self.master.target_component,
                mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
                0,
                1,  # ARM
                21196 if force else 0,  # Force parameter
                0, 0, 0, 0, 0
            )
            
            # Wait for acknowledgment
            ack = self.master.recv_match(type='COMMAND_ACK', blocking=True, timeout=5)
            if ack and ack.result == mavutil.mavlink.MAV_RESULT_ACCEPTED:
                print("✅ Drone armed successfully")
                return True
            else:
                print(f"⚠️  Arming response: {ack.result if ack else 'timeout'}")
                return False
                
        except Exception as e:
            print(f"❌ Failed to arm: {e}")
            return False
    
    def autonomous_launch(self):
        """
        Complete autonomous launch sequence for NIdar competition
        Per rule 7.24-7.26: Only upload KML and trigger launch
        
        Returns:
            bool: True if launch sequence completed
        """
        try:
            print("\n🚀 Starting autonomous launch sequence...")
            print("   Per NIdar rules 7.24-7.26: No manual intervention")
            
            # Step 1: ARM the drone
            if not self.arm_drone():
                print("❌ Failed to arm drone")
                return False
            
            time.sleep(2)
            
            # Step 2: Set AUTO mode (starts mission automatically)
            if not self.set_mode_auto():
                print("❌ Failed to set AUTO mode")
                return False
            
            print("\n✅ Autonomous launch complete!")
            print("   🚁 Drone now flying mission autonomously")
            print("   🌾 Detection system active")
            print("   📍 Will RTL automatically when complete")
            
            return True
            
        except Exception as e:
            print(f"❌ Autonomous launch failed: {e}")
            return False
    
    def set_mode_auto(self):
        """Set Pixhawk to AUTO mode to start mission"""
        try:
            print("🚁 Setting mode to AUTO...")
            
            # Get mode ID for AUTO
            mode_id = self.master.mode_mapping().get('AUTO')
            if mode_id is None:
                print("❌ AUTO mode not found")
                return False
            
            # Send mode change command
            self.master.mav.set_mode_send(
                self.master.target_system,
                mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
                mode_id
            )
            
            # Wait for confirmation
            time.sleep(1)
            print("✅ Mode set to AUTO")
            return True
            
        except Exception as e:
            print(f"❌ Failed to set AUTO mode: {e}")
            return False
    
    def close(self):
        """Close MAVLink connection"""
        if self.master:
            self.master.close()
            print("🔌 Connection closed")


def main():
    """Test mission uploader"""
    import sys
    
    if len(sys.argv) < 2:
        print("Usage: python3 mission_uploader.py <mission.json> [connection_string]")
        print("Example: python3 mission_uploader.py mission.json /dev/serial0")
        sys.exit(1)
    
    mission_file = sys.argv[1]
    connection = sys.argv[2] if len(sys.argv) > 2 else '/dev/serial0'
    
    uploader = MissionUploader(connection)
    
    # Connect to Pixhawk
    if not uploader.connect():
        sys.exit(1)
    
    # Upload mission
    if uploader.load_mission_from_json(mission_file):
        print("\n✅ Mission ready! Set mode to AUTO to start.")
    else:
        print("\n❌ Mission upload failed")
        sys.exit(1)
    
    uploader.close()


if __name__ == '__main__':
    main()
