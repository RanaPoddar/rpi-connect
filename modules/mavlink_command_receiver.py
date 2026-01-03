"""
MAVLink Command Receiver - Listens for custom MAVLink commands from GCS
Allows control of Pi functions over long-range MAVLink radio (1-10km)
"""

import time
import threading
from typing import Callable, Dict
from pymavlink import mavutil

class MAVLinkCommandReceiver:
    """
    Receives custom MAVLink commands from GCS to control Pi functions.
    Uses COMMAND_LONG with custom command IDs for detection control.
    
    Custom Command IDs:
    - 42000: Start detection
    - 42001: Stop detection  
    - 42002: Request detection stats
    - 42003: Start periodic capture
    - 42004: Stop periodic capture
    """
    
    def __init__(self, master, callback: Callable = None):
        """
        Initialize MAVLink command receiver
        
        Args:
            master: PyMAVLink connection object
            callback: Optional callback function for command events
        """
        self.master = master
        self.callback = callback
        self.running = False
        self.thread = None
        
        # Command handlers registry
        self.command_handlers = {}
        self._register_default_handlers()
        
    def _register_default_handlers(self):
        """Register default command handlers"""
        self.register_handler(42000, self._handle_start_detection, "Start Detection")
        self.register_handler(42001, self._handle_stop_detection, "Stop Detection")
        self.register_handler(42002, self._handle_request_stats, "Request Stats")
        self.register_handler(42003, self._handle_start_capture, "Start Periodic Capture")
        self.register_handler(42004, self._handle_stop_capture, "Stop Periodic Capture")
        
    def register_handler(self, command_id: int, handler: Callable, description: str = ""):
        """
        Register a custom command handler
        
        Args:
            command_id: MAVLink command ID (use 42000-42999 for custom commands)
            handler: Function to call when command received (receives params dict)
            description: Human-readable description
        """
        self.command_handlers[command_id] = {
            'handler': handler,
            'description': description
        }
        print(f"📡 Registered MAVLink command: {command_id} - {description}")
        
    def _handle_start_detection(self, params: Dict):
        """Handle start detection command"""
        print("🌾 MAVLink Command: START DETECTION")
        if self.callback:
            self.callback('start_detection', params)
        return True
        
    def _handle_stop_detection(self, params: Dict):
        """Handle stop detection command"""
        print("🌾 MAVLink Command: STOP DETECTION")
        if self.callback:
            self.callback('stop_detection', params)
        return True
        
    def _handle_request_stats(self, params: Dict):
        """Handle request stats command"""
        print("📊 MAVLink Command: REQUEST DETECTION STATS")
        if self.callback:
            self.callback('request_stats', params)
        return True
        
    def _handle_start_capture(self, params: Dict):
        """Handle start periodic capture command"""
        print("📸 MAVLink Command: START PERIODIC CAPTURE")
        if self.callback:
            self.callback('start_capture', params)
        return True
        
    def _handle_stop_capture(self, params: Dict):
        """Handle stop periodic capture command"""
        print("📸 MAVLink Command: STOP PERIODIC CAPTURE")
        if self.callback:
            self.callback('stop_capture', params)
        return True
        
    def _listen_loop(self):
        """Background thread that listens for MAVLink commands"""
        print("📡 MAVLink command receiver started")
        
        while self.running:
            try:
                # Wait for COMMAND_LONG messages
                msg = self.master.recv_match(type='COMMAND_LONG', blocking=True, timeout=1.0)
                
                if msg:
                    command_id = msg.command
                    
                    # Check if this is a custom command we handle
                    if command_id in self.command_handlers:
                        handler_info = self.command_handlers[command_id]
                        
                        # Extract parameters from MAVLink message
                        params = {
                            'param1': msg.param1,
                            'param2': msg.param2,
                            'param3': msg.param3,
                            'param4': msg.param4,
                            'param5': msg.param5,
                            'param6': msg.param6,
                            'param7': msg.param7,
                            'target_system': msg.target_system,
                            'target_component': msg.target_component
                        }
                        
                        print(f"📡 Received MAVLink command: {command_id} ({handler_info['description']})")
                        
                        # Call the handler
                        try:
                            success = handler_info['handler'](params)
                            
                            # Send acknowledgment back to GCS
                            self._send_ack(command_id, success)
                            
                        except Exception as e:
                            print(f"❌ Command handler error: {e}")
                            self._send_ack(command_id, False)
                    
            except Exception as e:
                if self.running:
                    print(f"⚠️  MAVLink receive error: {e}")
                time.sleep(0.1)
                
        print("📡 MAVLink command receiver stopped")
        
    def _send_ack(self, command_id: int, success: bool):
        """Send command acknowledgment to GCS"""
        try:
            result = mavutil.mavlink.MAV_RESULT_ACCEPTED if success else mavutil.mavlink.MAV_RESULT_FAILED
            
            self.master.mav.command_ack_send(
                command=command_id,
                result=result
            )
            
            print(f"✅ Sent ACK for command {command_id}: {'SUCCESS' if success else 'FAILED'}")
            
        except Exception as e:
            print(f"⚠️  Failed to send ACK: {e}")
            
    def start(self):
        """Start listening for MAVLink commands"""
        if not self.running:
            self.running = True
            self.thread = threading.Thread(target=self._listen_loop, daemon=True)
            self.thread.start()
            print("🚀 MAVLink command receiver started")
            
    def stop(self):
        """Stop listening for MAVLink commands"""
        self.running = False
        if self.thread:
            self.thread.join(timeout=2.0)
        print("🛑 MAVLink command receiver stopped")


class MAVLinkCommandSender:
    """
    Sends custom MAVLink commands from GCS to Pi.
    Used on GCS side to send long-range control commands.
    """
    
    def __init__(self, master):
        """
        Initialize MAVLink command sender
        
        Args:
            master: PyMAVLink connection object
        """
        self.master = master
        
    def send_command(self, command_id: int, param1=0, param2=0, param3=0, 
                    param4=0, param5=0, param6=0, param7=0,
                    target_system=1, target_component=1):
        """
        Send custom MAVLink command
        
        Args:
            command_id: Command ID (42000-42999 for custom)
            param1-7: Command parameters
            target_system: Target system ID (usually 1 for Pixhawk)
            target_component: Target component ID (usually 1 for autopilot)
        """
        try:
            self.master.mav.command_long_send(
                target_system,
                target_component,
                command_id,
                0,  # confirmation
                param1,
                param2,
                param3,
                param4,
                param5,
                param6,
                param7
            )
            
            print(f"📡 Sent MAVLink command {command_id}")
            
            # Wait for acknowledgment
            ack = self.master.recv_match(type='COMMAND_ACK', blocking=True, timeout=3.0)
            
            if ack and ack.command == command_id:
                if ack.result == mavutil.mavlink.MAV_RESULT_ACCEPTED:
                    print(f"✅ Command {command_id} accepted")
                    return True
                else:
                    print(f"❌ Command {command_id} rejected (result={ack.result})")
                    return False
            else:
                print(f"⚠️  No ACK received for command {command_id}")
                return False
                
        except Exception as e:
            print(f"❌ Failed to send MAVLink command: {e}")
            return False
            
    def start_detection(self, target_system=1):
        """Send start detection command"""
        return self.send_command(42000, target_system=target_system)
        
    def stop_detection(self, target_system=1):
        """Send stop detection command"""
        return self.send_command(42001, target_system=target_system)
        
    def request_stats(self, target_system=1):
        """Request detection statistics"""
        return self.send_command(42002, target_system=target_system)
        
    def start_periodic_capture(self, interval_seconds=2.0, target_system=1):
        """Start periodic image capture"""
        return self.send_command(42003, param1=interval_seconds, target_system=target_system)
        
    def stop_periodic_capture(self, target_system=1):
        """Stop periodic image capture"""
        return self.send_command(42004, target_system=target_system)


# Example usage on Pi side
if __name__ == "__main__":
    print("MAVLink Command Receiver Test")
    
    def command_callback(command_type, params):
        """Handle received commands"""
        print(f"🎯 Command received: {command_type}")
        print(f"   Parameters: {params}")
        
        # In real implementation, this would trigger actual Pi actions
        if command_type == 'start_detection':
            print("   → Would start crop detection here")
        elif command_type == 'stop_detection':
            print("   → Would stop crop detection here")
    
    # Connect to Pixhawk
    try:
        master = mavutil.mavlink_connection('/dev/serial0', baud=57600)
        master.wait_heartbeat()
        print("✅ Connected to Pixhawk")
        
        # Start command receiver
        receiver = MAVLinkCommandReceiver(master, callback=command_callback)
        receiver.start()
        
        print("📡 Listening for MAVLink commands... (Press Ctrl+C to exit)")
        
        while True:
            time.sleep(1)
            
    except KeyboardInterrupt:
        print("\n🛑 Shutting down...")
        receiver.stop()
    except Exception as e:
        print(f"❌ Error: {e}")
