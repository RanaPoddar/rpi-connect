#!/usr/bin/env python3
"""
Pixhawk Connection Debugger
Systematically tests different serial ports and baud rates
"""

import serial
import time
from pymavlink import mavutil
import sys

print("=" * 60)
print("PIXHAWK CONNECTION DEBUGGER")
print("=" * 60)

# Common serial ports on Raspberry Pi
PORTS_TO_TEST = [
    '/dev/ttyACM0',  # USB connection (most common)
    '/dev/ttyAMA0',  # GPIO serial (older Pi models)
    '/dev/serial0',  # GPIO serial (newer Pi models)
    '/dev/ttyUSB0',  # USB-to-serial adapter
]

# Common baud rates
BAUD_RATES = [57600, 115200, 921600]

def check_serial_ports():
    """Check which serial ports exist"""
    print("\n1️⃣  CHECKING AVAILABLE SERIAL PORTS...")
    print("-" * 60)
    
    import os
    available_ports = []
    
    for port in PORTS_TO_TEST:
        if os.path.exists(port):
            print(f"✅ {port} exists")
            available_ports.append(port)
            
            # Check permissions
            try:
                with open(port, 'r'):
                    print(f"   ✓ Readable")
            except PermissionError:
                print(f"   ❌ Permission denied - Run: sudo usermod -a -G dialout $USER")
            except Exception as e:
                print(f"   ⚠️  {e}")
        else:
            print(f"❌ {port} not found")
    
    return available_ports

def check_port_in_use(port):
    """Check if port is already in use"""
    try:
        ser = serial.Serial(port, 57600, timeout=1)
        ser.close()
        return False
    except serial.SerialException as e:
        if "in use" in str(e).lower() or "busy" in str(e).lower():
            return True
        return False

def test_mavlink_connection(port, baud):
    """Test MAVLink connection on specific port and baud rate"""
    print(f"\n   Testing {port} at {baud} baud...")
    
    # Check if port is in use
    if check_port_in_use(port):
        print(f"   ⚠️  Port is in use by another process")
        return False
    
    try:
        # Create MAVLink connection with short timeout
        master = mavutil.mavlink_connection(
            port,
            baud=baud,
            source_system=255
        )
        
        print(f"   ⏳ Waiting for heartbeat (10 seconds)...")
        
        # Wait for heartbeat with timeout
        msg = master.wait_heartbeat(timeout=10)
        
        if msg:
            print(f"   ✅ HEARTBEAT RECEIVED!")
            print(f"      System ID: {master.target_system}")
            print(f"      Component ID: {master.target_component}")
            print(f"      MAVLink Version: {msg.mavlink_version}")
            print(f"      Autopilot: {msg.autopilot}")
            print(f"      Type: {msg.type}")
            
            # Try to get a few more messages
            print(f"   📡 Receiving messages...")
            for i in range(5):
                m = master.recv_match(blocking=True, timeout=2)
                if m:
                    print(f"      - {m.get_type()}")
            
            master.close()
            return True
        else:
            print(f"   ❌ No heartbeat received")
            master.close()
            return False
            
    except Exception as e:
        print(f"   ❌ Connection failed: {e}")
        return False

def check_pixhawk_power():
    """Check if Pixhawk might be powered"""
    print("\n2️⃣  HARDWARE CHECKLIST")
    print("-" * 60)
    print("Manual checks:")
    print("  □ Pixhawk LED is on (red/blue/green)")
    print("  □ USB cable is connected (if using USB)")
    print("  □ Battery is connected (for standalone power)")
    print("  □ Pixhawk is booted (wait 10-15 seconds after power on)")
    print("  □ No other programs are connected (Mission Planner, QGC, etc.)")
    input("\n Press Enter after verifying the above...")

def main():
    # Step 1: Check available ports
    available_ports = check_serial_ports()
    
    if not available_ports:
        print("\n❌ No serial ports found!")
        print("\nTroubleshooting:")
        print("  1. Connect Pixhawk via USB")
        print("  2. Run: ls -l /dev/tty* | grep ACM")
        print("  3. Run: dmesg | tail -20  (to see connection logs)")
        return
    
    # Step 2: Hardware checklist
    check_pixhawk_power()
    
    # Step 3: Test connections
    print("\n3️⃣  TESTING MAVLINK CONNECTIONS...")
    print("-" * 60)
    
    success = False
    working_config = None
    
    for port in available_ports:
        print(f"\n🔌 Testing port: {port}")
        for baud in BAUD_RATES:
            if test_mavlink_connection(port, baud):
                success = True
                working_config = (port, baud)
                print(f"\n{'='*60}")
                print(f"✅ SUCCESS! Working configuration found:")
                print(f"   Port: {port}")
                print(f"   Baud Rate: {baud}")
                print(f"{'='*60}")
                break
        
        if success:
            break
    
    if not success:
        print("\n" + "="*60)
        print("❌ NO WORKING CONNECTION FOUND")
        print("="*60)
        print("\nTroubleshooting steps:")
        print("\n1. Check USB connection:")
        print("   lsusb")
        print("   dmesg | grep -i usb | tail -20")
        print("\n2. Check serial permissions:")
        print("   sudo usermod -a -G dialout $USER")
        print("   sudo usermod -a -G tty $USER")
        print("   (Then logout and login)")
        print("\n3. Kill competing processes:")
        print("   sudo lsof | grep /dev/tty")
        print("   sudo pkill -9 mavproxy")
        print("\n4. Check Pixhawk is in the right mode:")
        print("   - Connect via Mission Planner/QGroundControl first")
        print("   - Verify firmware is working")
        print("   - Check SERIAL1_PROTOCOL or SERIAL2_PROTOCOL = 2 (MAVLink2)")
        print("\n5. Try different USB cable")
        print("   - Some cables are power-only, not data cables")
        print("\n6. Check Pixhawk parameters:")
        print("   - SERIAL0_BAUD (USB): usually 115200")
        print("   - SERIAL1_BAUD (Telem1): usually 57600")
        print("   - SERIAL2_BAUD (Telem2): usually 57600")
    else:
        print("\n" + "="*60)
        print("CONFIGURATION TO USE:")
        print("="*60)
        print(f"\nUpdate your config.json:")
        print(f'''
"pixhawk": {{
  "enabled": true,
  "connection_string": "{working_config[0]}",
  "baud_rate": {working_config[1]},
  "simulation_mode": false
}}
''')

if __name__ == '__main__':
    try:
        main()
    except KeyboardInterrupt:
        print("\n\nDebug interrupted by user")
    except Exception as e:
        print(f"\n❌ Error: {e}")
        import traceback
        traceback.print_exc()
