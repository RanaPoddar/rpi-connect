#!/usr/bin/env python3
"""
Pixhawk Parameter Setup for Dual Telemetry
Automatically configures TELEM1 (GCS) and TELEM2 (Raspberry Pi)
"""

from pymavlink import mavutil
import time
import sys

def connect_pixhawk(connection_string, baud):
    """Connect to Pixhawk"""
    print(f"Connecting to {connection_string} at {baud} baud...")
    master = mavutil.mavlink_connection(connection_string, baud=baud)
    
    print("Waiting for heartbeat...")
    master.wait_heartbeat()
    print(f"✅ Connected to system {master.target_system}, component {master.target_component}")
    return master

def get_parameter(master, param_name):
    """Get a parameter value"""
    master.mav.param_request_read_send(
        master.target_system,
        master.target_component,
        param_name.encode('utf-8'),
        -1
    )
    
    message = master.recv_match(type='PARAM_VALUE', blocking=True, timeout=3)
    if message and message.param_id == param_name:
        return message.param_value
    return None

def set_parameter(master, param_name, param_value, param_type=mavutil.mavlink.MAV_PARAM_TYPE_REAL32):
    """Set a parameter value"""
    master.mav.param_set_send(
        master.target_system,
        master.target_component,
        param_name.encode('utf-8'),
        param_value,
        param_type
    )
    
    # Wait for confirmation
    message = master.recv_match(type='PARAM_VALUE', blocking=True, timeout=3)
    if message and message.param_id == param_name:
        print(f"   ✓ {param_name} = {message.param_value}")
        return True
    else:
        print(f"   ❌ Failed to set {param_name}")
        return False

def configure_pixhawk(master):
    """Configure Pixhawk for dual telemetry operation"""
    
    print("\n" + "="*70)
    print("PIXHAWK DUAL TELEMETRY CONFIGURATION")
    print("="*70)
    
    # Parameters to configure
    params = {
        # TELEM1 - GCS Radio (long range)
        'SERIAL1_PROTOCOL': (2, mavutil.mavlink.MAV_PARAM_TYPE_INT8, "MAVLink2 on TELEM1"),
        'SERIAL1_BAUD': (57, mavutil.mavlink.MAV_PARAM_TYPE_INT32, "57600 baud (telemetry radio)"),
        
        # TELEM2 - Raspberry Pi (high speed local)
        'SERIAL2_PROTOCOL': (2, mavutil.mavlink.MAV_PARAM_TYPE_INT8, "MAVLink2 on TELEM2"),
        'SERIAL2_BAUD': (921, mavutil.mavlink.MAV_PARAM_TYPE_INT32, "921600 baud (Pi connection)"),
        
        # Stream rates for TELEM1 (GCS) - moderate rates for long range
        'SR1_POSITION': (2, mavutil.mavlink.MAV_PARAM_TYPE_INT16, "GPS position @ 2Hz to GCS"),
        'SR1_EXTRA1': (4, mavutil.mavlink.MAV_PARAM_TYPE_INT16, "Attitude @ 4Hz to GCS"),
        'SR1_EXTRA2': (2, mavutil.mavlink.MAV_PARAM_TYPE_INT16, "VFR_HUD @ 2Hz to GCS"),
        'SR1_EXTRA3': (2, mavutil.mavlink.MAV_PARAM_TYPE_INT16, "Other data @ 2Hz to GCS"),
        'SR1_EXT_STAT': (2, mavutil.mavlink.MAV_PARAM_TYPE_INT16, "Extended status @ 2Hz (enables STATUSTEXT)"),
        
        # Stream rates for TELEM2 (Pi) - high rates for local connection
        'SR2_POSITION': (10, mavutil.mavlink.MAV_PARAM_TYPE_INT16, "GPS position @ 10Hz to Pi"),
        'SR2_EXTRA1': (10, mavutil.mavlink.MAV_PARAM_TYPE_INT16, "Attitude @ 10Hz to Pi"),
        'SR2_EXTRA2': (10, mavutil.mavlink.MAV_PARAM_TYPE_INT16, "VFR_HUD @ 10Hz to Pi"),
        'SR2_EXTRA3': (2, mavutil.mavlink.MAV_PARAM_TYPE_INT16, "Other data @ 2Hz to Pi"),
    }
    
    print("\n1️⃣  Reading current parameters...")
    print("-" * 70)
    
    current_values = {}
    for param_name in params:
        value = get_parameter(master, param_name)
        current_values[param_name] = value
        if value is not None:
            print(f"   {param_name:20s} = {value}")
        else:
            print(f"   {param_name:20s} = NOT FOUND")
    
    print("\n2️⃣  Setting new parameters...")
    print("-" * 70)
    
    changes_made = False
    for param_name, (param_value, param_type, description) in params.items():
        current = current_values.get(param_name)
        
        if current == param_value:
            print(f"   ✓ {param_name} already set to {param_value} ({description})")
        else:
            print(f"   ⏳ Setting {param_name} to {param_value} ({description})")
            if set_parameter(master, param_name, param_value, param_type):
                changes_made = True
            time.sleep(0.2)
    
    if changes_made:
        print("\n3️⃣  Saving parameters to EEPROM...")
        print("-" * 70)
        
        # Request parameter write
        master.mav.command_long_send(
            master.target_system,
            master.target_component,
            mavutil.mavlink.MAV_CMD_PREFLIGHT_STORAGE,
            0,  # confirmation
            1,  # Action: 1 = Write params to EEPROM
            0, 0, 0, 0, 0, 0
        )
        
        # Wait for ACK
        ack = master.recv_match(type='COMMAND_ACK', blocking=True, timeout=5)
        if ack and ack.result == mavutil.mavlink.MAV_RESULT_ACCEPTED:
            print("   ✅ Parameters saved to EEPROM")
        else:
            print("   ⚠️  Could not confirm parameter save")
        
        print("\n⚠️  IMPORTANT: Reboot Pixhawk for changes to take effect!")
        print("   - Disconnect power and USB")
        print("   - Wait 5 seconds")
        print("   - Reconnect power")
    else:
        print("\n✅ All parameters already configured correctly")
    
    print("\n" + "="*70)
    print("CONFIGURATION COMPLETE")
    print("="*70)
    
    print("\nNext steps:")
    print("1. Reboot Pixhawk (if changes were made)")
    print("2. Connect GCS radio to TELEM1")
    print("3. Connect Raspberry Pi to TELEM2 (or use USB)")
    print("4. Run: python pi_controller.py")
    print("5. Verify GPS coordinates in detections")

def verify_gps(master):
    """Verify GPS is working"""
    print("\n4️⃣  Verifying GPS reception...")
    print("-" * 70)
    
    msg = master.recv_match(type='GPS_RAW_INT', blocking=True, timeout=5)
    if msg:
        lat = msg.lat / 1e7
        lon = msg.lon / 1e7
        alt = msg.alt / 1000.0
        sats = msg.satellites_visible
        fix = msg.fix_type
        
        fix_types = {0: "No Fix", 1: "No Fix", 2: "2D Fix", 3: "3D Fix", 4: "DGPS", 5: "RTK Float", 6: "RTK Fixed"}
        
        print(f"   GPS Status: {fix_types.get(fix, 'Unknown')}")
        print(f"   Satellites: {sats}")
        print(f"   Position: {lat:.6f}, {lon:.6f}")
        print(f"   Altitude: {alt:.1f}m")
        
        if fix >= 3:
            print("   ✅ GPS has 3D fix - ready for missions")
        else:
            print("   ⚠️  GPS does not have 3D fix - move outdoors")
    else:
        print("   ❌ No GPS data received")

def main():
    if len(sys.argv) < 2:
        print("Usage: python setup_pixhawk_params.py <connection_string> [baud]")
        print("\nExamples:")
        print("  python setup_pixhawk_params.py /dev/ttyACM0 115200")
        print("  python setup_pixhawk_params.py /dev/serial0 57600")
        print("  python setup_pixhawk_params.py COM5 57600")
        sys.exit(1)
    
    connection_string = sys.argv[1]
    baud = int(sys.argv[2]) if len(sys.argv) > 2 else 115200
    
    try:
        # Connect to Pixhawk
        master = connect_pixhawk(connection_string, baud)
        
        # Configure parameters
        configure_pixhawk(master)
        
        # Verify GPS
        verify_gps(master)
        
        master.close()
        
    except KeyboardInterrupt:
        print("\n\nInterrupted by user")
    except Exception as e:
        print(f"\n❌ Error: {e}")
        import traceback
        traceback.print_exc()

if __name__ == '__main__':
    main()
