#!/usr/bin/env python3
"""
Test MAVLink Detection Transmission
Verifies if Pi is sending detection data to GCS via MAVLink
"""

import time
import json
from pymavlink import mavutil
from datetime import datetime

def test_mavlink_connection():
    """Test basic MAVLink connection to Pixhawk"""
    print("=" * 60)
    print("Testing MAVLink Connection to Pixhawk")
    print("=" * 60)
    
    try:
        # Connect to Pixhawk on TELEM2
        print("\n1. Connecting to /dev/serial0 at 921600 baud...")
        master = mavutil.mavlink_connection('/dev/serial0', baud=921600)
        
        print("   Waiting for heartbeat...")
        master.wait_heartbeat()
        print(f"   ✅ Connected! System ID: {master.target_system}, Component ID: {master.target_component}")
        
        return master
    except Exception as e:
        print(f"   ❌ Connection failed: {e}")
        return None

def test_send_statustext(master):
    """Test sending STATUSTEXT message"""
    print("\n" + "=" * 60)
    print("Testing STATUSTEXT Transmission")
    print("=" * 60)
    
    try:
        # Send test STATUSTEXT message
        test_message = "TEST|Pi to GCS via MAVLink"
        
        print(f"\n2. Sending test STATUSTEXT: '{test_message}'")
        master.mav.statustext_send(
            mavutil.mavlink.MAV_SEVERITY_INFO,
            test_message.encode('utf-8')
        )
        print("   ✅ Test message sent successfully")
        print("   → Check Mission Planner/QGC MESSAGES tab for: 'TEST|Pi to GCS via MAVLink'")
        
        return True
    except Exception as e:
        print(f"   ❌ Failed to send STATUSTEXT: {e}")
        return False

def test_send_detection(master):
    """Test sending detection message in actual format"""
    print("\n" + "=" * 60)
    print("Testing Detection Message Format")
    print("=" * 60)
    
    try:
        # Create fake detection data (same format as real detection)
        det_id = f"TEST_DET_{int(time.time())}"
        lat = 40.712800
        lon = -74.006000
        confidence = 0.95
        area = 1732
        
        # Format: DET|ID|LAT|LON|CONF|AREA
        message = f"DET|{det_id[:15]}|{lat:.6f}|{lon:.6f}|{confidence:.2f}|{int(area)}"
        
        print(f"\n3. Sending detection message:")
        print(f"   Detection ID: {det_id}")
        print(f"   GPS: {lat:.6f}, {lon:.6f}")
        print(f"   Confidence: {confidence:.2f}")
        print(f"   Area: {area} pixels")
        print(f"\n   Raw message: '{message}'")
        
        master.mav.statustext_send(
            mavutil.mavlink.MAV_SEVERITY_INFO,
            message.encode('utf-8')
        )
        
        print("   ✅ Detection message sent successfully")
        print(f"   → Check GCS for: '{message}'")
        
        return True
    except Exception as e:
        print(f"   ❌ Failed to send detection: {e}")
        return False

def monitor_incoming_messages(master, duration=5):
    """Monitor incoming MAVLink messages to verify forwarding"""
    print("\n" + "=" * 60)
    print(f"Monitoring Incoming Messages ({duration}s)")
    print("=" * 60)
    
    print(f"\n4. Listening for messages (this tests if Pixhawk is forwarding)...")
    print("   Note: If you see STATUSTEXT messages, forwarding is working!")
    
    start_time = time.time()
    msg_count = 0
    statustext_count = 0
    
    while time.time() - start_time < duration:
        msg = master.recv_match(blocking=True, timeout=1.0)
        if msg:
            msg_count += 1
            if msg.get_type() == 'STATUSTEXT':
                statustext_count += 1
                text = msg.text.decode('utf-8') if isinstance(msg.text, bytes) else msg.text
                print(f"   📡 STATUSTEXT received: '{text}'")
    
    print(f"\n   Total messages received: {msg_count}")
    print(f"   STATUSTEXT messages: {statustext_count}")
    
    if statustext_count > 0:
        print("   ✅ Pixhawk is receiving/forwarding STATUSTEXT messages")
    else:
        print("   ⚠️  No STATUSTEXT received (this is normal, they go to TELEM1/GCS)")

def check_config():
    """Check config.json settings"""
    print("\n" + "=" * 60)
    print("Checking Configuration")
    print("=" * 60)
    
    try:
        with open('config.json', 'r') as f:
            config = json.load(f)
        
        print("\n5. Configuration Check:")
        
        # Check pixhawk settings
        pixhawk = config.get('pixhawk', {})
        print(f"\n   Pixhawk:")
        print(f"   - Enabled: {pixhawk.get('enabled', False)}")
        print(f"   - Connection: {pixhawk.get('connection_string', 'N/A')}")
        print(f"   - Baud rate: {pixhawk.get('baud_rate', 'N/A')}")
        print(f"   - Read-only: {pixhawk.get('read_only', True)}")
        
        if pixhawk.get('read_only', True):
            print("   ⚠️  WARNING: read_only=true will prevent sending!")
        else:
            print("   ✅ read_only=false allows sending")
        
        # Check MAVLink detection settings
        mavlink_det = config.get('mavlink_detection', {})
        print(f"\n   MAVLink Detection:")
        print(f"   - Enabled: {mavlink_det.get('enabled', False)}")
        print(f"   - Send metadata: {mavlink_det.get('send_metadata', False)}")
        
        if mavlink_det.get('enabled', False):
            print("   ✅ MAVLink detection transmission is enabled")
        else:
            print("   ❌ MAVLink detection transmission is DISABLED")
        
        # Check detection settings
        detection = config.get('detection', {})
        print(f"\n   Detection:")
        print(f"   - Enabled: {detection.get('enabled', False)}")
        print(f"   - Auto-detect in AUTO mode: {detection.get('auto_detect_in_auto_mode', False)}")
        
        return True
    except FileNotFoundError:
        print("   ❌ config.json not found")
        return False
    except Exception as e:
        print(f"   ❌ Error reading config: {e}")
        return False

def main():
    """Run all tests"""
    print("\n")
    print("█" * 60)
    print("█  MAVLink Detection Transmission Test")
    print("█  Verifies Pi → Pixhawk → GCS communication")
    print("█" * 60)
    
    # Check config first
    check_config()
    
    # Test MAVLink connection
    master = test_mavlink_connection()
    
    if not master:
        print("\n" + "=" * 60)
        print("❌ Cannot proceed without MAVLink connection")
        print("=" * 60)
        print("\nTroubleshooting:")
        print("1. Check /dev/serial0 exists: ls -l /dev/serial0")
        print("2. Check permissions: sudo usermod -a -G dialout $USER")
        print("3. Verify TELEM2 physical connection to Pi")
        print("4. Check baud rate matches Pixhawk (921600)")
        return
    
    # Test STATUSTEXT sending
    if not test_send_statustext(master):
        print("\n⚠️  STATUSTEXT sending failed - check connection and read_only setting")
        return
    
    # Wait a bit for message to propagate
    time.sleep(1)
    
    # Test detection message format
    if not test_send_detection(master):
        print("\n⚠️  Detection message sending failed")
        return
    
    # Monitor incoming messages briefly
    monitor_incoming_messages(master, duration=3)
    
    # Send a few more test detections
    print("\n" + "=" * 60)
    print("Sending Multiple Test Detections")
    print("=" * 60)
    
    print("\n6. Sending 3 test detections (1 per second)...")
    for i in range(3):
        det_id = f"TEST_{i+1:03d}"
        lat = 40.712800 + (i * 0.0001)
        lon = -74.006000 + (i * 0.0001)
        message = f"DET|{det_id}|{lat:.6f}|{lon:.6f}|0.{90+i}|1732"
        
        master.mav.statustext_send(
            mavutil.mavlink.MAV_SEVERITY_INFO,
            message.encode('utf-8')
        )
        print(f"   {i+1}. Sent: {message}")
        time.sleep(1)
    
    print("   ✅ All test detections sent")
    
    # Summary
    print("\n" + "=" * 60)
    print("Test Summary")
    print("=" * 60)
    
    print("\n✅ Pi can send MAVLink messages to Pixhawk")
    print("\n📋 Next Steps:")
    print("   1. On GCS (Mission Planner/QGC):")
    print("      - Connect to drone via TELEM1 radio")
    print("      - Open Messages/STATUSTEXT tab")
    print("      - Look for messages starting with 'TEST|' or 'DET|'")
    print()
    print("   2. If messages appear on GCS:")
    print("      ✅ Full transmission path working: Pi → TELEM2 → Pixhawk → TELEM1 → GCS")
    print()
    print("   3. If messages DON'T appear on GCS:")
    print("      - Check Pixhawk SERIAL2 parameters (SR2_*, SERIAL2_PROTOCOL)")
    print("      - Verify TELEM1 radio connection")
    print("      - Check GCS is showing STATUSTEXT messages")
    print()
    print("   4. To test with real detections:")
    print("      - Start pi_controller.py")
    print("      - Trigger detection (auto in AUTO mode or via command)")
    print("      - Monitor GCS for 'DET|' messages with real GPS coordinates")
    print()

if __name__ == '__main__':
    try:
        main()
    except KeyboardInterrupt:
        print("\n\n⚠️  Test interrupted by user")
    except Exception as e:
        print(f"\n\n❌ Unexpected error: {e}")
        import traceback
        traceback.print_exc()
