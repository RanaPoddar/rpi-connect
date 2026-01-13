#!/usr/bin/env python3
"""
Simple script to manually start/stop detection on Pi
Run this after pi_controller.py is already running

Usage:
    python3 toggle_detection.py start
    python3 toggle_detection.py stop
    python3 toggle_detection.py status
"""

import sys
import os
import signal
import time

def find_pi_controller_pid():
    """Find the PID of running pi_controller.py"""
    try:
        result = os.popen("pgrep -f 'python3.*pi_controller.py'").read().strip()
        if result:
            return int(result.split()[0])
    except:
        pass
    return None

def main():
    if len(sys.argv) < 2:
        print("Usage: python3 toggle_detection.py [start|stop|status]")
        sys.exit(1)
    
    command = sys.argv[1].lower()
    
    # Check if pi_controller is running
    pid = find_pi_controller_pid()
    if not pid:
        print("❌ pi_controller.py is not running")
        print("\n💡 Start it first:")
        print("   cd /home/pi/rpi-connect")
        print("   source venv/bin/activate")
        print("   python3 pi_controller.py")
        sys.exit(1)
    
    print(f"✅ Found pi_controller.py (PID: {pid})")
    
    # Use flag files for communication
    flag_dir = "/tmp"
    
    if command == "start":
        # Create flag file to signal start
        flag_file = os.path.join(flag_dir, "pi_detection_start")
        with open(flag_file, 'w') as f:
            f.write(str(time.time()))
        print("\n🌾 Detection START signal sent")
        print("   Watch pi_controller.py terminal for confirmation")
        
    elif command == "stop":
        # Create flag file to signal stop
        flag_file = os.path.join(flag_dir, "pi_detection_stop")
        with open(flag_file, 'w') as f:
            f.write(str(time.time()))
        print("\n🛑 Detection STOP signal sent")
        print("   Watch pi_controller.py terminal for confirmation")
        
    elif command == "status":
        # Check status file
        status_file = os.path.join(flag_dir, "pi_detection_status")
        if os.path.exists(status_file):
            with open(status_file, 'r') as f:
                status = f.read().strip()
            print(f"\n📊 Detection Status: {status}")
        else:
            print("\n📊 Detection Status: Unknown")
            print("   (Status file not created yet)")
    else:
        print(f"❌ Unknown command: {command}")
        print("   Use: start, stop, or status")
        sys.exit(1)

if __name__ == "__main__":
    main()
