#!/usr/bin/env python3
"""
Simple Detection Control for Pi
Run this after pi_controller.py is running to toggle detection
"""

import socketio
import time

# Connect to local pi_controller
sio = socketio.Client()

try:
    print("Connecting to pi_controller...")
    sio.connect('http://localhost:8080')
    print("✅ Connected")
    
    while True:
        print("\n" + "="*50)
        print("Detection Control")
        print("="*50)
        print("1 - Start Detection")
        print("2 - Stop Detection")
        print("0 - Exit")
        print("="*50)
        
        choice = input("Choice: ").strip()
        
        if choice == "1":
            # Manually set detection flag
            print("🌾 Starting detection...")
            # This would require modifying pi_controller to expose this
            print("⚠️  Use Option B below instead")
            
        elif choice == "2":
            print("🛑 Stopping detection...")
            print("⚠️  Use Option B below instead")
            
        elif choice == "0":
            break
            
except Exception as e:
    print(f"❌ Error: {e}")
    print("\n💡 This requires pi_controller to have Socket.IO enabled")
finally:
    sio.disconnect()
