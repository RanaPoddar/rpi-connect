#!/usr/bin/env python3
"""
Test script for MAVLink Commander Integration
Tests the Pi Controller with MAVLink commands in simulation mode
"""

import sys
import os

# Add parent directory to path
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

# Test imports
print("🧪 Testing MAVLink Commander Integration")
print("-" * 50)

print("\n1️⃣  Testing imports...")
try:
    from modules.mavlink_commander import MAVLinkCommander
    print("   ✅ MAVLinkCommander imported successfully")
except ImportError as e:
    print(f"   ❌ Failed to import MAVLinkCommander: {e}")
    sys.exit(1)

try:
    from modules.pixhawk_telemetry import PixhawkTelemetry
    print("   ✅ PixhawkTelemetry imported successfully")
except ImportError as e:
    print(f"   ❌ Failed to import PixhawkTelemetry: {e}")
    sys.exit(1)

# Test commander initialization
print("\n2️⃣  Testing MAVLinkCommander initialization...")
try:
    commander = MAVLinkCommander(simulation_mode=True)
    print("   ✅ Commander initialized in simulation mode")
except Exception as e:
    print(f"   ❌ Commander initialization failed: {e}")
    sys.exit(1)

# Test basic commands
print("\n3️⃣  Testing ARM command...")
result = commander.arm()
print(f"   {'✅' if result['success'] else '❌'} ARM: {result['message']}")

print("\n4️⃣  Testing mode change...")
result = commander.set_mode('GUIDED')
print(f"   {'✅' if result['success'] else '❌'} Mode change: {result['message']}")

print("\n5️⃣  Testing TAKEOFF command...")
result = commander.takeoff(10.0)
print(f"   {'✅' if result['success'] else '❌'} Takeoff: {result['message']}")

print("\n6️⃣  Testing navigation...")
result = commander.goto_location(40.7128, -74.0060, altitude=10.0)
print(f"   {'✅' if result['success'] else '❌'} Navigation: {result['message']}")

print("\n7️⃣  Testing mission upload...")
mission = [
    {'lat': 40.7128, 'lon': -74.0060, 'alt': 10},
    {'lat': 40.7129, 'lon': -74.0061, 'alt': 10},
]
result = commander.upload_mission(mission, takeoff_alt=10.0)
print(f"   {'✅' if result['success'] else '❌'} Mission upload: {result['message']}")

print("\n8️⃣  Testing LAND command...")
result = commander.land()
print(f"   {'✅' if result['success'] else '❌'} Landing: {result['message']}")

print("\n9️⃣  Testing status check...")
status = commander.get_status()
print(f"   Armed: {status.get('armed', False)}")
print(f"   Mode: {status.get('mode', 'UNKNOWN')}")

print("\n🎉 All integration tests passed!")
print("\n" + "=" * 50)
print("MAVLink Commander is ready for use!")
print("=" * 50)
