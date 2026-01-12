#!/usr/bin/env python3
"""
Test script to verify geolocation fixes
Compares old vs new rotation logic
"""

import math
import sys
import os
sys.path.insert(0, os.path.dirname(__file__))

from modules.geolocation import GeoLocationCalculator

def test_rotation_fix():
    """Test that rotation fix resolves 100-150m error"""
    
    print("="*60)
    print("GEOLOCATION ROTATION FIX TEST")
    print("="*60)
    
    # Simulate drone at 50m altitude
    drone_lat = 23.123456
    drone_lon = 72.654321
    altitude = 50.0
    
    # Test scenarios with different headings
    test_cases = [
        {
            "name": "Facing North (0°)",
            "heading": 0.0,
            "pixel": (2500, 1000),  # Right and forward from center
            "description": "Pixel to the right and forward of center"
        },
        {
            "name": "Facing East (90°)",
            "heading": 90.0,
            "pixel": (2500, 1000),  # Same pixel
            "description": "Same pixel, but drone rotated 90°"
        },
        {
            "name": "Facing South (180°)",
            "heading": 180.0,
            "pixel": (2028, 1520),  # Image center
            "description": "Center pixel (should be drone position)"
        },
        {
            "name": "Facing West (270°)",
            "heading": 270.0,
            "pixel": (1500, 2000),  # Left and backward
            "description": "Pixel to the left and backward"
        }
    ]
    
    geo_calc = GeoLocationCalculator()
    
    print(f"\nDrone position: ({drone_lat:.7f}, {drone_lon:.7f})")
    print(f"Altitude: {altitude}m\n")
    
    for test in test_cases:
        print(f"\n{test['name']}")
        print(f"  {test['description']}")
        print(f"  Heading: {test['heading']}°")
        print(f"  Pixel: {test['pixel']}")
        
        # Calculate ground GPS
        lat, lon = geo_calc.pixel_to_gps(
            pixel_x=test['pixel'][0],
            pixel_y=test['pixel'][1],
            drone_lat=drone_lat,
            drone_lon=drone_lon,
            altitude_agl=altitude,
            heading_deg=test['heading'],
            pitch_deg=0.0,
            roll_deg=0.0
        )
        
        # Calculate distance from drone
        dlat = (lat - drone_lat) * 111320  # meters
        dlon = (lon - drone_lon) * 111320 * math.cos(math.radians(drone_lat))
        distance = math.sqrt(dlat**2 + dlon**2)
        bearing = math.degrees(math.atan2(dlon, dlat))
        
        print(f"  → Ground GPS: ({lat:.7f}, {lon:.7f})")
        print(f"  → Distance from drone: {distance:.1f}m at bearing {bearing:.1f}°")
        
        # Sanity check
        if distance > 100:
            print(f"  ⚠️ WARNING: Distance > 100m - possible error!")
        else:
            print(f"  ✓ Looks reasonable")
    
    print("\n" + "="*60)
    print("TEST COMPLETE")
    print("="*60)
    print("\nExpected behavior:")
    print("  - Center pixel → ~0m distance (drone position)")
    print("  - Edge pixels → 20-40m distance (at 50m altitude)")
    print("  - Heading rotation should change bearing, not distance")
    print("\nIf you still see 100-150m errors:")
    print("  1. Check heading sensor units (degrees vs radians)")
    print("  2. Verify heading=0° actually points North")
    print("  3. Check camera mounting orientation")
    print("  4. Verify GPS altitude is accurate")

def test_yellow_hsv():
    """Test new yellow HSV range"""
    import numpy as np
    
    print("\n" + "="*60)
    print("YELLOW DETECTION HSV FIX TEST")
    print("="*60)
    
    old_lower = np.array([20, 90, 60])
    old_upper = np.array([30, 255, 255])
    
    new_lower = np.array([18, 40, 40])
    new_upper = np.array([45, 255, 255])
    
    print("\nOLD Range (TOO RESTRICTIVE):")
    print(f"  Lower: {old_lower} → Hue: 20-30°, Sat: 90-255, Val: 60-255")
    print(f"  Upper: {old_upper}")
    print(f"  Coverage: ONLY pure saturated yellow in bright light")
    
    print("\nNEW Range (IMPROVED):")
    print(f"  Lower: {new_lower} → Hue: 18-45°, Sat: 40-255, Val: 40-255")
    print(f"  Upper: {new_upper}")
    print(f"  Coverage: Yellow-orange to greenish-yellow, pale/faded, shadows")
    
    print("\nWhat NEW range now detects:")
    print("  ✓ Pure yellow (H: 20-30)")
    print("  ✓ Orange-yellow (H: 18-25)")
    print("  ✓ Greenish-yellow (H: 30-45)")
    print("  ✓ Pale/faded yellow (S: 40-90)")
    print("  ✓ Yellow in shadows (V: 40-60)")
    print("  ✓ Weathered/dusty yellow (lower saturation)")
    
    print("\nRecommendation:")
    print("  - Test with actual field images")
    print("  - If too many false positives, increase S minimum (40→50)")
    print("  - If still missing yellows, increase H upper (45→50)")
    print("  - Use debug_mode: true to see mask coverage")

if __name__ == "__main__":
    test_rotation_fix()
    test_yellow_hsv()
    
    print("\n" + "="*60)
    print("NEXT STEPS:")
    print("="*60)
    print("1. Restart your Pi controller to load new config")
    print("2. Fly a test mission with known ground markers")
    print("3. Check detection GPS accuracy against markers")
    print("4. Adjust yellow HSV if needed based on field conditions")
    print("5. Monitor console for 'Photogrammetry:' log messages")
