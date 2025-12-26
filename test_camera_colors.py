#!/usr/bin/env python3
"""
Camera Color Test - Captures 4 images with different white balance settings
Run this to find the best color profile for your camera
"""

import time
from picamera2 import Picamera2
from datetime import datetime
import os

def test_camera_colors():
    """Capture images with different white balance settings"""
    
    print("=" * 60)
    print("Camera Color Test - Testing 4 White Balance Modes")
    print("=" * 60)
    
    # Create output directory
    output_dir = "camera_color_tests"
    os.makedirs(output_dir, exist_ok=True)
    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    
    # Define test configurations
    test_configs = [
        {
            "name": "1_AUTO",
            "description": "Auto White Balance (adapts to lighting)",
            "controls": {
                "AwbEnable": True,
                "AwbMode": 0,  # Auto
                "Saturation": 1.0,
                "Contrast": 1.0,
                "Brightness": 0.0,
                "Sharpness": 1.0
            }
        },
        {
            "name": "2_DAYLIGHT",
            "description": "Daylight Mode (cooler, outdoor)",
            "controls": {
                "AwbEnable": True,
                "AwbMode": 4,  # Daylight
                "Saturation": 1.0,
                "Contrast": 1.0,
                "Brightness": 0.0,
                "Sharpness": 1.0
            }
        },
        {
            "name": "3_TUNGSTEN",
            "description": "Tungsten Mode (warm indoor/incandescent)",
            "controls": {
                "AwbEnable": True,
                "AwbMode": 1,  # Tungsten
                "Saturation": 1.0,
                "Contrast": 1.0,
                "Brightness": 0.0,
                "Sharpness": 1.0
            }
        },
        {
            "name": "4_CLOUDY",
            "description": "Cloudy Mode (very cool, overcast)",
            "controls": {
                "AwbEnable": True,
                "AwbMode": 5,  # Cloudy
                "Saturation": 1.0,
                "Contrast": 1.0,
                "Brightness": 0.0,
                "Sharpness": 1.0
            }
        }
    ]
    
    camera = Picamera2()
    
    print("\nCapturing test images...\n")
    
    for idx, test_config in enumerate(test_configs, 1):
        try:
            print(f"[{idx}/4] Testing: {test_config['name']}")
            print(f"      Description: {test_config['description']}")
            
            # Configure camera with test settings
            config = camera.create_still_configuration(
                main={"size": (1920, 1080)},
                controls=test_config['controls']
            )
            camera.configure(config)
            camera.start()
            
            # Wait for camera to adjust
            print("      Warming up camera...")
            time.sleep(3)
            
            # Capture image
            filename = f"{output_dir}/test_{timestamp}_{test_config['name']}.jpg"
            camera.capture_file(filename)
            print(f"      ✓ Saved: {filename}\n")
            
            camera.stop()
            
        except Exception as e:
            print(f"      ✗ Error: {e}\n")
            camera.stop()
            continue
    
    camera.close()
    
    print("=" * 60)
    print("Test Complete!")
    print("=" * 60)
    print(f"\nImages saved in: {output_dir}/")
    print("\nCompare the images and note which mode looks best:")
    print("  1_AUTO     - Auto white balance")
    print("  2_DAYLIGHT - Cooler colors (outdoor)")
    print("  3_TUNGSTEN - Warmer colors (indoor)")
    print("  4_CLOUDY   - Very cool colors (overcast)")
    print("\nTell me which number (1-4) looks most natural!")
    print("=" * 60)

if __name__ == "__main__":
    try:
        test_camera_colors()
    except KeyboardInterrupt:
        print("\n\nTest cancelled by user")
    except Exception as e:
        print(f"\n\nError: {e}")
