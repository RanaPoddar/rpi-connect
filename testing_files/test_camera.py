#!/usr/bin/env python3
"""
Test script for Raspberry Pi Camera
"""

import sys
import time
from datetime import datetime

try:
    from picamera2 import Picamera2
    import io
    from PIL import Image
    import numpy as np
except ImportError as e:
    print(f"Error: Missing required module - {e}")
    print("Install with: pip install picamera2 pillow numpy")
    sys.exit(1)

def test_camera_basic():
    """Basic camera test"""
    print("=" * 50)
    print("Basic Camera Test")
    print("=" * 50)
    
    try:
        camera = Picamera2()
        print("✓ Camera initialized successfully")
        
        # Test resolution settings
        config = camera.create_preview_configuration(main={"size": (1920, 1080)})
        camera.configure(config)
        print(f"✓ Resolution set to: 1920x1080")
        
        # Camera warm-up
        print("Warming up camera...")
        camera.start()
        time.sleep(2)
        print("✓ Camera ready")
        
        camera.stop()
        camera.close()
        return True
    except Exception as e:
        print(f"✗ Camera test failed: {e}")
        return False

def test_camera_capture():
    """Test image capture"""
    print("\n" + "=" * 50)
    print("Image Capture Test")
    print("=" * 50)
    
    try:
        camera = Picamera2()
        config = camera.create_still_configuration(main={"size": (1920, 1080)})
        camera.configure(config)
        camera.start()
        time.sleep(2)
        
        # Capture to file
        filename = f"test_capture_{datetime.now().strftime('%Y%m%d_%H%M%S')}.jpg"
        camera.capture_file(filename)
        print(f"✓ Image saved to: {filename}")
        
        # Capture to memory as numpy array
        image_array = camera.capture_array()
        image = Image.fromarray(image_array)
        
        print(f"✓ Captured image size: {image.size}")
        print(f"✓ Image format: {image.format if image.format else 'RGB'}")
        
        camera.stop()
        camera.close()
        return True
    except Exception as e:
        print(f"✗ Capture test failed: {e}")
        return False

def test_camera_video():
    """Test video recording"""
    print("\n" + "=" * 50)
    print("Video Recording Test")
    print("=" * 50)
    
    try:
        from picamera2.encoders import H264Encoder
        from picamera2.outputs import FileOutput
        
        camera = Picamera2()
        config = camera.create_video_configuration(
            main={"size": (1280, 720)},
            controls={"FrameRate": 30}
        )
        camera.configure(config)
        
        # Record 5 seconds of video
        filename = f"test_video_{datetime.now().strftime('%Y%m%d_%H%M%S')}.h264"
        print(f"Recording 5 seconds to: {filename}")
        
        encoder = H264Encoder()
        camera.start_recording(encoder, filename)
        time.sleep(5)
        camera.stop_recording()
        print(f"✓ Video saved to: {filename}")
        
        camera.close()
        return True
    except Exception as e:
        print(f"✗ Video test failed: {e}")
        return False

def test_camera_stream():
    """Test streaming capture"""
    print("\n" + "=" * 50)
    print("Streaming Test (10 frames)")
    print("=" * 50)
    
    try:
        camera = Picamera2()
        config = camera.create_video_configuration(
            main={"size": (640, 480)},
            controls={"FrameRate": 30}
        )
        camera.configure(config)
        camera.start()
        time.sleep(2)
        
        frame_count = 0
        start_time = time.time()
        
        while frame_count < 10:
            frame = camera.capture_array()
            frame_count += 1
            print(f"✓ Captured frame {frame_count} (shape: {frame.shape})")
        
        elapsed = time.time() - start_time
        fps = frame_count / elapsed
        print(f"✓ Captured {frame_count} frames in {elapsed:.2f}s ({fps:.1f} fps)")
        
        camera.stop()
        camera.close()
        return True
    except Exception as e:
        print(f"✗ Streaming test failed: {e}")
        return False

def main():
    """Run all tests"""
    print("\nRaspberry Pi Camera Test Suite")
    print("=" * 50)
    print(f"Test started at: {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}")
    
    tests = [
        ("Basic Camera", test_camera_basic),
        ("Image Capture", test_camera_capture),
        ("Video Recording", test_camera_video),
        ("Streaming", test_camera_stream)
    ]
    
    results = []
    for test_name, test_func in tests:
        try:
            result = test_func()
            results.append((test_name, result))
        except KeyboardInterrupt:
            print("\n\nTest interrupted by user")
            break
        except Exception as e:
            print(f"\n✗ Unexpected error in {test_name}: {e}")
            results.append((test_name, False))
    
    # Summary
    print("\n" + "=" * 50)
    print("Test Summary")
    print("=" * 50)
    for test_name, result in results:
        status = "✓ PASS" if result else "✗ FAIL"
        print(f"{test_name}: {status}")
    
    passed = sum(1 for _, r in results if r)
    total = len(results)
    print(f"\nTotal: {passed}/{total} tests passed")
    
    if passed == total:
        print("\n All tests passed!")
        return 0
    else:
        print(f"\n  {total - passed} test(s) failed")
        return 1

if __name__ == '__main__':
    sys.exit(main())
