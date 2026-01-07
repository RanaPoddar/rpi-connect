#!/usr/bin/env python3
"""
Test utility for yellow crop detection
Helps debug and tune detection parameters
"""

import cv2
import numpy as np
import json
import sys
from modules.yellow_crop_detector import YellowCropDetector

def load_config():
    """Load configuration"""
    try:
        with open('config.json', 'r') as f:
            return json.load(f)
    except Exception as e:
        print(f"Error loading config: {e}")
        return {}

def test_detection_on_image(image_path: str, show_steps: bool = True):
    """
    Test detection on a single image
    
    Args:
        image_path: Path to test image
        show_steps: Show intermediate processing steps
    """
    print(f"Testing detection on: {image_path}")
    
    # Load config and create detector
    config = load_config()
    detector = YellowCropDetector(config=config)
    
    print(f"Detection parameters:")
    print(f"  HSV Lower: {detector.lower_yellow}")
    print(f"  HSV Upper: {detector.upper_yellow}")
    print(f"  Min Area: {detector.min_area}")
    print(f"  Confidence Threshold: {detector.confidence_threshold}")
    print(f"  Adaptive: {detector.adaptive_threshold}")
    
    # Load image
    frame = cv2.imread(image_path)
    if frame is None:
        print(f"Error: Could not load image from {image_path}")
        return
    
    print(f"Image size: {frame.shape}")
    
    # Detect crops
    detections = detector.detect(frame)
    
    print(f"\nDetection results:")
    print(f"  Found {len(detections)} yellow crops")
    
    for i, det in enumerate(detections):
        print(f"  [{i+1}] Area: {det.area:.0f}px, Confidence: {det.confidence:.2f}, "
              f"Bbox: {det.bbox}, Centroid: {det.centroid}")
    
    # Visualize
    if show_steps:
        # Show original
        cv2.namedWindow('1. Original Image', cv2.WINDOW_NORMAL)
        cv2.imshow('1. Original Image', frame)
        
        # Show preprocessed
        preprocessed = detector.preprocess_frame(frame)
        cv2.namedWindow('2. Preprocessed', cv2.WINDOW_NORMAL)
        cv2.imshow('2. Preprocessed', preprocessed)
        
        # Show mask
        mask = detector.create_yellow_mask(preprocessed)
        cv2.namedWindow('3. Yellow Mask', cv2.WINDOW_NORMAL)
        cv2.imshow('3. Yellow Mask', mask)
        
        # Show detections
        result = detector.visualize_detections(frame, detections, show_info=True)
        cv2.namedWindow('4. Detections', cv2.WINDOW_NORMAL)
        cv2.imshow('4. Detections', result)
        
        print("\nPress any key to close windows...")
        cv2.waitKey(0)
        cv2.destroyAllWindows()
    else:
        result = detector.visualize_detections(frame, detections, show_info=True)
        cv2.namedWindow('Detection Result', cv2.WINDOW_NORMAL)
        cv2.imshow('Detection Result', result)
        cv2.waitKey(0)
        cv2.destroyAllWindows()

def test_detection_live():
    """Test detection using camera feed"""
    print("Testing detection with live camera feed")
    
    # Load config and create detector
    config = load_config()
    detector = YellowCropDetector(config=config)
    
    print(f"Detection parameters:")
    print(f"  HSV Lower: {detector.lower_yellow}")
    print(f"  HSV Upper: {detector.upper_yellow}")
    print(f"  Min Area: {detector.min_area}")
    print(f"  Confidence Threshold: {detector.confidence_threshold}")
    
    # Try to open camera
    try:
        from picamera2 import Picamera2
        print("Using Picamera2...")
        camera = Picamera2()
        camera.configure(camera.create_preview_configuration(main={"size": (640, 480)}))
        camera.start()
        use_picamera = True
    except:
        print("Picamera2 not available, using OpenCV camera...")
        camera = cv2.VideoCapture(0)
        if not camera.isOpened():
            print("Error: Could not open camera")
            return
        use_picamera = False
    
    print("\nLive detection active!")
    print("Press 'q' to quit, 's' to show steps, 'd' to toggle debug")
    
    show_steps = False
    
    while True:
        # Capture frame
        if use_picamera:
            frame = camera.capture_array()
            frame = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
        else:
            ret, frame = camera.read()
            if not ret:
                break
        
        # Detect
        detections = detector.detect(frame)
        
        # Visualize
        if show_steps:
            preprocessed = detector.preprocess_frame(frame)
            mask = detector.create_yellow_mask(preprocessed)
            
            cv2.imshow('Mask', mask)
            cv2.imshow('Preprocessed', preprocessed)
        
        result = detector.visualize_detections(frame, detections, show_info=True)
        cv2.imshow('Detection', result)
        
        # Handle keys
        key = cv2.waitKey(1) & 0xFF
        if key == ord('q'):
            break
        elif key == ord('s'):
            show_steps = not show_steps
            print(f"Show steps: {show_steps}")
        elif key == ord('d'):
            detector.debug_mode = not detector.debug_mode
            print(f"Debug mode: {detector.debug_mode}")
    
    # Cleanup
    if use_picamera:
        camera.stop()
    else:
        camera.release()
    cv2.destroyAllWindows()
    
    # Print statistics
    stats = detector.get_statistics()
    print("\nDetection Statistics:")
    for key, value in stats.items():
        print(f"  {key}: {value}")

def interactive_hsv_tuner(image_path: str):
    """Interactive HSV threshold tuner"""
    print("Interactive HSV Tuner")
    print("Adjust trackbars to find optimal yellow detection range")
    
    # Load image
    frame = cv2.imread(image_path)
    if frame is None:
        print(f"Error: Could not load image from {image_path}")
        return
    
    # Create window and trackbars
    window_name = 'HSV Tuner'
    cv2.namedWindow(window_name)
    
    # Initial values from config
    config = load_config()
    detection_config = config.get('detection', {})
    
    lower = detection_config.get('yellow_hsv_lower', [15, 40, 40])
    upper = detection_config.get('yellow_hsv_upper', [40, 255, 255])
    
    cv2.createTrackbar('H Lower', window_name, lower[0], 180, lambda x: None)
    cv2.createTrackbar('S Lower', window_name, lower[1], 255, lambda x: None)
    cv2.createTrackbar('V Lower', window_name, lower[2], 255, lambda x: None)
    cv2.createTrackbar('H Upper', window_name, upper[0], 180, lambda x: None)
    cv2.createTrackbar('S Upper', window_name, upper[1], 255, lambda x: None)
    cv2.createTrackbar('V Upper', window_name, upper[2], 255, lambda x: None)
    
    print("Adjust trackbars, press 's' to save values, 'q' to quit")
    
    while True:
        # Get current trackbar values
        h_lower = cv2.getTrackbarPos('H Lower', window_name)
        s_lower = cv2.getTrackbarPos('S Lower', window_name)
        v_lower = cv2.getTrackbarPos('V Lower', window_name)
        h_upper = cv2.getTrackbarPos('H Upper', window_name)
        s_upper = cv2.getTrackbarPos('S Upper', window_name)
        v_upper = cv2.getTrackbarPos('V Upper', window_name)
        
        # Create mask
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        lower_thresh = np.array([h_lower, s_lower, v_lower])
        upper_thresh = np.array([h_upper, s_upper, v_upper])
        mask = cv2.inRange(hsv, lower_thresh, upper_thresh)
        
        # Apply to original image
        result = cv2.bitwise_and(frame, frame, mask=mask)
        
        # Stack images
        display = np.hstack([frame, cv2.cvtColor(mask, cv2.COLOR_GRAY2BGR), result])
        
        # Add text
        cv2.putText(display, 'Original | Mask | Result', (10, 30),
                   cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
        
        cv2.imshow(window_name, display)
        
        key = cv2.waitKey(1) & 0xFF
        if key == ord('q'):
            break
        elif key == ord('s'):
            print(f"\nCurrent HSV values:")
            print(f"  yellow_hsv_lower: [{h_lower}, {s_lower}, {v_lower}]")
            print(f"  yellow_hsv_upper: [{h_upper}, {s_upper}, {v_upper}]")
            print(f"\nUpdate these values in config.json")
    
    cv2.destroyAllWindows()

if __name__ == "__main__":
    if len(sys.argv) < 2:
        print("Yellow Crop Detection Tester")
        print("\nUsage:")
        print("  python test_yellow_detection.py <mode> [image_path]")
        print("\nModes:")
        print("  image <path>   - Test on single image with step-by-step visualization")
        print("  live           - Test with live camera feed")
        print("  tune <path>    - Interactive HSV threshold tuner")
        print("\nExamples:")
        print("  python test_yellow_detection.py image test.jpg")
        print("  python test_yellow_detection.py live")
        print("  python test_yellow_detection.py tune test.jpg")
        sys.exit(1)
    
    mode = sys.argv[1].lower()
    
    if mode == 'image':
        if len(sys.argv) < 3:
            print("Error: Image path required")
            sys.exit(1)
        test_detection_on_image(sys.argv[2], show_steps=True)
    
    elif mode == 'live':
        test_detection_live()
    
    elif mode == 'tune':
        if len(sys.argv) < 3:
            print("Error: Image path required")
            sys.exit(1)
        interactive_hsv_tuner(sys.argv[2])
    
    else:
        print(f"Error: Unknown mode '{mode}'")
        sys.exit(1)
