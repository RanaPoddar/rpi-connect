#!/usr/bin/env python3
"""
Laptop-friendly Yellow Crop Detection Tester
Works without Raspberry Pi hardware - uses webcam or image files
"""

import cv2
import numpy as np
import json
import sys
import os

# Simplified detector that doesn't require Pi imports
class SimpleYellowDetector:
    def __init__(self, config):
        detection_config = config.get('detection', {})
        
        # STRICT yellow-only detection (updated defaults)
        self.lower_yellow = np.array(detection_config.get('yellow_hsv_lower', [20, 90, 60]))
        self.upper_yellow = np.array(detection_config.get('yellow_hsv_upper', [30, 255, 255]))
        self.min_area = detection_config.get('min_contour_area', 300)
        self.confidence_threshold = detection_config.get('confidence_threshold', 0.5)
        self.adaptive_threshold = detection_config.get('adaptive_threshold', False)
        
        # Morphological kernels (larger for merging nearby regions)
        self.kernel_open = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))
        self.kernel_close = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (15, 15))
        self.kernel_dilate = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (7, 7))
        
        self.detection_count = 0
        
        print(f"✅ Detector initialized:")
        print(f"   HSV Range: {self.lower_yellow} to {self.upper_yellow}")
        print(f"   Min Area: {self.min_area}px")
        print(f"   Confidence: {self.confidence_threshold}")
        print(f"   Adaptive: {self.adaptive_threshold}")
    
    def preprocess_frame(self, frame):
        """Enhance frame for better yellow detection (selective boost)"""
        # Bilateral filter
        filtered = cv2.bilateralFilter(frame, 9, 75, 75)
        
        # CLAHE contrast enhancement
        lab = cv2.cvtColor(filtered, cv2.COLOR_BGR2LAB)
        l, a, b = cv2.split(lab)
        clahe = cv2.createCLAHE(clipLimit=3.0, tileGridSize=(8, 8))
        l = clahe.apply(l)
        enhanced = cv2.merge([l, a, b])
        enhanced = cv2.cvtColor(enhanced, cv2.COLOR_LAB2BGR)
        
        # Selective saturation boost ONLY in yellow hue range
        hsv = cv2.cvtColor(enhanced, cv2.COLOR_BGR2HSV)
        h, s, v = cv2.split(hsv)
        hue_mask = cv2.inRange(h, 20, 30)  # Only yellow hues (binary mask)
        s_boosted = s.copy()
        # Apply boost where mask is non-zero
        s_boosted = np.where(hue_mask > 0, cv2.add(s, 20), s)
        enhanced_hsv = cv2.merge([h, s_boosted, v])
        enhanced = cv2.cvtColor(enhanced_hsv, cv2.COLOR_HSV2BGR)
        
        return enhanced
    
    def create_yellow_mask(self, frame):
        """Create binary mask for STRICT yellow-only regions"""
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        
        # Single strict mask for pure yellow only
        # H: 20-30 (pure yellow)
        # S: 90+ (vivid saturation, rejects pale colors)
        # V: 60+ (bright enough to be visible)
        mask = cv2.inRange(hsv, self.lower_yellow, self.upper_yellow)
        
        # Aggressive morphological operations to merge nearby regions
        # Opening: remove noise
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, self.kernel_open, iterations=1)
        # Dilation: expand regions
        mask = cv2.dilate(mask, self.kernel_dilate, iterations=2)
        # Closing: merge nearby regions (multiple iterations for larger sheets)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, self.kernel_close, iterations=2)
        
        return mask
    
    def detect(self, frame):
        """Detect yellow crops"""
        preprocessed = self.preprocess_frame(frame)
        mask = self.create_yellow_mask(preprocessed)
        
        # Find contours
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        detections = []
        for contour in contours:
            area = cv2.contourArea(contour)
            
            if area < self.min_area:
                continue
            
            x, y, w, h = cv2.boundingRect(contour)
            
            # Calculate confidence
            bbox_area = w * h
            fill_ratio = area / bbox_area if bbox_area > 0 else 0
            size_score = min(area / (self.min_area * 5), 1.0)
            
            mask_region = mask[y:y+h, x:x+w]
            density = np.sum(mask_region > 0) / mask_region.size if mask_region.size > 0 else 0
            
            confidence = 0.3 * fill_ratio + 0.3 * size_score + 0.4 * density
            
            if area > self.min_area * 3:
                confidence = min(confidence + 0.1, 1.0)
            
            if confidence < self.confidence_threshold:
                continue
            
            # Centroid
            M = cv2.moments(contour)
            if M["m00"] != 0:
                cx = int(M["m10"] / M["m00"])
                cy = int(M["m01"] / M["m00"])
            else:
                cx, cy = x + w // 2, y + h // 2
            
            detections.append({
                'bbox': (x, y, w, h),
                'centroid': (cx, cy),
                'area': area,
                'confidence': confidence
            })
            
            self.detection_count += 1
        
        return detections, mask
    
    def visualize(self, frame, detections):
        """Draw detections on frame"""
        output = frame.copy()
        
        for det in detections:
            x, y, w, h = det['bbox']
            cx, cy = det['centroid']
            confidence = det['confidence']
            area = det['area']
            
            # Color based on confidence
            if confidence > 0.85:
                color = (0, 255, 0)  # Green
            elif confidence > 0.75:
                color = (0, 255, 255)  # Yellow
            else:
                color = (0, 165, 255)  # Orange
            
            cv2.rectangle(output, (x, y), (x + w, y + h), color, 2)
            cv2.circle(output, (cx, cy), 5, (0, 0, 255), -1)
            cv2.circle(output, (cx, cy), 8, color, 2)
            
            cv2.putText(output, f"Area: {area:.0f}px", (x, y - 25),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.4, color, 1)
            cv2.putText(output, f"Conf: {confidence:.2f}", (x, y - 10),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.4, color, 1)
        
        cv2.putText(output, f"Detected: {len(detections)}", (10, 30),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
        
        return output

def load_config():
    """Load configuration from config.json"""
    try:
        config_path = 'config.json'
        if not os.path.exists(config_path):
            # Try parent directory
            config_path = '../config.json'
        
        with open(config_path, 'r') as f:
            return json.load(f)
    except Exception as e:
        print(f"⚠️  Could not load config.json: {e}")
        print("Using default parameters...")
        return {}

def test_on_image(image_path, show_steps=True):
    """Test detection on a single image"""
    print(f"\n🖼️  Testing on image: {image_path}")
    
    if not os.path.exists(image_path):
        print(f"❌ Error: Image not found: {image_path}")
        return
    
    config = load_config()
    detector = SimpleYellowDetector(config)
    
    # Load image
    frame = cv2.imread(image_path)
    if frame is None:
        print(f"❌ Error: Could not load image")
        return
    
    print(f"✅ Image loaded: {frame.shape}")
    
    # Detect
    detections, mask = detector.detect(frame)
    
    print(f"\n📊 Results:")
    print(f"   Found {len(detections)} yellow crops")
    
    for i, det in enumerate(detections):
        print(f"   [{i+1}] Area: {det['area']:.0f}px, "
              f"Confidence: {det['confidence']:.2f}, "
              f"Position: {det['centroid']}")
    
    # Visualize
    if show_steps:
        # Original
        cv2.imshow('1. Original', cv2.resize(frame, (800, 600)))
        
        # Preprocessed
        preprocessed = detector.preprocess_frame(frame)
        cv2.imshow('2. Preprocessed', cv2.resize(preprocessed, (800, 600)))
        
        # Mask
        cv2.imshow('3. Yellow Mask', cv2.resize(mask, (800, 600)))
        
        # Detections
        result = detector.visualize(frame, detections)
        cv2.imshow('4. Detections', cv2.resize(result, (800, 600)))
        
        print("\n👁️  Showing all processing steps")
        print("Press any key to continue...")
        cv2.waitKey(0)
    else:
        result = detector.visualize(frame, detections)
        cv2.imshow('Detection Result', cv2.resize(result, (800, 600)))
        print("\nPress any key to close...")
        cv2.waitKey(0)
    
    cv2.destroyAllWindows()

def test_webcam():
    """Test detection using laptop webcam"""
    print(f"\n📹 Starting webcam test...")
    
    config = load_config()
    detector = SimpleYellowDetector(config)
    
    # Open webcam
    cap = cv2.VideoCapture(0)
    
    if not cap.isOpened():
        print("❌ Error: Could not open webcam")
        return
    
    print("✅ Webcam opened")
    print("\n🎮 Controls:")
    print("   'q' - Quit")
    print("   's' - Toggle processing steps")
    print("   'c' - Capture screenshot")
    print("   'h' - Toggle HSV tuning window")
    
    show_steps = False
    show_hsv = False
    
    while True:
        ret, frame = cap.read()
        if not ret:
            print("❌ Failed to grab frame")
            break
        
        # Detect
        detections, mask = detector.detect(frame)
        result = detector.visualize(frame, detections)
        
        # Show result
        cv2.imshow('Detection', result)
        
        # Show steps if enabled
        if show_steps:
            cv2.imshow('Mask', mask)
            preprocessed = detector.preprocess_frame(frame)
            cv2.imshow('Preprocessed', preprocessed)
        else:
            cv2.destroyWindow('Mask')
            cv2.destroyWindow('Preprocessed')
        
        # Show HSV values if enabled
        if show_hsv:
            hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
            h, s, v = cv2.split(hsv)
            cv2.imshow('H (Hue)', h)
            cv2.imshow('S (Saturation)', s)
            cv2.imshow('V (Value)', v)
        else:
            cv2.destroyWindow('H (Hue)')
            cv2.destroyWindow('S (Saturation)')
            cv2.destroyWindow('V (Value)')
        
        # Handle keys
        key = cv2.waitKey(1) & 0xFF
        if key == ord('q'):
            break
        elif key == ord('s'):
            show_steps = not show_steps
            print(f"Processing steps: {'ON' if show_steps else 'OFF'}")
        elif key == ord('h'):
            show_hsv = not show_hsv
            print(f"HSV view: {'ON' if show_hsv else 'OFF'}")
        elif key == ord('c'):
            filename = f"capture_{detector.detection_count}.jpg"
            cv2.imwrite(filename, result)
            print(f"📸 Saved: {filename}")
    
    cap.release()
    cv2.destroyAllWindows()
    
    print(f"\n📊 Total detections: {detector.detection_count}")

def interactive_hsv_tuner(image_path):
    """Interactive HSV threshold tuner with real-time feedback"""
    print(f"\n🎨 Interactive HSV Tuner (Enhanced)")
    
    if not os.path.exists(image_path):
        print(f"❌ Error: Image not found: {image_path}")
        return
    
    frame = cv2.imread(image_path)
    if frame is None:
        print(f"❌ Error: Could not load image")
        return
    
    # Resize if too large
    h, w = frame.shape[:2]
    max_width = 600
    if w > max_width:
        scale = max_width / w
        frame = cv2.resize(frame, None, fx=scale, fy=scale)
    
    print(f"✅ Image loaded: {frame.shape}")
    
    # Get average HSV values from image for hints
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    h_mean, s_mean, v_mean = cv2.mean(hsv)[:3]
    print(f"📊 Average HSV in image: H={h_mean:.0f}, S={s_mean:.0f}, V={v_mean:.0f}")
    
    config = load_config()
    detection_config = config.get('detection', {})
    
    lower = detection_config.get('yellow_hsv_lower', [15, 30, 30])
    upper = detection_config.get('yellow_hsv_upper', [45, 255, 255])
    
    window_name = 'HSV Tuner - Press S to save, Q to quit, SPACE for presets'
    cv2.namedWindow(window_name, cv2.WINDOW_NORMAL)
    cv2.resizeWindow(window_name, 1400, 500)
    
    cv2.createTrackbar('H Lower', window_name, lower[0], 180, lambda x: None)
    cv2.createTrackbar('S Lower', window_name, lower[1], 255, lambda x: None)
    cv2.createTrackbar('V Lower', window_name, lower[2], 255, lambda x: None)
    cv2.createTrackbar('H Upper', window_name, upper[0], 180, lambda x: None)
    cv2.createTrackbar('S Upper', window_name, upper[1], 255, lambda x: None)
    cv2.createTrackbar('V Upper', window_name, upper[2], 255, lambda x: None)
    cv2.createTrackbar('Min Area', window_name, 300, 2000, lambda x: None)
    
    print("\n🎮 Controls:")
    print("   's' - Save/print current values")
    print("   'q' - Quit")
    print("   '1' - Preset: Bright Yellow")
    print("   '2' - Preset: Yellow-Green")
    print("   '3' - Preset: Dark Yellow")
    print("   '4' - Preset: Wide Range (catch all)")
    print("   'r' - Reset to config values")
    print("   'SPACE' - Cycle through presets")
    
    preset_index = 0
    presets = [
        {"name": "Bright Yellow", "lower": [20, 100, 100], "upper": [35, 255, 255]},
        {"name": "Yellow-Green", "lower": [15, 50, 50], "upper": [40, 255, 255]},
        {"name": "Dark Yellow", "lower": [20, 50, 30], "upper": [40, 255, 200]},
        {"name": "Wide Range", "lower": [10, 30, 30], "upper": [50, 255, 255]},
        {"name": "Very Wide", "lower": [10, 20, 20], "upper": [55, 255, 255]},
    ]
    
    while True:
        h_lower = cv2.getTrackbarPos('H Lower', window_name)
        s_lower = cv2.getTrackbarPos('S Lower', window_name)
        v_lower = cv2.getTrackbarPos('V Lower', window_name)
        h_upper = cv2.getTrackbarPos('H Upper', window_name)
        s_upper = cv2.getTrackbarPos('S Upper', window_name)
        v_upper = cv2.getTrackbarPos('V Upper', window_name)
        min_area = cv2.getTrackbarPos('Min Area', window_name)
        
        # Create mask
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        lower_thresh = np.array([h_lower, s_lower, v_lower])
        upper_thresh = np.array([h_upper, s_upper, v_upper])
        mask = cv2.inRange(hsv, lower_thresh, upper_thresh)
        
        # Apply morphology
        kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
        
        # Find contours
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        # Draw on copy
        result = frame.copy()
        count = 0
        total_area = 0
        for contour in contours:
            area = cv2.contourArea(contour)
            if area >= min_area:
                x, y, w, h = cv2.boundingRect(contour)
                cv2.rectangle(result, (x, y), (x+w, y+h), (0, 255, 0), 2)
                cv2.putText(result, f"{int(area)}", (x, y-5), 
                           cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 255, 0), 1)
                count += 1
                total_area += area
        
        # Create HSV visualization
        h, s, v = cv2.split(hsv)
        h_colored = cv2.applyColorMap(h, cv2.COLORMAP_HSV)
        s_colored = cv2.applyColorMap(s, cv2.COLORMAP_BONE)
        v_colored = cv2.applyColorMap(v, cv2.COLORMAP_BONE)
        
        # Info overlay on result
        info_lines = [
            f"HSV: [{h_lower},{s_lower},{v_lower}] to [{h_upper},{s_upper},{v_upper}]",
            f"Detections: {count} | Total Area: {int(total_area)}px",
            f"Min Area: {min_area}px | Coverage: {(np.sum(mask>0)/mask.size)*100:.1f}%"
        ]
        
        y_pos = 25
        for line in info_lines:
            cv2.putText(result, line, (10, y_pos), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)
            y_pos += 20
        
        # Stack images - two rows
        row1 = np.hstack([frame, cv2.cvtColor(mask, cv2.COLOR_GRAY2BGR), result])
        row2 = np.hstack([h_colored, s_colored, v_colored])
        
        # Add labels
        cv2.putText(row1, "Original", (10, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
        cv2.putText(row1, "Mask", (frame.shape[1]+10, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
        cv2.putText(row1, "Result", (frame.shape[1]*2+10, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
        cv2.putText(row2, "H (Hue)", (10, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
        cv2.putText(row2, "S (Saturation)", (frame.shape[1]+10, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
        cv2.putText(row2, "V (Value)", (frame.shape[1]*2+10, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
        
        display = np.vstack([row1, row2])
        
        cv2.imshow(window_name, display)
        
        key = cv2.waitKey(1) & 0xFF
        if key == ord('q'):
            break
        elif key == ord('s'):
            print(f"\n📋 Current HSV values:")
            print(f'   "yellow_hsv_lower": [{h_lower}, {s_lower}, {v_lower}],')
            print(f'   "yellow_hsv_upper": [{h_upper}, {s_upper}, {v_upper}],')
            print(f'   "min_contour_area": {min_area},')
            print(f"\n✏️  Copy these to config.json")
            print(f"📊 Detection stats: {count} objects, {int(total_area)}px total area")
        elif key == ord(' ') or key in [ord('1'), ord('2'), ord('3'), ord('4')]:
            if key == ord(' '):
                preset_index = (preset_index + 1) % len(presets)
            else:
                preset_index = int(chr(key)) - 1
            
            preset = presets[preset_index]
            print(f"\n🎨 Applied preset: {preset['name']}")
            cv2.setTrackbarPos('H Lower', window_name, preset['lower'][0])
            cv2.setTrackbarPos('S Lower', window_name, preset['lower'][1])
            cv2.setTrackbarPos('V Lower', window_name, preset['lower'][2])
            cv2.setTrackbarPos('H Upper', window_name, preset['upper'][0])
            cv2.setTrackbarPos('S Upper', window_name, preset['upper'][1])
            cv2.setTrackbarPos('V Upper', window_name, preset['upper'][2])
        elif key == ord('r'):
            print("\n🔄 Reset to config values")
            cv2.setTrackbarPos('H Lower', window_name, lower[0])
            cv2.setTrackbarPos('S Lower', window_name, lower[1])
            cv2.setTrackbarPos('V Lower', window_name, lower[2])
            cv2.setTrackbarPos('H Upper', window_name, upper[0])
            cv2.setTrackbarPos('S Upper', window_name, upper[1])
            cv2.setTrackbarPos('V Upper', window_name, upper[2])
    
    cv2.destroyAllWindows()

if __name__ == "__main__":
    print("=" * 60)
    print("🌾 Yellow Crop Detection Tester (Laptop Edition)")
    print("=" * 60)
    
    if len(sys.argv) < 2:
        print("\nUsage:")
        print("  python test_detection_laptop.py <mode> [image_path]")
        print("\nModes:")
        print("  image <path>  - Test on single image")
        print("  webcam        - Test with laptop webcam")
        print("  tune <path>   - Interactive HSV tuner")
        print("\nExamples:")
        print("  python test_detection_laptop.py image test.jpg")
        print("  python test_detection_laptop.py webcam")
        print("  python test_detection_laptop.py tune test.jpg")
        sys.exit(0)
    
    mode = sys.argv[1].lower()
    
    if mode == 'image':
        if len(sys.argv) < 3:
            print("❌ Error: Image path required")
            print("Usage: python test_detection_laptop.py image <path>")
            sys.exit(1)
        test_on_image(sys.argv[2], show_steps=True)
    
    elif mode == 'webcam':
        test_webcam()
    
    elif mode == 'tune':
        if len(sys.argv) < 3:
            print("❌ Error: Image path required")
            print("Usage: python test_detection_laptop.py tune <path>")
            sys.exit(1)
        interactive_hsv_tuner(sys.argv[2])
    
    else:
        print(f"❌ Error: Unknown mode '{mode}'")
        print("Valid modes: image, webcam, tune")
        sys.exit(1)
