"""
Yellow Crop Detection Model for rpi-connect
Specialized model for detecting yellow-pigmented crops using computer vision
Optimized for agricultural drone applications with Raspberry Pi
"""

import cv2
import numpy as np
from typing import List, Dict, Tuple, Optional
from dataclasses import dataclass, asdict
from datetime import datetime
import logging


@dataclass
class CropDetection:
    """Data class for crop detection results"""
    bbox: Tuple[int, int, int, int]  # x, y, width, height
    centroid: Tuple[int, int]  # center point
    area: float  # contour area in pixels
    confidence: float  # detection confidence score
    timestamp: float
    detection_id: str
    
    # GPS data (added when geotagged)
    latitude: Optional[float] = None
    longitude: Optional[float] = None
    altitude: Optional[float] = None
    
    def to_dict(self) -> Dict:
        """Convert to dictionary for transmission"""
        return {
            'detection_id': self.detection_id,
            'timestamp': self.timestamp,
            'datetime': datetime.fromtimestamp(self.timestamp).isoformat(),
            'detection_area': self.area,
            'confidence': self.confidence,
            'bounding_box': {
                'x': self.bbox[0],
                'y': self.bbox[1],
                'width': self.bbox[2],
                'height': self.bbox[3]
            },
            'centroid': {
                'x': self.centroid[0],
                'y': self.centroid[1]
            },
            'latitude': self.latitude,
            'longitude': self.longitude,
            'altitude': self.altitude
        }


class YellowCropDetector:
    """
    Yellow Crop Detection Model
    
    Detects yellow-pigmented crops using HSV color space analysis
    optimized for Raspberry Pi and agricultural drone applications
    """
    
    def __init__(self, config: dict = None, logger: logging.Logger = None):
        """
        Initialize the yellow crop detector
        
        Args:
            config: Configuration dictionary with detection parameters
            logger: Logger instance
        """
        self.config = config.get('detection', {}) if config else {}
        self.logger = logger or logging.getLogger(__name__)
        
        # HSV Color thresholds for yellow detection
        # IMPORTANT: These ranges must be TIGHT to avoid false positives on other colors
        # Hue 20-30 = pure yellow, higher saturation = vivid yellow only
        # Broader HSV range for yellow (to catch more real-world yellow shades)
        # Stricter HSV range for vivid yellow only
        self.lower_yellow = np.array(self.config.get('yellow_hsv_lower', [20, 120, 100]))
        self.upper_yellow = np.array(self.config.get('yellow_hsv_upper', [32, 255, 255]))
        
        # Detection parameters
        self.min_area = self.config.get('min_contour_area', 800)
        self.confidence_threshold = self.config.get('confidence_threshold', 0.8)
        self.adaptive_threshold = self.config.get('adaptive_threshold', True)
        self.debug_mode = self.config.get('debug_mode', True)
        
        # Morphological operations kernels (larger to merge nearby regions)
        self.kernel_open = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (9, 9))
        self.kernel_close = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (35, 35))
        self.kernel_dilate = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (25, 25))
        
        # Detection statistics
        self.total_detections = 0
        self.frame_count = 0
        
        self.logger.info("🌾 Yellow Crop Detector initialized")
        self.logger.info(f"   Mode: Stressed Crop Detection (Competition)")
        self.logger.info(f"   HSV Range: Lower={self.lower_yellow}, Upper={self.upper_yellow}")
        self.logger.info(f"   Min Area: {self.min_area}px, Confidence: {self.confidence_threshold}")
        self.logger.info(f"   Target: Yellow spots/pigmentation on affected crops")
    
    def preprocess_frame(self, frame: np.ndarray) -> np.ndarray:
        """
        Preprocess frame for better yellow detection
        Enhances yellow while suppressing other colors
        
        Args:
            frame: Input BGR frame
            
        Returns:
            Preprocessed frame
        """
        # Apply bilateral filter to reduce noise while preserving edges
        filtered = cv2.bilateralFilter(frame, 9, 75, 75)
        
        # Enhance contrast using CLAHE on L channel
        lab = cv2.cvtColor(filtered, cv2.COLOR_BGR2LAB)
        l, a, b = cv2.split(lab)
        clahe = cv2.createCLAHE(clipLimit=3.0, tileGridSize=(8, 8))
        l = clahe.apply(l)
        enhanced = cv2.merge([l, a, b])
        enhanced = cv2.cvtColor(enhanced, cv2.COLOR_LAB2BGR)
        
        # Convert to HSV and selectively boost saturation ONLY in yellow range
        hsv = cv2.cvtColor(enhanced, cv2.COLOR_BGR2HSV)
        h, s, v = cv2.split(hsv)
        
        # Create mask for yellow hue range (20-30 in HSV)
        hue_mask = cv2.inRange(h, 20, 30)
        
        # Only boost saturation where hue is yellow (preserve other colors)
        s_boosted = np.where(hue_mask > 0, cv2.add(s, 20), s)
        
        # Merge back and convert to BGR
        enhanced_hsv = cv2.merge([h, s_boosted, v])
        enhanced = cv2.cvtColor(enhanced_hsv, cv2.COLOR_HSV2BGR)
        
        return enhanced
    
    def create_yellow_mask(self, frame: np.ndarray) -> np.ndarray:
        """
        Create binary mask for yellow regions (competition: yellow spots on stressed crops)
        
        Args:
            frame: Input BGR frame
            
        Returns:
            Binary mask with yellow regions as white
        """
        # Convert to HSV color space
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        
        # Create primary mask for STRICT yellow color detection only
        # Hue 20-30: Pure yellow range
        # Saturation 90+: Only vivid, saturated yellows (excludes pale/desaturated colors)
        # Value 60+: Bright enough to be visible
        mask = cv2.inRange(hsv, self.lower_yellow, self.upper_yellow)
        
        # Apply aggressive morphological operations to merge nearby regions
        # Opening: removes small noise/artifacts
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, self.kernel_open, iterations=1)
        # Dilation: expand detected regions (merges nearby fragments)
        mask = cv2.dilate(mask, self.kernel_dilate, iterations=4)
        # Closing: fills holes and merges nearby regions (multiple iterations)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, self.kernel_close, iterations=4)
        
        if self.debug_mode:
            total_pixels = mask.size
            white_pixels = np.sum(mask > 0)
            coverage = (white_pixels / total_pixels) * 100
            self.logger.info(f"Mask stats: {white_pixels} yellow pixels ({coverage:.2f}% coverage)")
        
        return mask
    
    def extract_crop_regions(self, mask: np.ndarray) -> List[Tuple]:
        """
        Extract crop regions from binary mask
        
        Args:
            mask: Binary mask
            
        Returns:
            List of (contour, bounding_box, area, centroid) tuples
        """
        # Find contours
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        crop_regions = []
        
        for contour in contours:
            # Calculate area
            area = cv2.contourArea(contour)
            
            # Filter by minimum area
            if area < self.min_area:
                continue
            
            # Get bounding box
            x, y, w, h = cv2.boundingRect(contour)
            
            # Calculate centroid
            M = cv2.moments(contour)
            if M["m00"] != 0:
                cx = int(M["m10"] / M["m00"])
                cy = int(M["m01"] / M["m00"])
            else:
                cx, cy = x + w // 2, y + h // 2
            
            crop_regions.append((contour, (x, y, w, h), area, (cx, cy)))
        
        return crop_regions
    
    def calculate_confidence(self, contour: np.ndarray, area: float, 
                           mask_region: np.ndarray) -> float:
        """
        Calculate confidence score for detection
        
        Args:
            contour: Contour of detected region
            area: Area of contour
            mask_region: Binary mask region
            
        Returns:
            Confidence score (0-1)
        """
        # Factor 1: Compactness (how circular/regular the shape is)
        perimeter = cv2.arcLength(contour, True)
        if perimeter == 0:
            circularity = 0
        else:
            circularity = 4 * np.pi * area / (perimeter * perimeter)
            circularity = min(circularity, 1.0)
        
        # Factor 2: Fill ratio (how much of bounding box is filled)
        x, y, w, h = cv2.boundingRect(contour)
        bbox_area = w * h
        fill_ratio = area / bbox_area if bbox_area > 0 else 0
        
        # Factor 3: Size-based confidence (prefer medium-sized objects, but be generous)
        size_score = min(area / (self.min_area * 5), 1.0)
        
        # Factor 4: Pixel density in mask
        mask_pixels = np.sum(mask_region > 0)
        density = mask_pixels / mask_region.size if mask_region.size > 0 else 0
        
        # Weighted combination (adjusted to be less restrictive)
        confidence = (
            0.15 * circularity +      # Less weight on shape
            0.30 * fill_ratio +       # Moderate weight on fill
            0.25 * size_score +       # Moderate weight on size
            0.30 * density            # Good weight on actual yellow pixels
        )
        
        # Boost confidence if area is substantial
        if area > self.min_area * 3:
            confidence = min(confidence + 0.1, 1.0)
        
        return min(confidence, 1.0)
    
    def detect(self, frame: np.ndarray) -> List[CropDetection]:
        """
        Detect yellow crops in frame
        
        Args:
            frame: Input BGR frame
            
        Returns:
            List of CropDetection objects
        """
        if frame is None or frame.size == 0:
            return []
        
        self.frame_count += 1
        timestamp = datetime.now().timestamp()
        
        # Preprocess frame
        processed_frame = self.preprocess_frame(frame)
        
        # Create yellow mask
        mask = self.create_yellow_mask(processed_frame)
        
        # Extract crop regions
        crop_regions = self.extract_crop_regions(mask)
        
        if self.debug_mode:
            self.logger.info(f"Frame {self.frame_count}: Found {len(crop_regions)} candidate regions")
        
        # Create detection objects
        detections = []
        
        for idx, (contour, bbox, area, centroid) in enumerate(crop_regions):
            x, y, w, h = bbox
            
            # Calculate confidence
            mask_region = mask[y:y+h, x:x+w]
            confidence = self.calculate_confidence(contour, area, mask_region)
            
            # Filter by confidence threshold
            if confidence < self.confidence_threshold:
                if self.debug_mode:
                    self.logger.debug(f"Filtered out: confidence {confidence:.2f} < threshold {self.confidence_threshold}")
                continue
            
            # Create detection object
            detection = CropDetection(
                bbox=bbox,
                centroid=centroid,
                area=area,
                confidence=confidence,
                timestamp=timestamp,
                detection_id=f"DET_{datetime.now().strftime('%Y%m%d_%H%M%S')}_{self.total_detections + idx + 1}"
            )
            
            detections.append(detection)
        
        self.total_detections += len(detections)
        
        return detections
    
    def visualize_detections(self, frame: np.ndarray, 
                           detections: List[CropDetection],
                           show_info: bool = True) -> np.ndarray:
        """
        Draw detections on frame
        
        Args:
            frame: Input frame
            detections: List of detections
            show_info: Whether to show detailed info
            
        Returns:
            Frame with drawn detections
        """
        output = frame.copy()
        
        for detection in detections:
            x, y, w, h = detection.bbox
            cx, cy = detection.centroid
            
            # Color based on confidence (green = high, yellow = medium, orange = low)
            if detection.confidence > 0.85:
                color = (0, 255, 0)  # Green
            elif detection.confidence > 0.75:
                color = (0, 255, 255)  # Yellow
            else:
                color = (0, 165, 255)  # Orange
            
            # Draw bounding box
            cv2.rectangle(output, (x, y), (x + w, y + h), color, 2)
            
            # Draw centroid
            cv2.circle(output, detection.centroid, 5, (0, 0, 255), -1)
            cv2.circle(output, detection.centroid, 8, color, 2)
            
            if show_info:
                # Draw detection info
                info_text = f"Area: {detection.area:.0f}px"
                conf_text = f"Conf: {detection.confidence:.2f}"
                
                cv2.putText(output, info_text, (x, y - 25),
                           cv2.FONT_HERSHEY_SIMPLEX, 0.4, color, 1)
                cv2.putText(output, conf_text, (x, y - 10),
                           cv2.FONT_HERSHEY_SIMPLEX, 0.4, color, 1)
        
        # Draw detection count
        cv2.putText(output, f"Crops Detected: {len(detections)}", (10, 30),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
        
        return output
    
    def get_statistics(self) -> Dict:
        """Get detection statistics"""
        return {
            'total_detections': self.total_detections,
            'frames_processed': self.frame_count,
            'avg_detections_per_frame': self.total_detections / max(self.frame_count, 1),
            'hsv_lower': self.lower_yellow.tolist(),
            'hsv_upper': self.upper_yellow.tolist(),
            'min_area': self.min_area,
            'confidence_threshold': self.confidence_threshold
        }
