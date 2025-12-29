"""
Geolocation Module - Convert pixel coordinates to GPS coordinates
Implements photogrammetry calculations for accurate ground coordinate mapping
"""

import math
from typing import Tuple, Optional


class GeoLocationCalculator:
    """
    Calculate ground GPS coordinates from camera pixel coordinates
    using photogrammetry and drone telemetry
    """
    
    # Camera parameters for Raspberry Pi HQ Camera + 6mm lens widr angle
    CAMERA_PARAMS = {
        'sensor_width_mm': 7.9,       # Sony IMX477 sensor width in mm
        'sensor_height_mm': 6.0,      # Sony IMX477 sensor height in mm
        'focal_length_mm': 6.0,       # 6mm ultra wide lens
        'image_width_px': 4056,       # Image width in pixels (12.3 MP)
        'image_height_px': 3040,      # Image height in pixels
        'fov_horizontal_deg': 66.7,   # Calculated: 2*arctan(7.9/(2*6))
        'fov_vertical_deg': 53.1      # Calculated: 2*arctan(6.0/(2*6))
    }
    
    def __init__(self, camera_params: dict = None):
        """
        Initialize geolocation calculator
        
        Args:
            camera_params: Optional custom camera parameters
        """
        self.params = camera_params or self.CAMERA_PARAMS
        
        # Pre-calculate FOV in radians
        self.fov_h_rad = math.radians(self.params['fov_horizontal_deg'])
        self.fov_v_rad = math.radians(self.params['fov_vertical_deg'])
    
    def pixel_to_gps(
        self,
        pixel_x: float,
        pixel_y: float,
        drone_lat: float,
        drone_lon: float,
        altitude_agl: float,
        heading_deg: float,
        pitch_deg: float = 0.0,
        roll_deg: float = 0.0
    ) -> Tuple[float, float]:
        """
        Convert pixel coordinates to GPS coordinates
        
        Args:
            pixel_x: Pixel X coordinate (from left)
            pixel_y: Pixel Y coordinate (from top)
            drone_lat: Drone latitude (degrees)
            drone_lon: Drone longitude (degrees)
            altitude_agl: Altitude above ground level (meters)
            heading_deg: Drone heading (degrees, 0=North, clockwise)
            pitch_deg: Camera pitch angle (degrees, + = up, - = down)
            roll_deg: Camera roll angle (degrees, + = right, - = left)
        
        Returns:
            Tuple of (latitude, longitude) for the ground point
        """
        
        # Calculate ground footprint dimensions
        ground_width, ground_height = self._calculate_ground_footprint(
            altitude_agl, pitch_deg
        )
        
        # Convert pixel to meters offset from image center
        offset_x, offset_y = self._pixel_to_meters_offset(
            pixel_x, pixel_y,
            ground_width, ground_height
        )
        
        # Apply rotation for heading, pitch, and roll
        rotated_x, rotated_y = self._apply_rotation(
            offset_x, offset_y,
            heading_deg, pitch_deg, roll_deg
        )
        
        # Convert meters offset to GPS coordinates
        detection_lat, detection_lon = self._meters_to_gps(
            drone_lat, drone_lon,
            rotated_x, rotated_y
        )
        
        return detection_lat, detection_lon
    
    def _calculate_ground_footprint(
        self,
        altitude_agl: float,
        pitch_deg: float = 0.0
    ) -> Tuple[float, float]:
        """
        Calculate the ground coverage area (footprint) of the camera
        
        Args:
            altitude_agl: Altitude above ground (meters)
            pitch_deg: Camera pitch angle (degrees)
        
        Returns:
            Tuple of (width_meters, height_meters) of ground coverage
        """
        
        # Adjust altitude for pitch
        pitch_rad = math.radians(pitch_deg)
        effective_altitude = altitude_agl / math.cos(pitch_rad)
        
        # Calculate ground dimensions using field of view
        # GSD (Ground Sampling Distance) formula
        ground_width = 2 * effective_altitude * math.tan(self.fov_h_rad / 2)
        ground_height = 2 * effective_altitude * math.tan(self.fov_v_rad / 2)
        
        return ground_width, ground_height
    
    def _pixel_to_meters_offset(
        self,
        pixel_x: float,
        pixel_y: float,
        ground_width: float,
        ground_height: float
    ) -> Tuple[float, float]:
        """
        Convert pixel coordinates to meters offset from image center
        
        Args:
            pixel_x: Pixel X coordinate
            pixel_y: Pixel Y coordinate
            ground_width: Ground coverage width (meters)
            ground_height: Ground coverage height (meters)
        
        Returns:
            Tuple of (offset_x_meters, offset_y_meters) from image center
        """
        
        image_width = self.params['image_width_px']
        image_height = self.params['image_height_px']
        
        # Calculate meters per pixel
        meters_per_pixel_x = ground_width / image_width
        meters_per_pixel_y = ground_height / image_height
        
        # Calculate offset from center (image center = drone position)
        center_x = image_width / 2
        center_y = image_height / 2
        
        offset_x = (pixel_x - center_x) * meters_per_pixel_x
        # Y axis: positive pixel_y is DOWN, but we want positive meters to be FORWARD
        offset_y = (center_y - pixel_y) * meters_per_pixel_y
        
        return offset_x, offset_y
    
    def _apply_rotation(
        self,
        offset_x: float,
        offset_y: float,
        heading_deg: float,
        pitch_deg: float = 0.0,
        roll_deg: float = 0.0
    ) -> Tuple[float, float]:
        """
        Apply rotation transformations for heading, pitch, and roll
        
        Args:
            offset_x: X offset in meters
            offset_y: Y offset in meters (forward/backward)
            heading_deg: Drone heading (0=North, clockwise)
            pitch_deg: Camera pitch
            roll_deg: Camera roll
        
        Returns:
            Rotated (x, y) offsets in meters
        """
        
        # Convert angles to radians
        heading_rad = math.radians(heading_deg)
        
        # Rotate by heading (yaw)
        # In camera frame: X=right, Y=forward
        # Rotate to ground frame: North-East coordinate system
        cos_h = math.cos(heading_rad)
        sin_h = math.sin(heading_rad)
        
        # Rotation matrix application
        rotated_x = offset_x * cos_h - offset_y * sin_h
        rotated_y = offset_x * sin_h + offset_y * cos_h
        
        # Note: For more accuracy, pitch and roll corrections could be added
        # but for typical agricultural drone operations with stable flight,
        # heading rotation is the primary correction needed
        
        return rotated_x, rotated_y
    
    def _meters_to_gps(
        self,
        origin_lat: float,
        origin_lon: float,
        offset_x: float,
        offset_y: float
    ) -> Tuple[float, float]:
        """
        Convert meters offset to GPS coordinates
        
        Args:
            origin_lat: Origin latitude (degrees)
            origin_lon: Origin longitude (degrees)
            offset_x: East-West offset in meters (+ = East)
            offset_y: North-South offset in meters (+ = North)
        
        Returns:
            Tuple of (latitude, longitude) in degrees
        """
        
        # Earth radius in meters
        EARTH_RADIUS = 6378137.0
        
        # Convert offsets to angular distances
        # 1 degree latitude ≈ 111,320 meters (constant)
        delta_lat = offset_y / 111320.0
        
        # 1 degree longitude varies with latitude
        # At equator: 111,320 meters, decreases with cos(latitude)
        lat_rad = math.radians(origin_lat)
        meters_per_degree_lon = 111320.0 * math.cos(lat_rad)
        delta_lon = offset_x / meters_per_degree_lon if meters_per_degree_lon > 0 else 0.0
        
        # Calculate final coordinates
        final_lat = origin_lat + delta_lat
        final_lon = origin_lon + delta_lon
        
        return final_lat, final_lon
    
    def calculate_ground_sample_distance(self, altitude_agl: float) -> float:
        """
        Calculate Ground Sampling Distance (GSD) - cm per pixel
        
        Args:
            altitude_agl: Altitude above ground (meters)
        
        Returns:
            GSD in centimeters per pixel
        """
        
        # GSD formula: (sensor_width * altitude * 100) / (focal_length * image_width)
        gsd_cm = (
            self.params['sensor_width_mm'] * altitude_agl * 100
        ) / (
            self.params['focal_length_mm'] * self.params['image_width_px']
        )
        
        return gsd_cm
    
    def get_footprint_info(self, altitude_agl: float) -> dict:
        """
        Get detailed camera footprint information
        
        Args:
            altitude_agl: Altitude above ground (meters)
        
        Returns:
            Dictionary with footprint information
        """
        
        ground_width, ground_height = self._calculate_ground_footprint(altitude_agl)
        gsd = self.calculate_ground_sample_distance(altitude_agl)
        
        return {
            'ground_width_m': ground_width,
            'ground_height_m': ground_height,
            'ground_area_m2': ground_width * ground_height,
            'gsd_cm_per_pixel': gsd,
            'altitude_agl_m': altitude_agl
        }
