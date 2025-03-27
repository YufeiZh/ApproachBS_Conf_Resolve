"""
ApproachBS Restricted Area Class
------------------------------
This module contains the RestrictedArea class that defines restricted
airspaces within the TRACON environment, matching your original implementation.
"""

from .utils import geocalc

class RestrictedArea:
    """
    Class representing a restricted airspace area.
    Matches the implementation from the original restricted_area.py.
    """
    
    def __init__(self, area_id="", area_type="circle", bottom=0, top=3000, area_args=None):
        """
        Initialize a restricted area.
        
        Args:
            area_id: Identifier for the restricted area
            area_type: Type of area ("circle" or "polygon")
            bottom: Bottom altitude in feet (MSL)
            top: Top altitude in feet (MSL)
            area_args: 
                For circle: [center_lat, center_lon, radius_nm]
                For polygon: [lat1, lon1, lat2, lon2, ..., latN, lonN]
        """
        # Identifier of the restricted area
        self.area_id = area_id
        
        # Type: "circle" or "polygon"
        self.area_type = area_type
        
        # Bottom altitude (MSL)
        self.area_bottom_alt = bottom
        
        # Top altitude (MSL)
        self.area_top_alt = top
        
        # Area arguments (coordinates)
        if area_args is None:
            area_args = []
            
        # Limit number of vertices for polygons (8 vertices = 16 coordinates)
        if area_type == "polygon" and len(area_args) > 16:
            print(f"Warning: Restricting polygon {area_id} to 8 vertices maximum")
            area_args = area_args[:16]
            
        self.area_args = area_args
    
    def __str__(self):
        """String representation of the restricted area."""
        return f"AREA ID: {self.area_id}, TYPE: {self.area_type}, " \
               f"RANGE: {self.area_bottom_alt} to {self.area_top_alt} ft, " \
               f"args: {self.area_args}"
    
    def in_area(self, lat, lon, alt):
        """
        Check if a point is within the restricted area.
        
        Args:
            lat: Latitude in degrees
            lon: Longitude in degrees
            alt: Altitude in feet
            
        Returns:
            bool: True if point is within area, False otherwise
        """
        # Check altitude first
        if alt < self.area_bottom_alt or alt > self.area_top_alt:
            return False
            
        # Check horizontal containment
        if self.area_type == "circle":
            if len(self.area_args) < 3:
                return False
                
            center_lat = self.area_args[0]
            center_lon = self.area_args[1]
            radius = self.area_args[2]
            
            # Calculate distance from center
            distance = geocalc.distance_between(lat, lon, center_lat, center_lon)
            
            # Point is in circle if distance to center is less than radius
            return distance <= radius
        
        elif self.area_type == "polygon":
            if len(self.area_args) < 6:  # Need at least 3 vertices (6 coordinates)
                return False
                
            # Use ray casting algorithm to determine if point is in polygon
            return geocalc.in_polygon(lat, lon, self.area_args)
        
        # Unknown area type
        return False
    
    def dist_to_area(self, lat, lon):
        """
        Calculate distance to area boundary.
        Negative if inside area, positive if outside.
        
        Args:
            lat: Latitude in degrees
            lon: Longitude in degrees
            
        Returns:
            float: Distance in nautical miles
        """
        # If point is in area, distance is 0
        if self.in_area(lat, lon, (self.area_bottom_alt + self.area_top_alt) / 2):
            return 0.0
            
        # Calculate horizontal distance
        if self.area_type == "circle":
            if len(self.area_args) < 3:
                return 999.0  # Invalid area
                
            center_lat = self.area_args[0]
            center_lon = self.area_args[1]
            radius = self.area_args[2]
            
            # Calculate distance from center and subtract radius
            distance = geocalc.distance_between(lat, lon, center_lat, center_lon)
            return distance - radius
        
        elif self.area_type == "polygon":
            if len(self.area_args) < 6:  # Need at least 3 vertices
                return 999.0  # Invalid area
                
            # Calculate distance to polygon
            return geocalc.distance_to_polygon(lat, lon, self.area_args)
        
        # Unknown area type
        return 999.0
    
    def to_dict(self):
        """Convert to dictionary for serialization."""
        return {
            'area_id': self.area_id,
            'area_type': self.area_type,
            'area_bottom_alt': self.area_bottom_alt,
            'area_top_alt': self.area_top_alt,
            'area_args': self.area_args
        }
    
    @classmethod
    def from_dict(cls, data_dict):
        """Create instance from dictionary."""
        return cls(
            area_id=data_dict.get('area_id', ''),
            area_type=data_dict.get('area_type', 'circle'),
            bottom=data_dict.get('area_bottom_alt', 0),
            top=data_dict.get('area_top_alt', 3000),
            area_args=data_dict.get('area_args', [])
        )