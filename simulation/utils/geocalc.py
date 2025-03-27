"""
ApproachBS Geographic Calculation Utilities
----------------------------------------
This module contains the geographic calculation utilities for the ApproachBS plugin,
matching the geocalc.py from your original implementation.
"""

import numpy as np
import random
import math

# Global constants from your original implementation
REARTH = 6.3710088e6  # Mean radius (in meters) of the standard earth model
REARTH_INV = 1.56961231e-7  # Surface curvature (inverse of mean radius)

def distance_between(lat1, lon1, lat2, lon2):
    """
    Calculate the distance between two points using Haversine formula.
    
    Args:
        lat1, lon1: First point coordinates in degrees
        lat2, lon2: Second point coordinates in degrees
        
    Returns:
        float: Distance in nautical miles
    """
    phi1 = np.radians(lat1)
    phi2 = np.radians(lat2)
    lambda1 = np.radians(lon1)
    lambda2 = np.radians(lon2)
    delta_phi = phi2 - phi1
    delta_lambda = lambda2 - lambda1
    a = np.square(np.sin(delta_phi/2)) + np.cos(phi1) * np.cos(phi2) * np.square(np.sin(delta_lambda/2))
    c = 2 * np.arctan2(np.sqrt(a), np.sqrt(1-a))
    d = REARTH * c
    return d * 5.4e-4  # Convert to nautical miles

def bearing_from_to(lat1, lon1, lat2, lon2):
    """
    Calculate the bearing from one point to another.
    
    Args:
        lat1, lon1: First point coordinates in degrees
        lat2, lon2: Second point coordinates in degrees
        
    Returns:
        float: Bearing in degrees (0-360)
    """
    phi1, lambda1 = np.radians(lat1), np.radians(lon1)
    phi2, lambda2 = np.radians(lat2), np.radians(lon2)
    delta_lambda = lambda2 - lambda1
    
    theta = np.arctan2(
        np.sin(delta_lambda) * np.cos(phi2),
        np.cos(phi1) * np.sin(phi2) - np.sin(phi1) * np.cos(phi2) * np.cos(delta_lambda)
    )
    
    return np.degrees(theta) % 360

def destination_by_bearing(lat0, lon0, bearing, dist):
    """
    Calculate the destination point given distance and bearing from a start point.
    
    Args:
        lat0, lon0: Starting point coordinates in degrees
        bearing: Bearing in degrees
        dist: Distance in nautical miles
        
    Returns:
        tuple: (lat, lon) of destination point in degrees
    """
    distm = dist * 1852  # Convert nm to meters
    delta = distm * REARTH_INV
    phi0 = np.radians(lat0)
    lambda0 = np.radians(lon0)
    theta = np.radians(bearing)
    
    phi1 = np.arcsin(
        np.sin(phi0) * np.cos(delta) +
        np.cos(phi0) * np.sin(delta) * np.cos(theta)
    )
    
    lambda1 = lambda0 + np.arctan2(
        np.sin(theta) * np.sin(delta) * np.cos(phi0),
        np.cos(delta) - np.sin(phi0) * np.sin(phi1)
    )
    
    return np.degrees(phi1), (np.degrees(lambda1) + 180) % 360 - 180

def point_in_polygon(lat0, lon0, vertices):
    """
    Check if a point is inside a polygon using ray casting algorithm.
    
    Args:
        lat0, lon0: Point coordinates in degrees
        vertices: List of coordinates forming the polygon [lat1, lon1, lat2, lon2, ...]
        
    Returns:
        bool: True if point is inside polygon, False otherwise
    """
    # Number of vertices
    num_vertices = len(vertices) // 2
    
    # Try different bearings until we get a valid result
    max_attempts = 5
    for attempt in range(max_attempts):
        # Choose a random bearing for the ray
        bearing = random.randint(0, 359)
        valid_bearing = True
        
        # Count intersections
        num_intersections = 0
        for i in range(num_vertices):
            # Get edge vertices
            if i < num_vertices - 1:
                lat1, lon1 = vertices[2*i], vertices[2*i+1]
                lat2, lon2 = vertices[2*i+2], vertices[2*i+3]
            else:
                lat1, lon1 = vertices[2*i], vertices[2*i+1]
                lat2, lon2 = vertices[0], vertices[1]
            
            # Check if ray intersects this edge
            intersection = intersect_ray_segment(lat0, lon0, bearing, lat1, lon1, lat2, lon2)
            
            if intersection is None:
                continue
                
            # Check if it passes directly through a vertex
            if (intersection == (round(lat1, 7), round(lon1, 7)) or
                intersection == (round(lat2, 7), round(lon2, 7))):
                # Ray passes through a vertex, use a different bearing
                valid_bearing = False
                break
                
            # Count the intersection
            num_intersections += 1
        
        # If bearing is valid, use the result
        if valid_bearing:
            return num_intersections % 2 == 1
    
    # If we tried max_attempts times and couldn't get a valid bearing,
    # fall back to checking if point is close to any vertex
    for i in range(num_vertices):
        if distance_between(lat0, lon0, vertices[2*i], vertices[2*i+1]) < 0.001:
            return True
    
    # If all else fails, assume outside
    return False

def intersect_ray_segment(lat0, lon0, bearing, lat1, lon1, lat2, lon2):
    """
    Determine if a ray from point (lat0, lon0) with given bearing intersects
    the segment between (lat1, lon1) and (lat2, lon2).
    
    Args:
        lat0, lon0: Origin of ray
        bearing: Bearing of ray in degrees
        lat1, lon1: First endpoint of segment
        lat2, lon2: Second endpoint of segment
        
    Returns:
        tuple or None: Intersection coordinates or None if no intersection
    """
    # Check if either endpoint is on the ray to handle edge cases
    if round(bearing_from_to(lat0, lon0, lat1, lon1), 5) == round(bearing, 5):
        return lat1, lon1
    if round(bearing_from_to(lat0, lon0, lat2, lon2), 5) == round(bearing, 5):
        return lat2, lon2
    
    # Find bearings for the segment and ray
    bearing_12 = bearing_from_to(lat1, lon1, lat2, lon2)
    bearing_21 = bearing_from_to(lat2, lon2, lat1, lon1)
    
    # Find ray-ray intersection
    try:
        intersection_lat, intersection_lon = intersect_ray_ray(
            lat0, lon0, bearing, lat1, lon1, bearing_12
        )
    except:
        return None
    
    # Check if the intersection is on the ray from point 0
    if round(bearing_from_to(lat0, lon0, intersection_lat, intersection_lon), 5) != round(bearing, 5):
        return None
    
    # Check if the intersection is on the segment
    bearing_1_intersection = bearing_from_to(lat1, lon1, intersection_lat, intersection_lon)
    bearing_2_intersection = bearing_from_to(lat2, lon2, intersection_lat, intersection_lon)
    
    if (round(bearing_1_intersection, 5) == round(bearing_12, 5) and
        round(bearing_2_intersection, 5) == round(bearing_21, 5)):
        return round(intersection_lat, 7), round(intersection_lon, 7)
    
    return None

def intersect_ray_ray(lat1, lon1, bearing1, lat2, lon2, bearing2):
    """
    Find the intersection of two rays.
    
    Args:
        lat1, lon1: Origin of first ray
        bearing1: Bearing of first ray in degrees
        lat2, lon2: Origin of second ray
        bearing2: Bearing of second ray in degrees
        
    Returns:
        tuple: Coordinates of intersection point
    """
    # Corner case: one point is on the other ray
    if (round(bearing_from_to(lat1, lon1, lat2, lon2), 5) in 
        (round(bearing1, 5), round((bearing1 + 180) % 360, 5))):
        return lat2, lon2
    if (round(bearing_from_to(lat2, lon2, lat1, lon1), 5) in 
        (round(bearing2, 5), round((bearing2 + 180) % 360, 5))):
        return lat1, lon1
    
    # Convert to radians
    phi1, lambda1 = np.radians(lat1), np.radians(lon1)
    phi2, lambda2 = np.radians(lat2), np.radians(lon2)
    theta_13 = np.radians(bearing1)
    theta_23 = np.radians(bearing2)
    
    # Calculate great circle distance between points
    d_phi = phi2 - phi1
    d_lambda = lambda2 - lambda1
    
    delta_12 = 2 * np.arcsin(np.sqrt(
        np.square(np.sin(d_phi/2)) + 
        np.cos(phi1) * np.cos(phi2) * np.square(np.sin(d_lambda/2))
    ))
    
    # Calculate bearings between points
    theta_a = np.arccos(
        np.divide(
            np.sin(phi2) - np.sin(phi1) * np.cos(delta_12),
            np.sin(delta_12) * np.cos(phi1)
        )
    )
    
    theta_b = np.arccos(
        np.divide(
            np.sin(phi1) - np.sin(phi2) * np.cos(delta_12),
            np.sin(delta_12) * np.cos(phi2)
        )
    )
    
    # Determine initial and final bearings
    if np.sin(d_lambda) > 0:
        theta_12 = theta_a
        theta_21 = 2 * np.pi - theta_b
    else:
        theta_12 = 2 * np.pi - theta_a
        theta_21 = theta_b
    
    # Calculate intersection
    alpha1 = theta_13 - theta_12
    alpha2 = theta_21 - theta_23
    alpha3 = np.arccos(-np.cos(alpha1) * np.cos(alpha2) + 
                      np.sin(alpha1) * np.sin(alpha2) * np.cos(delta_12))
    
    delta_13 = np.arctan2(
        np.sin(delta_12) * np.sin(alpha1) * np.sin(alpha2),
        np.cos(alpha2) + np.cos(alpha1) * np.cos(alpha3)
    )
    
    # Calculate intersection coordinates
    phi3 = np.arcsin(
        np.sin(phi1) * np.cos(delta_13) + 
        np.cos(phi1) * np.sin(delta_13) * np.cos(theta_13)
    )
    
    d_lambda_13 = np.arctan2(
        np.sin(theta_13) * np.sin(delta_13) * np.cos(phi1),
        np.cos(delta_13) - np.sin(phi1) * np.sin(phi3)
    )
    
    lambda3 = lambda1 + d_lambda_13
    
    # Convert back to degrees
    return np.degrees(phi3), (np.degrees(lambda3) + 180) % 360 - 180

def distance_to_polygon(lat0, lon0, vertices):
    """
    Calculate minimum distance from a point to a polygon.
    
    Args:
        lat0, lon0: Point coordinates in degrees
        vertices: List of coordinates forming the polygon [lat1, lon1, lat2, lon2, ...]
        
    Returns:
        float: Minimum distance in nautical miles
    """
    # If point is inside the polygon, distance is 0
    if point_in_polygon(lat0, lon0, vertices):
        return 0.0
    
    # Number of vertices
    num_vertices = len(vertices) // 2
    
    # Check distance to each vertex
    min_dist = float('inf')
    for i in range(num_vertices):
        dist = distance_between(lat0, lon0, vertices[2*i], vertices[2*i+1])
        min_dist = min(min_dist, dist)
    
    # Check distance to each edge
    for i in range(num_vertices):
        if i < num_vertices - 1:
            lat1, lon1 = vertices[2*i], vertices[2*i+1]
            lat2, lon2 = vertices[2*i+2], vertices[2*i+3]
        else:
            lat1, lon1 = vertices[2*i], vertices[2*i+1]
            lat2, lon2 = vertices[0], vertices[1]
        
        # Check if point projects onto the segment
        bearing_10 = bearing_from_to(lat1, lon1, lat0, lon0)
        bearing_20 = bearing_from_to(lat2, lon2, lat0, lon0)
        bearing_12 = bearing_from_to(lat1, lon1, lat2, lon2)
        bearing_21 = bearing_from_to(lat2, lon2, lat1, lon1)
        
        # Skip if point doesn't project onto segment
        if (bearing_10 - bearing_12) % 360 > 90 or (bearing_20 - bearing_21) % 360 > 90:
            continue
        
        # Calculate distance to great circle arc
        dist = abs(distance_to_arc(lat0, lon0, lat1, lon1, lat2, lon2))
        min_dist = min(min_dist, dist)
    
    return min_dist

def distance_to_arc(lat0, lon0, lat1, lon1, lat2, lon2):
    """
    Calculate the distance from a point to a great circle arc.
    
    Args:
        lat0, lon0: Point coordinates in degrees
        lat1, lon1: Start of arc in degrees
        lat2, lon2: End of arc in degrees
        
    Returns:
        float: Distance in nautical miles
    """
    delta_10 = distance_between(lat1, lon1, lat0, lon0) * 1852 * REARTH_INV
    theta_10 = np.radians(bearing_from_to(lat1, lon1, lat0, lon0))
    theta_12 = np.radians(bearing_from_to(lat1, lon1, lat2, lon2))
    
    # Calculate angular distance
    dist = np.arcsin(np.sin(delta_10) * np.sin(theta_10 - theta_12)) * REARTH
    
    # Convert to nautical miles
    return dist * 5.4e-4

def random_angle_steps(steps, irregularity=0.2):
    """
    Generate random angles dividing a circle.
    
    Args:
        steps: Number of angles to generate
        irregularity: Variance of the spacing between consecutive angles
        
    Returns:
        list: List of angles in radians
    """
    # Handle irregularity parameter
    if irregularity is None:
        irregularity = np.random.exponential(0.2)
        if irregularity > 1:
            irregularity = 1.0
    
    # Generate steps
    irregularity *= 360 / steps
    angles = []
    lower = (360 / steps) - irregularity
    upper = (360 / steps) + irregularity
    cumsum = 0
    
    for i in range(steps):
        angle = random.uniform(lower, upper)
        angles.append(angle)
        cumsum += angle
    
    # Normalize so sum equals 360 degrees
    cumsum /= 360
    for i in range(steps):
        angles[i] /= cumsum
    
    return angles