# Geographic calculation tools

# Formulas can be found at https://www.movable-type.co.uk/scripts/latlong.html

''' Measurement units:
        - altitude/height/elevation: feet
        - lateral distance: nautical miles
        - speed: knots
        - latitude and longitude: degrees
        - heading/track: degrees '''

# from config import DATABASE_PATH
import numpy as np
import random
from pathlib import Path

# Default data: data cycle 2013.10, build 20131335, metadata AptXP1000. Copyright 2013, Robin A. Peel (robin@x-plane.com).
# The default navdata is open source lisenced with GPLv2+ by X-Plane. The default navdata was published in 2013 thus outdated.

# Global variable: the mean radius (in meters) of the standard earth model
REARTH = 6.3710088e6
# Global variable: the surface curvature (i.e. the inverse of the mean radius) of the standard earth model
REARTH_INV = 1.56961231e-7


def distance_between(lat1, lon1, lat2, lon2):
    ''' Calculate the distance between two points using Haversine formula. '''
    phi1 = np.radians(lat1)
    phi2 = np.radians(lat2)
    lambda1 = np.radians(lon1)
    lambda2 = np.radians(lon2)
    delta_phi = phi2 - phi1
    delta_lambda = lambda2 - lambda1
    a = np.square(np.sin(delta_phi/2)) + np.cos(phi1) * np.cos(phi2) * np.square(np.sin(delta_lambda/2))
    c = 2 * np.arctan2(np.sqrt(a), np.sqrt(1-a))
    d = REARTH * c
    return d * 5.4e-4


def bearing_from_to(lat1, lon1, lat2, lon2):
    ''' Calculate the bearing from one point to another. '''
    phi1, lambda1 = np.radians(lat1), np.radians(lon1)
    phi2, lambda2 = np.radians(lat2), np.radians(lon2)
    delta_lambda = lambda2 - lambda1

    theta = np.arctan2(np.sin(delta_lambda) * np.cos(phi2), np.cos(phi1) * np.sin(phi2) - np.sin(phi1) * np.cos(phi2) * np.cos(delta_lambda))

    return np.degrees(theta) % 360
    

def destination_by_bearing(lat0, lon0, bearing, dist):
    ''' Calculate the destination point given distance and bearing from a start point. '''
    distm = dist * 1852
    delta = distm * REARTH_INV
    phi0 = np.radians(lat0)
    lambda0 = np.radians(lon0)
    theta = np.radians(bearing)
    phi1 = np.arcsin(np.sin(phi0) * np.cos(delta) + np.cos(phi0) * np.sin(delta) * np.cos(theta))
    lambda1 = lambda0 + np.arctan2(np.sin(theta) * np.sin(delta) * np.cos(phi0), np.cos(delta) - np.sin(phi0) * np.sin(phi1))

    return np.degrees(phi1), (np.degrees(lambda1) + 180) % 360 - 180


def intersect_ray_ray(lat1, lon1, bearing1, lat2, lon2, bearing2):
    ''' Given two (great circle) rays, calculate the intersection. '''
    # Note that the "ray" is a great circle, instead of a Rhumb line.

    # Corner case: one point is on the other ray
    if round(bearing_from_to(lat1, lon1, lat2, lon2), 5) in (round(bearing1, 6), round((bearing1 + 180) % 360, 5)):
        return (lat2, lon2)
    if round(bearing_from_to(lat2, lon2, lat1, lon1), 5) in (round(bearing2, 6), round((bearing2 + 180) % 360, 5)):
        return (lat1, lon1)

    phi1, lambda1 = np.radians(lat1), np.radians(lon1)
    phi2, lambda2 = np.radians(lat2), np.radians(lon2)
    theta_13 = np.radians(bearing1)
    theta_23 = np.radians(bearing2)

    d_phi = phi2 - phi1
    d_lambda = lambda2 - lambda1

    delta_12 = 2 * np.arcsin(np.sqrt(np.square(np.sin(d_phi/2)) + np.cos(phi1) * np.cos(phi2) * np.square(np.sin(d_lambda/2))))
    theta_a = np.arccos(np.divide(np.sin(phi2) - np.sin(phi1) * np.cos(delta_12), np.sin(delta_12) * np.cos(phi1)))
    theta_b = np.arccos(np.divide(np.sin(phi1) - np.sin(phi2) * np.cos(delta_12), np.sin(delta_12) * np.cos(phi2)))

    if np.sin(d_lambda) > 0:
        theta_12 = theta_a
        theta_21 = 2 * np.pi - theta_b
    else:
        theta_12 = 2 * np.pi - theta_a
        theta_21 = theta_b
    
    alpha1 = theta_13 - theta_12
    alpha2 = theta_21 - theta_23
    alpha3 = np.arccos(-np.cos(alpha1) * np.cos(alpha2) + np.sin(alpha1) * np.sin(alpha2) * np.cos(delta_12))
    delta_13 = np.arctan2(np.sin(delta_12) * np.sin(alpha1) * np.sin(alpha2), np.cos(alpha2) + np.cos(alpha1) * np.cos(alpha3))

    phi3 = np.arcsin(np.sin(phi1) * np.cos(delta_13) + np.cos(phi1) * np.sin(delta_13) * np.cos(theta_13))
    d_lambda_03 = np.arctan2(np.sin(theta_13) * np.sin(delta_13) * np.cos(phi1), np.cos(delta_13) - np.sin(phi1) * np.sin(phi3))
    lambda3 = lambda1 + d_lambda_03

    return np.degrees(phi3), (np.degrees(lambda3) + 180) % 360 - 180


def intersect_ray_segment(lat0, lon0, bearing, lat1, lon1, lat2, lon2):
    ''' Determine if a ray start at a point with certain bearing intersects the arc segment between to points. 
     Return the intersection point if there is one. '''
    # Note that the "ray" is a great circle arc ray, instead of a Rhumb line ray.

    # Check if either (lat1, lon1) or (lat2, lon2) is on the ray to exclude the corner case.
    # Otherwise, illegal input for arcsin or arccos may accur when calling `intersect_ray_ray()`
    if round(bearing_from_to(lat0, lon0, lat1, lon1), 5) == round(bearing, 5):
        return lat1, lon1
    if round(bearing_from_to(lat0, lon0, lat2, lon2), 5) == round(bearing, 5):
        return lat2, lon2

    # Find the initial bearing of the two rays
    bearing_12 = bearing_from_to(lat1, lon1, lat2, lon2)
    bearing_21 = bearing_from_to(lat2, lon2, lat1, lon1)
    intersection_lat, intersection_lon = intersect_ray_ray(lat0, lon0, bearing, lat1, lon1, bearing_12)

    # Check if the arc from (lat0, lon0) to the intersection is a minor arc. If not, discard it.
    if round(bearing_from_to(lat0, lon0, intersection_lat, intersection_lon), 5) != round(bearing, 5):
        return

    # Check if the intersection is on the arc segment. If not, also discard it.
    bearing_1_intersection = bearing_from_to(lat1, lon1, intersection_lat, intersection_lon)
    bearing_2_intersection = bearing_from_to(lat2, lon2, intersection_lat, intersection_lon)
    if round(bearing_1_intersection, 5) == round(bearing_12, 5) \
        and round(bearing_2_intersection, 5) == round(bearing_21, 5):
        return round(intersection_lat, 7), round(intersection_lon, 7)

    return


def intersect_segment_segment(lat1, lon1, lat2, lon2, lat3, lon3, lat4, lon4):
    ''' Check if the arc segment connecting (lat1, lon1), (lat2, lon2) and the arc segment connecting 
     (lat3, lon3), (lat4, lon4) intersect. Return the intersection point if there is one. '''
    
    # Round up floats
    lat1, lon1 = round(lat1, 7), round(lon1, 7)
    lat2, lon2 = round(lat2, 7), round(lon2, 7)
    lat3, lon3 = round(lat3, 7), round(lon3, 7)
    lat4, lon4 = round(lat4, 7), round(lon4, 7)

    # If the two arc segments have common vertices, they intersect
    if (lat1, lon1) == (lat3, lon3) or (lat1, lon1) == (lat4, lon4):
        return lat1, lon1
    if (lat2, lon2) == (lat3, lon3) or (lat2, lon2) == (lat4, lon4):
        return lat2, lon2

    bearing_12 = bearing_from_to(lat1, lon1, lat2, lon2)
    bearing_21 = bearing_from_to(lat2, lon2, lat1, lon1)

    # They intersect if and only if:
     # - the ray start at vertex1 with bearing bearing_12 intersects the arc segment connecting
     #   vertex3 and vertex4 at vertex5, and
     # - bearing_21 equals bearing_25 
    # Note that this does not hold if vertex3 and vertex4 are antipodals.

    vertex5 = intersect_ray_segment(lat1, lon1, bearing_12, lat3, lon3, lat4, lon4)
    if vertex5 is None:
        return
    lat5, lon5 = vertex5
    bearing_25 = bearing_from_to(lat2, lon2, lat5, lon5)
    if round(bearing_21, 5) == round(bearing_25, 5):
        return lat5, lon5
    return


def distance_to_arc(lat0, lon0, lat1, lon1, lat2, lon2):
    ''' Calculate the distance from (lat0, lon0) to the great circle arc between (lat1, lon1) and (lat2, lon2). '''
    delta_10 = distance_between(lat1, lon1, lat0, lon0) * 1852 * REARTH_INV
    theta_10 = np.radians(bearing_from_to(lat1, lon1, lat0, lon0))
    theta_12 = np.radians(bearing_from_to(lat1, lon1, lat2, lon2))
    dist = np.arcsin(np.sin(delta_10) * np.sin(theta_10 - theta_12)) * REARTH

    return dist * 5.4e-4


def in_polygon(lat0, lon0, vertices:list):
    ''' Determine if a given point is in a polygon by Ray Casting. '''
    
    # vertices = [lat1, lon1, lat2, lon2, ..., latn, lonn]
    num_vertices = len(vertices) // 2


    while True:
        # randomly choose a bearing of the ray
        bearing = random.randint(0, 359)
        valid_bearing = True

        # count the number of intersections of the ray with the boundary edges
        num_intersections = 0
        for i in range(num_vertices):
            if i < num_vertices - 1:
                lat1, lon1, lat2, lon2 = vertices[2*i], vertices[2*i+1], vertices[2*i+2], vertices[2*i+3]
            else:
                lat1, lon1, lat2, lon2 = vertices[-2], vertices[-1], vertices[0], vertices[1]
            intersection = intersect_ray_segment(lat0, lon0, bearing, lat1, lon1, lat2, lon2)
            if intersection is None:
                continue
            if intersection == (round(lat1, 7), round(lon1, 7)) or intersection == (round(lat2, 7), round(lon2, 7)):
                # A choice of bearing is invalid if the ray passes a vertex
                valid_bearing = False
                break
            num_intersections += 1
        
        if valid_bearing:
            return num_intersections % 2 == 1
        

def distance_to_polygon(lat0, lon0, vertices:list):
    ''' Return the distance from a point to a polygon. '''

    if in_polygon(lat0, lon0, vertices):
        return 0.0
    
    # For point not in the polygon, return the min distance to edges and vertices
    min_dist = float('inf')

    num_vertices = len(vertices) // 2

    for i in range(num_vertices):
        cur_dist = distance_between(lat0, lon0, vertices[2*i], vertices[2*i+1])
        min_dist = min(min_dist, cur_dist)

    for i in range(num_vertices):
        if i < num_vertices - 1:
            lat1, lon1, lat2, lon2 = vertices[2*i], vertices[2*i+1], vertices[2*i+2], vertices[2*i+3]
        else:
            lat1, lon1, lat2, lon2 = vertices[-2], vertices[-1], vertices[0], vertices[1]
        bearing_10 = bearing_from_to(lat1, lon1, lat0, lon0)
        bearing_20 = bearing_from_to(lat2, lon2, lat0, lon0)
        bearing_12 = bearing_from_to(lat1, lon1, lat2, lon2)
        bearing_21 = bearing_from_to(lat2, lon2, lat1, lon1)

        if (bearing_10 - bearing_12) % 360 > 90 or (bearing_20 - bearing_21) % 360 > 90:
            continue
        
        cur_dist = abs(distance_to_arc(lat0, lon0, lat1, lon1, lat2, lon2))
        min_dist = min(min_dist, cur_dist)

    return min_dist


def check_simple_polygon(vertices:list):
    ''' Check if a given list of vertices can form a simple polygon in the sense that there are no intersecting edges. '''
    num_vertices = len(vertices) // 2
    for i in range(num_vertices-2):
        for j in range(i+2, num_vertices):
            lat1, lon1, lat2, lon2 = vertices[2*i], vertices[2*i+1], vertices[2*i+2], vertices[2*i+3]
            if j < num_vertices-1:
                lat3, lon3, lat4, lon4 = vertices[2*j], vertices[2*j+1], vertices[2*j+2], vertices[2*j+3]
            else:
                if i == 0:
                    continue
                lat3, lon3, lat4, lon4 = vertices[-2], vertices[-1], vertices[0], vertices[1]
            if intersect_segment_segment(lat1, lon1, lat2, lon2, lat3, lon3, lat4, lon4) is not None:
                return False
    return True

