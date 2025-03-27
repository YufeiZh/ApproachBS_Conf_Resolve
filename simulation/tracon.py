"""
ApproachBS Tracon Class
---------------------
This module contains the Tracon class that defines the TRACON airspace,
closely matching the implementation in your original tracon.py.
"""

import numpy as np
import random
from pathlib import Path
import json

from .utils import geocalc
from .restricted_area import RestrictedArea

# Constants from your original implementation
BOTTOM = 500  # Default min elevation above ground
TOP = 15000  # Default max elevation above ground
DEFAULT_FAP_HEIGHT = 3000  # Default height for final approach points

class Tracon:
    """
    The class representing TRACON airspace.
    
    Matches the implementation from the original tracon.py for consistency
    with the standalone TRACON simulator.
    """
    
    def __init__(self, 
                 identifier="DEFAULT", 
                 ctr_lat=0, 
                 ctr_lon=0,
                 tracon_range=30, 
                 elevation=0,
                 apt_id=None,
                 apt_lat=None,
                 apt_lon=None,
                 apt_ele=None,
                 runway_id=None, 
                 runway_thres_lat=None,
                 runway_thres_lon=None,
                 runway_bearing=None, 
                 runway_length=None,
                 fap_lat=None, 
                 fap_lon=None,
                 fap_alt=None):
        """
        Initialize a TRACON airspace.
        
        Args:
            identifier: TRACON identifier/name
            ctr_lat: Center latitude (degrees)
            ctr_lon: Center longitude (degrees)
            tracon_range: Radius of TRACON area (nm)
            elevation: Ground elevation (ft)
            apt_id: List of airport IDs within TRACON
            apt_lat: List of airport latitudes
            apt_lon: List of airport longitudes
            apt_ele: List of airport elevations
            runway_id: Dictionary mapping airport IDs to lists of runway IDs
            runway_thres_lat: Dictionary mapping airport IDs to lists of runway threshold latitudes
            runway_thres_lon: Dictionary mapping airport IDs to lists of runway threshold longitudes
            runway_bearing: Dictionary mapping airport IDs to lists of runway headings
            runway_length: Dictionary mapping airport IDs to lists of runway lengths
            fap_lat: Dictionary mapping airport IDs to lists of final approach point latitudes
            fap_lon: Dictionary mapping airport IDs to lists of final approach point longitudes
            fap_alt: Dictionary mapping airport IDs to lists of final approach point altitudes
        """
        # TRACON Name
        self.identifier = identifier
        
        # Center location
        self.ctr_lat = ctr_lat
        self.ctr_lon = ctr_lon
        self.range = tracon_range
        self.elevation = elevation
        
        # Altitude limits - min altitude can be assigned at or above this
        self.min_alt = elevation + BOTTOM
        self.max_alt = elevation + TOP
        
        # Airport information
        self.apt_id = apt_id if apt_id else []
        self.apt_lat = apt_lat if apt_lat else []
        self.apt_lon = apt_lon if apt_lon else []
        self.apt_ele = apt_ele if apt_ele else []
        
        # Runway information
        self.runway_id = runway_id if runway_id else {}
        self.runway_thres_lat = runway_thres_lat if runway_thres_lat else {}
        self.runway_thres_lon = runway_thres_lon if runway_thres_lon else {}
        self.runway_bearing = runway_bearing if runway_bearing else {}
        self.runway_length = runway_length if runway_length else {}
        
        # Final approach points
        self.fap_lat = fap_lat if fap_lat else {}
        self.fap_lon = fap_lon if fap_lon else {}
        self.fap_alt = fap_alt if fap_alt else {}
        
        # Restricted areas
        self.restrict = []
    
    def load_rwy_info(self, navdata_loader):
        """
        Load runway and FAP information based on airport IDs.
        
        Args:
            navdata_loader: Navigation data loader to retrieve airport/runway data
        """
        if len(self.apt_id) == 0:
            return
        
        # Clear existing data
        self.apt_lat = []
        self.apt_lon = []
        self.apt_ele = []
        self.runway_id = {}
        self.runway_thres_lat = {}
        self.runway_thres_lon = {}
        self.runway_bearing = {}
        self.runway_length = {}
        self.fap_lat = {}
        self.fap_lon = {}
        self.fap_alt = {}
        
        # Load airport information
        aptinfo = navdata_loader.loadaptinfo(self.apt_id)
        rwyinfo = navdata_loader.loadrwyinfo(self.apt_id)
        fapinfo = navdata_loader.loadFAPs(self.apt_id)
        
        # Process airport data
        for apt in self.apt_id:
            if apt in aptinfo:
                self.apt_lat.append(aptinfo[apt][0])
                self.apt_lon.append(aptinfo[apt][1])
                self.apt_ele.append(aptinfo[apt][3])
            else:
                # Skip airports not in the database
                continue
        
        # Process runway data
        for apt in self.apt_id:
            if apt not in rwyinfo:
                continue
                
            self.runway_id[apt] = []
            self.runway_thres_lat[apt] = []
            self.runway_thres_lon[apt] = []
            self.runway_bearing[apt] = []
            self.runway_length[apt] = []
            self.fap_lat[apt] = []
            self.fap_lon[apt] = []
            self.fap_alt[apt] = []
            
            for rwy in rwyinfo[apt]:
                self.runway_id[apt].append(rwy)
                self.runway_thres_lat[apt].append(rwyinfo[apt][rwy][0])
                self.runway_thres_lon[apt].append(rwyinfo[apt][rwy][1])
                self.runway_bearing[apt].append(rwyinfo[apt][rwy][2])
                self.runway_length[apt].append(rwyinfo[apt][rwy][3])
                
                # Add FAP information if available
                if apt in fapinfo and rwy in fapinfo[apt]:
                    self.fap_lat[apt].append(fapinfo[apt][rwy][0])
                    self.fap_lon[apt].append(fapinfo[apt][rwy][1])
                    apt_idx = self.apt_id.index(apt)
                    self.fap_alt[apt].append(self.apt_ele[apt_idx] + DEFAULT_FAP_HEIGHT)
                else:
                    # Calculate default FAP if not in database
                    # FAP is typically on extended centerline 5-7nm from threshold
                    rwy_idx = self.runway_id[apt].index(rwy)
                    rwy_lat = self.runway_thres_lat[apt][rwy_idx]
                    rwy_lon = self.runway_thres_lon[apt][rwy_idx]
                    rwy_hdg = self.runway_bearing[apt][rwy_idx]
                    
                    # Opposite direction from runway heading
                    fap_hdg = (rwy_hdg + 180) % 360
                    
                    # Calculate position
                    fap_lat, fap_lon = geocalc.destination_by_bearing(
                        rwy_lat, rwy_lon, fap_hdg, 7.0
                    )
                    
                    self.fap_lat[apt].append(fap_lat)
                    self.fap_lon[apt].append(fap_lon)
                    
                    # Default altitude at FAP
                    apt_idx = self.apt_id.index(apt)
                    self.fap_alt[apt].append(self.apt_ele[apt_idx] + DEFAULT_FAP_HEIGHT)
    
    def add_restricted_area(self, restricted_area):
        """
        Add a restricted area to the TRACON airspace.
        
        Args:
            restricted_area: RestrictedArea object
            
        Returns:
            bool: True if added successfully, False if at limit (20 areas)
        """
        # Do not add if there are already 20 restricted areas
        if len(self.restrict) >= 20:
            return False
            
        self.restrict.append(restricted_area)
        return True
    
    def auto_add_airport_restrict(self):
        """
        Add restricted areas around airports automatically.
        Creates circular restricted areas around each airport.
        """
        for i in range(len(self.apt_id)):
            area_id = self.apt_id[i]
            bottom = self.apt_ele[i]
            top = bottom + 3000
            ctr_lat = self.apt_lat[i]
            ctr_lon = self.apt_lon[i]
            
            new_restrict = RestrictedArea(
                area_id=area_id, 
                area_type="circle", 
                bottom=bottom, 
                top=top, 
                area_args=[ctr_lat, ctr_lon, 5]
            )
            
            self.add_restricted_area(new_restrict)
    
    def add_random_restricts(self, num=None, area_type=None, avoid_apt=False):
        """
        Add random restricted areas to the TRACON.
        
        Args:
            num: Number of areas to add (random if None)
            area_type: Type of area ("circle", "polygon", or None for random)
            avoid_apt: Whether to avoid airports when placing areas
        """
        if num is None:
            num = random.randint(1, 20)

        # Add areas until we reach the target number or the limit of 20
        added = 0
        while added < num and len(self.restrict) < 20:
            if area_type is None:
                # Random type selection
                if random.randint(0, 1) == 1:
                    self.add_random_circle_restricted(avoid_apt=avoid_apt)
                else:
                    self.add_random_polygon_restricted(avoid_apt=avoid_apt)
            elif area_type == "circle":
                self.add_random_circle_restricted(avoid_apt=avoid_apt)
            elif area_type == "polygon":
                self.add_random_polygon_restricted(avoid_apt=avoid_apt)
            
            added += 1
    
    def add_random_circle_restricted(self, avoid_apt=False):
        """
        Add a random circular restricted area.
        
        Args:
            avoid_apt: Whether to avoid airports when placing the area
        """
        # Do not add if there are already 20 restricted areas
        if len(self.restrict) >= 20:
            return
        
        # Generate area id
        exist_id = [area.area_id for area in self.restrict]
        new_id = "AREA"
        for i in range(1, 21):
            if new_id + str(i) not in exist_id:
                new_id += str(i)
                break

        # Generate area top_alt and bottom_alt
        bottom_alt = random.randint(self.min_alt, self.max_alt - 1500)
        top_alt = random.randint(bottom_alt + 1000, self.max_alt)

        # Generate area center lat and lon
        valid = False
        attempts = 0
        while not valid and attempts < 50:
            attempts += 1
            valid = True
            dist_to_apts = []
            dist_to_faps = []
            
            # Random location within TRACON
            bearing_from_ctr = round(random.uniform(0, 360), 1)
            dist_from_ctr = round(random.uniform(0, self.range), 1)
            lat, lon = geocalc.destination_by_bearing(
                self.ctr_lat, self.ctr_lon, bearing_from_ctr, dist_from_ctr
            )
            lat, lon = round(lat, 7), round(lon, 7)

            # Check if too close to airports or FAPs
            if avoid_apt:
                # Check airports
                for i in range(len(self.apt_id)):
                    dist = geocalc.distance_between(
                        lat, lon, self.apt_lat[i], self.apt_lon[i]
                    )
                    
                    if dist < 6.5:  # Too close to airport
                        valid = False
                        break
                    
                    dist_to_apts.append(dist)
                
                if not valid:
                    continue
                
                # Check FAPs
                for apt in self.apt_id:
                    if apt not in self.fap_lat:
                        continue
                        
                    for i in range(len(self.fap_lat[apt])):
                        dist = geocalc.distance_between(
                            lat, lon, self.fap_lat[apt][i], self.fap_lon[apt][i]
                        )
                        
                        if dist < 3.5:  # Too close to FAP
                            valid = False
                            break
                        
                        dist_to_faps.append(dist)
                    
                    if not valid:
                        break

        # If we couldn't find a valid location after 50 attempts, just use the last one
        if not valid:
            return

        # Generate area radius
        min_radius = 1
        
        # Calculate maximum allowed radius to avoid overlap with airports/FAPs
        if avoid_apt and dist_to_apts and dist_to_faps:
            min_dist_to_apts = min(dist_to_apts)
            min_dist_to_faps = min(dist_to_faps)
            max_radius = min(min_dist_to_apts-5, min_dist_to_faps-2, 10)
        else:
            max_radius = 10
        
        # Ensure minimum is not greater than maximum
        max_radius = max(min_radius, max_radius)
        
        # Generate radius using Gaussian distribution centered between min and max
        radius = 1
        attempts = 0
        while attempts < 10:
            radius = random.gauss((min_radius+max_radius)/2, 2)
            radius = round(radius * 2) / 2  # Round to nearest 0.5
            if min_radius <= radius <= max_radius:
                break
            attempts += 1
        
        # Ensure radius is within bounds
        radius = max(min_radius, min(radius, max_radius))

        # Create and add the restricted area
        new_restrict = RestrictedArea(
            area_id=new_id, 
            area_type="circle", 
            bottom=bottom_alt, 
            top=top_alt, 
            area_args=[lat, lon, radius]
        )
        
        self.add_restricted_area(new_restrict)
    
    def add_random_polygon_restricted(self, avoid_apt=False, num_vertices=None):
        """
        Add a random polygon restricted area.
        
        Args:
            avoid_apt: Whether to avoid airports when placing the area
            num_vertices: Number of vertices (random 3-8 if None)
        """
        # Do not add if there are already 20 restricted areas
        if len(self.restrict) >= 20:
            return
        
        # Generate area id
        exist_id = [area.area_id for area in self.restrict]
        new_id = "AREA"
        for i in range(1, 21):
            if new_id + str(i) not in exist_id:
                new_id += str(i)
                break

        # Generate area top_alt and bottom_alt
        bottom_alt = random.randint(self.min_alt, self.max_alt - 1500)
        top_alt = random.randint(bottom_alt + 1000, self.max_alt)

        # Generate area center lat and lon
        valid = False
        attempts = 0
        while not valid and attempts < 50:
            attempts += 1
            valid = True
            dist_to_apts = []
            dist_to_faps = []
            
            # Random location within TRACON
            bearing_from_ctr = round(random.uniform(0, 360), 1)
            dist_from_ctr = round(random.uniform(0, self.range), 1)
            lat, lon = geocalc.destination_by_bearing(
                self.ctr_lat, self.ctr_lon, bearing_from_ctr, dist_from_ctr
            )
            lat, lon = round(lat, 7), round(lon, 7)

            # Check if too close to airports or FAPs
            if avoid_apt:
                # Check airports
                for i in range(len(self.apt_id)):
                    dist = geocalc.distance_between(
                        lat, lon, self.apt_lat[i], self.apt_lon[i]
                    )
                    
                    if dist < 6.5:  # Too close to airport
                        valid = False
                        break
                    
                    dist_to_apts.append(dist)
                
                if not valid:
                    continue
                
                # Check FAPs
                for apt in self.apt_id:
                    if apt not in self.fap_lat:
                        continue
                        
                    for i in range(len(self.fap_lat[apt])):
                        dist = geocalc.distance_between(
                            lat, lon, self.fap_lat[apt][i], self.fap_lon[apt][i]
                        )
                        
                        if dist < 3.5:  # Too close to FAP
                            valid = False
                            break
                        
                        dist_to_faps.append(dist)
                    
                    if not valid:
                        break

        # If we couldn't find a valid location after 50 attempts, just use the last one
        if not valid:
            return

        # Calculate radius parameters for polygon vertices
        min_radius = 1
        
        # Calculate maximum allowed radius to avoid overlap with airports/FAPs
        if avoid_apt and dist_to_apts and dist_to_faps:
            min_dist_to_apts = min(dist_to_apts)
            min_dist_to_faps = min(dist_to_faps)
            max_radius = min(min_dist_to_apts-5, min_dist_to_faps-2, 10)
        else:
            max_radius = 10
        
        # Ensure minimum is not greater than maximum
        max_radius = max(min_radius, max_radius)

        # Generate number of vertices if not specified
        if num_vertices is None:
            num_vertices = random.randint(3, 8)

        # Generate angle steps
        angle_steps = geocalc.random_angle_steps(num_vertices)

        # Generate vertices
        vertices = []
        angle_cums = 0
        for angle in angle_steps:
            angle_cums += angle
            
            # Random distance from center for each vertex
            dist_to_ctr = 0
            while dist_to_ctr < min_radius or dist_to_ctr > max_radius:
                dist_to_ctr = max_radius - np.random.exponential(2.5)
                dist_to_ctr = max(min_radius, min(dist_to_ctr, max_radius))
            
            # Calculate vertex position
            vert_lat, vert_lon = geocalc.destination_by_bearing(
                lat, lon, angle_cums * 360, dist_to_ctr
            )
            
            vertices.append(vert_lat)
            vertices.append(vert_lon)

        # Rotate the vertices (random starting point)
        shift = random.randint(0, num_vertices-1)
        vertices_rotate = []
        for i in range(num_vertices):
            shifted_idx = (i + shift) % num_vertices
            vertices_rotate.append(vertices[2 * shifted_idx])
            vertices_rotate.append(vertices[2 * shifted_idx + 1])

        # Create and add the restricted area
        new_restrict = RestrictedArea(
            area_id=new_id, 
            area_type="polygon", 
            bottom=bottom_alt, 
            top=top_alt, 
            area_args=vertices_rotate
        )
        
        self.add_restricted_area(new_restrict)
    
    def to_dict(self):
        """
        Convert the TRACON to a dictionary for serialization.
        
        Returns:
            dict: Dictionary representation of the TRACON
        """
        # Create a copy of the object's dict to avoid modifying the original
        dict_tracon = self.__dict__.copy()
        
        # Convert restricted areas to dictionaries
        if len(self.restrict) > 0:
            dict_restricted = []
            for ele in self.restrict:
                dict_restricted.append(ele.to_dict())
            dict_tracon["restrict"] = dict_restricted
        
        return dict_tracon
    
    @classmethod
    def from_dict(cls, dict_data):
        """
        Create a TRACON instance from a dictionary.
        
        Args:
            dict_data: Dictionary containing TRACON data
            
        Returns:
            Tracon: New TRACON instance
        """
        # Create a new instance
        instance = cls()
        
        # Update with dictionary data (except 'restrict' which needs special handling)
        for key, value in dict_data.items():
            if key != 'restrict':
                setattr(instance, key, value)
        
        # Process restricted areas
        if 'restrict' in dict_data and dict_data['restrict']:
            restrict_data = []
            for ele in dict_data['restrict']:
                restrict_data.append(RestrictedArea.from_dict(ele))
            instance.restrict = restrict_data
        else:
            instance.restrict = []
        
        return instance
    
    def display_tracon(self):
        """
        Create a visualization of the TRACON area.
        This is a placeholder - actual implementation would depend on visualization needs.
        """
        # Note: A full implementation would create a PIL Image as in your original code
        # For BlueSky we typically use its own visualization methods
        return None