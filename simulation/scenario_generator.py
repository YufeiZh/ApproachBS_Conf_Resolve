"""
ApproachBS Scenario Generator
--------------------------
This module implements scenario generation functionality for the ApproachBS plugin,
closely matching your original scenariogenerator.py implementation.
"""

import numpy as np
import random
import copy
import os
import json
import h5py
from pathlib import Path

from .aircraft import Aircraft
from .tracon import Tracon
from .restricted_area import RestrictedArea

class ScenarioGenerator:
    """
    Generator for TRACON scenarios with various densities and configurations.
    Based on your scenariogenerator.py implementation.
    """
    
    def __init__(self, tracon_manager):
        """
        Initialize the scenario generator.
        
        Args:
            tracon_manager: TraconManager instance
        """
        self.tracon_manager = tracon_manager
    
    def generate_scenario(self, gentype="MM"):
        """
        Generate a complete scenario based on type code.
        
        Args:
            gentype: Generation type code:
                - First character: restricted area density (S/M/D/F/R)
                - Second character: aircraft density (S/M/D/F/R)
                - Optional third/fourth character: L (LoS) and/or U (uncontrolled)
                
        Returns:
            bool: Success flag
        """
        # Extract density codes
        rest_den = gentype[0] if len(gentype) > 0 else 'M'
        acf_den = gentype[1] if len(gentype) > 1 else 'M'
        
        # Determine number of restricted areas
        match rest_den:
            case 'S':
                num_rest = random.randint(0, 5)
            case 'M':
                num_rest = random.randint(6, 12)
            case 'D':
                num_rest = random.randint(13, 20)
            case 'F':
                num_rest = 20
            case 'R':
                num_rest = random.randint(0, 20)
            case _:
                num_rest = random.randint(0, 10)  # Default to medium-sparse
        
        # Determine number of aircraft
        match acf_den:
            case 'S':
                num_acf = random.randint(1, 10)
            case 'M':
                num_acf = random.randint(11, 30)
            case 'D':
                num_acf = random.randint(31, 50)
            case 'F':
                num_acf = 50
            case 'R':
                num_acf = random.randint(0, 50)
            case _:
                num_acf = random.randint(5, 20)  # Default to medium-sparse
        
        # Get a random raw TRACON from the manager
        raw_tracon = self.tracon_manager.get_random_raw_tracon()
        if not raw_tracon:
            return False
        
        # Create a new TRACON from the raw one
        self.tracon_manager.set_current_tracon(copy.deepcopy(raw_tracon))
        
        # Add random restricted areas
        self.tracon_manager.current_tracon.add_random_restricts(num=num_rest)
        
        # Reset aircraft tracking
        self.tracon_manager.reset_aircraft_tracking()
        
        # Generate aircraft with the specified settings
        for i in range(num_acf):
            trkchange = random.getrandbits(1) == 1
            altchange = random.getrandbits(1) == 1
            spdchange = random.getrandbits(1) == 1
            
            # Options for Loss of Separation (L) and uncontrolled area (U)
            if 'L' in gentype and i % 3 == 1 and len(self.tracon_manager.aircraft_data) > 0:
                # Generate aircraft with potential conflict
                random_acid = random.choice(list(self.tracon_manager.aircraft_data.keys()))
                success = self.generate_los_aircraft(
                    existing_acf_id=random_acid,
                    trkchange=trkchange,
                    altchange=altchange,
                    spdchange=spdchange,
                    unctrl_area='U' in gentype
                )
            elif 'C' in gentype and i > 0 and len(self.tracon_manager.aircraft_data) > 0:
                # Generate aircraft close to first aircraft
                first_acid = list(self.tracon_manager.aircraft_data.keys())[0]
                success = self.generate_close_aircraft(
                    existing_acf_id=first_acid,
                    trkchange=trkchange,
                    altchange=altchange,
                    spdchange=spdchange,
                    unctrl_area='U' in gentype
                )
            else:
                # Generate random aircraft
                success = self.generate_random_aircraft(
                    trkchange=trkchange,
                    altchange=altchange,
                    spdchange=spdchange,
                    unctrl_area='U' in gentype
                )
        
        # Randomly permute the order of aircraft
        # This is managed by randomizing later when creating BlueSky aircraft
        
        # Set to active
        self.tracon_manager.active = True
        
        return True
    
    def generate_random_aircraft(self, acf_id=None, acf_type=None, acf_class=None,
                                avoid_restrict=False, trkchange=False, altchange=False,
                                spdchange=False, unctrl_area=True):
        """
        Generate a random aircraft in the scenario.
        
        Args:
            acf_id: Optional aircraft ID (generated if None)
            acf_type: Aircraft type (e.g., B738)
            acf_class: Aircraft weight class
            avoid_restrict: Whether to avoid restricted areas
            trkchange: Whether aircraft is changing track
            altchange: Whether aircraft is changing altitude
            spdchange: Whether aircraft is changing speed
            unctrl_area: Whether to allow generation in uncontrolled areas
            
        Returns:
            bool: Success flag
        """
        # Check if at maximum aircraft limit
        if len(self.tracon_manager.aircraft_data) >= 50:
            return False
        
        # Generate or validate callsign
        if acf_id is None:
            # Generate a unique callsign
            prefix = "TEST"
            exists = True
            number = 1
            
            while exists and number <= 50:
                test_id = f"{prefix}{number}"
                exists = test_id in self.tracon_manager.aircraft_data
                if not exists:
                    acf_id = test_id
                    break
                number += 1
                
            if exists:  # Could not find an available ID
                return False
        elif acf_id in self.tracon_manager.aircraft_data:
            # ID already exists
            return False
        
        # Get current TRACON details
        tracon = self.tracon_manager.current_tracon
        if not tracon:
            return False
        
        # Generate location
        counter = 0
        while True:
            counter += 1
            if counter > 50:  # Give up after too many attempts
                return False
                
            # Random position within or near TRACON
            bearing_from_ctr = round(random.uniform(0, 360), 1)
            if unctrl_area:
                dist_from_ctr = round(random.uniform(0, tracon.range + 10), 1)
            else:
                dist_from_ctr = round(random.uniform(0, tracon.range), 1)
                
            from .utils import geocalc
            lat, lon = geocalc.destination_by_bearing(
                tracon.ctr_lat, tracon.ctr_lon, bearing_from_ctr, dist_from_ctr
            )
            lat, lon = round(lat, 7), round(lon, 7)
            
            # Random altitude between min-1500 and max+3000
            alt = random.randint(max(tracon.min_alt - 1500, 0), tracon.max_alt + 3000)
            
            # Check if in restricted area (if avoid_restrict is set)
            if avoid_restrict:
                in_restrict = False
                for area in tracon.restrict:
                    if area.in_area(lat, lon, alt):
                        in_restrict = True
                        break
                        
                if in_restrict:
                    continue  # Try again
            
            # Valid position found
            break
        
        # Determine aircraft class and type
        if acf_class is None:
            if acf_type is not None:
                # Try to determine class from type
                acf_class = self.tracon_manager.determine_aircraft_class(acf_type)
                
            if acf_class is None:
                # Randomize class
                acf_class = random.choices(
                    ["light", "medium", "heavy", "A380"],
                    weights=[0.4, 5.0, 4.5, 0.1]
                )[0]
        
        # If class specified but not type, randomize type within class
        if acf_type is None:
            acf_type = self.tracon_manager.get_random_aircraft_type(acf_class)
        
        # Generate speed and track
        cas = random.randint(160, 250)
        trk = random.randint(0, 359)
        
        # Generate vertical profile
        if altchange:
            # Decide climb or descent
            climb = random.getrandbits(1)
            if climb:
                # Climbing
                target_alt = random.randint(alt+1000, tracon.max_alt+5000)
                temp_vs = random.randint(0, 3000)
                vs = random.choice([0, temp_vs, 1500])
                target_vs = 1500
            else:
                # Descending
                if alt <= 1000:
                    target_alt = alt
                    vs = 0
                    target_vs = 0
                else:
                    target_alt = random.randint(0, alt-1000)
                    temp_vs = random.randint(-3000, 0)
                    vs = random.choice([0, temp_vs, -1500])
                    target_vs = -1500
        else:
            # Level flight
            vs = 0
            target_vs = 0
            target_alt = alt
        
        # Generate track changes
        if trkchange:
            # Turn within 120 degrees either way
            delta_trk = random.randint(-120, 120)
            target_trk = (trk + delta_trk) % 360
        else:
            target_trk = trk
        
        # Generate speed changes
        if spdchange:
            target_cas = random.randint(160, 250)
        else:
            target_cas = cas
        
        # Create aircraft
        aircraft = Aircraft(
            callsign=acf_id,
            lat=lat,
            lon=lon,
            curr_alt=alt,
            curr_cas=cas,
            curr_trk=trk,
            curr_vs=vs,
            acf_type=acf_type,
            acf_class=acf_class,
            target_alt=target_alt,
            target_vs=target_vs,
            target_cas=target_cas,
            target_trk=target_trk,
            intention=random.choice(["arrival", "departure"]),
        )
        
        # Add to TRACON
        return self.tracon_manager.add_aircraft(aircraft)
    
    def generate_los_aircraft(self, existing_acf_id, acf_id=None, acf_type=None, acf_class=None,
                             avoid_restrict=False, trkchange=False, altchange=False,
                             spdchange=False, unctrl_area=True):
        """
        Generate an aircraft with potential Loss of Separation (conflict).
        
        Args:
            existing_acf_id: ID of existing aircraft to potentially conflict with
            acf_id: Optional aircraft ID (generated if None)
            acf_type: Aircraft type (e.g., B738)
            acf_class: Aircraft weight class
            avoid_restrict: Whether to avoid restricted areas
            trkchange: Whether aircraft is changing track
            altchange: Whether aircraft is changing altitude
            spdchange: Whether aircraft is changing speed
            unctrl_area: Whether to allow generation in uncontrolled areas
            
        Returns:
            bool: Success flag
        """
        # Check if at maximum aircraft limit
        if len(self.tracon_manager.aircraft_data) >= 50:
            return False
        
        # Check if existing aircraft exists
        if existing_acf_id not in self.tracon_manager.aircraft_data:
            # Fall back to generating a random aircraft
            return self.generate_random_aircraft(
                acf_id=acf_id,
                acf_type=acf_type,
                acf_class=acf_class,
                avoid_restrict=avoid_restrict,
                trkchange=trkchange,
                altchange=altchange,
                spdchange=spdchange,
                unctrl_area=unctrl_area
            )
        
        target_acf = self.tracon_manager.aircraft_data[existing_acf_id]
        
        # Generate or validate callsign
        if acf_id is None:
            # Generate a unique callsign
            prefix = "TEST"
            exists = True
            number = 1
            
            while exists and number <= 50:
                test_id = f"{prefix}{number}"
                exists = test_id in self.tracon_manager.aircraft_data
                if not exists:
                    acf_id = test_id
                    break
                number += 1
                
            if exists:  # Could not find an available ID
                return False
        elif acf_id in self.tracon_manager.aircraft_data:
            # ID already exists
            return False
        
        # Get current TRACON details
        tracon = self.tracon_manager.current_tracon
        if not tracon:
            return False
        
        # Generate location close to existing aircraft
        counter = 0
        while True:
            counter += 1
            if counter > 50:  # Give up after too many attempts
                return False
                
            # Position close to target aircraft (within 8NM horizontally)
            bearing_from_existing_acf = round(random.uniform(0, 360), 1)
            dist_from_existing_acf = round(random.uniform(0, 8), 2)
            
            # Vertical separation relative to target aircraft (-1000 to 1000 ft)
            vert_dist_from_existing_acf = -20000  # Invalid initial value
            while vert_dist_from_existing_acf < -1000 or vert_dist_from_existing_acf > 1000 or target_acf.curr_alt + vert_dist_from_existing_acf < 500:
                vert_dist_from_existing_acf = random.randint(-1000, 1000)
            
            # Calculate position
            from .utils import geocalc
            new_lat, new_lon = geocalc.destination_by_bearing(
                target_acf.lat, target_acf.lon, bearing_from_existing_acf, dist_from_existing_acf
            )
            new_lat = round(new_lat, 7)
            new_lon = round(new_lon, 7)
            new_alt = target_acf.curr_alt + vert_dist_from_existing_acf
            
            # Check if valid position
            dist_from_ctr = geocalc.distance_between(
                new_lat, new_lon, tracon.ctr_lat, tracon.ctr_lon
            )
            
            # Outside valid range
            if dist_from_ctr > tracon.range + 10:
                continue
                
            # In uncontrolled area but not allowed
            if dist_from_ctr > tracon.range and not unctrl_area:
                continue
                
            # Altitude out of range
            if new_alt < 0 or new_alt > tracon.max_alt + 3000:
                continue
                
            # In restricted area but should avoid
            if avoid_restrict:
                in_restrict = False
                for area in tracon.restrict:
                    if area.in_area(new_lat, new_lon, new_alt):
                        in_restrict = True
                        break
                        
                if in_restrict:
                    continue
            
            # Valid position found
            break
        
        # Determine aircraft class and type
        if acf_class is None:
            if acf_type is not None:
                # Try to determine class from type
                acf_class = self.tracon_manager.determine_aircraft_class(acf_type)
                
            if acf_class is None:
                # Randomize class
                acf_class = random.choices(
                    ["light", "medium", "heavy"],
                    weights=[0.5, 5.0, 4.5]
                )[0]
        
        # If class specified but not type, randomize type within class
        if acf_type is None:
            acf_type = self.tracon_manager.get_random_aircraft_type(acf_class)
        
        # Generate speed and track
        cas = random.randint(160, 250)
        trk = random.randint(0, 359)
        
        # Generate vertical profile
        if altchange:
            # Decide climb or descent
            climb = random.getrandbits(1)
            if climb:
                # Climbing
                target_alt = random.randint(new_alt+1000, tracon.max_alt+5000)
                temp_vs = random.randint(0, 3000)
                vs = random.choice([0, temp_vs, 1500])
                target_vs = 1500
            else:
                # Descending
                if new_alt <= 1000:
                    target_alt = new_alt
                    vs = 0
                    target_vs = 0
                else:
                    target_alt = random.randint(0, new_alt-1000)
                    temp_vs = random.randint(-3000, 0)
                    vs = random.choice([0, temp_vs, -1500])
                    target_vs = -1500
        else:
            # Level flight
            vs = 0
            target_vs = 0
            target_alt = new_alt
        
        # Generate track changes
        if trkchange:
            # Turn within 120 degrees either way
            delta_trk = random.randint(-120, 120)
            target_trk = (trk + delta_trk) % 360
        else:
            target_trk = trk
        
        # Generate speed changes
        if spdchange:
            target_cas = random.randint(160, 250)
        else:
            target_cas = cas
        
        # Create aircraft
        aircraft = Aircraft(
            callsign=acf_id,
            lat=new_lat,
            lon=new_lon,
            curr_alt=new_alt,
            curr_cas=cas,
            curr_trk=trk,
            curr_vs=vs,
            acf_type=acf_type,
            acf_class=acf_class,
            target_alt=target_alt,
            target_vs=target_vs,
            target_cas=target_cas,
            target_trk=target_trk,
            intention=random.choice(["arrival", "departure"]),
        )
        
        # Add to TRACON
        return self.tracon_manager.add_aircraft(aircraft)
    
    def generate_close_aircraft(self, existing_acf_id, acf_id=None, acf_type=None, acf_class=None,
                              avoid_restrict=False, trkchange=False, altchange=False,
                              spdchange=False, unctrl_area=True):
        """
        Generate an aircraft close to an existing aircraft (but not necessarily in conflict).
        
        Args:
            existing_acf_id: ID of existing aircraft to be close to
            acf_id: Optional aircraft ID (generated if None)
            acf_type: Aircraft type (e.g., B738)
            acf_class: Aircraft weight class
            avoid_restrict: Whether to avoid restricted areas
            trkchange: Whether aircraft is changing track
            altchange: Whether aircraft is changing altitude
            spdchange: Whether aircraft is changing speed
            unctrl_area: Whether to allow generation in uncontrolled areas
            
        Returns:
            bool: Success flag
        """
        # Similar to the LoS aircraft generation but with different separation parameters
        
        # Check if at maximum aircraft limit
        if len(self.tracon_manager.aircraft_data) >= 50:
            return False
        
        # Check if existing aircraft exists
        if existing_acf_id not in self.tracon_manager.aircraft_data:
            # Fall back to generating a random aircraft
            return self.generate_random_aircraft(
                acf_id=acf_id,
                acf_type=acf_type,
                acf_class=acf_class,
                avoid_restrict=avoid_restrict,
                trkchange=trkchange,
                altchange=altchange,
                spdchange=spdchange,
                unctrl_area=unctrl_area
            )
        
        target_acf = self.tracon_manager.aircraft_data[existing_acf_id]
        
        # Generate or validate callsign
        if acf_id is None:
            # Generate a unique callsign
            prefix = "TEST"
            exists = True
            number = 1
            
            while exists and number <= 50:
                test_id = f"{prefix}{number}"
                exists = test_id in self.tracon_manager.aircraft_data
                if not exists:
                    acf_id = test_id
                    break
                number += 1
                
            if exists:  # Could not find an available ID
                return False
        elif acf_id in self.tracon_manager.aircraft_data:
            # ID already exists
            return False
        
        # Get current TRACON details
        tracon = self.tracon_manager.current_tracon
        if not tracon:
            return False
        
        # Generate location close to existing aircraft
        counter = 0
        while True:
            counter += 1
            if counter > 50:  # Give up after too many attempts
                return False
                
            # Position close to target aircraft (within 10NM horizontally)
            bearing_from_existing_acf = round(random.uniform(0, 360), 1)
            dist_from_existing_acf = round(random.uniform(0, 10), 2)
            
            # Vertical separation between -3000 and 3000 ft
            vert_dist_from_existing_acf = -20000  # Invalid initial value
            while vert_dist_from_existing_acf < -3000 or vert_dist_from_existing_acf > 3000 or target_acf.curr_alt + vert_dist_from_existing_acf < 500:
                vert_dist_from_existing_acf = random.randint(-3000, 3000)
            
            # Calculate position
            from .utils import geocalc
            new_lat, new_lon = geocalc.destination_by_bearing(
                target_acf.lat, target_acf.lon, bearing_from_existing_acf, dist_from_existing_acf
            )
            new_lat = round(new_lat, 7)
            new_lon = round(new_lon, 7)
            new_alt = target_acf.curr_alt + vert_dist_from_existing_acf
            
            # Check if valid position
            dist_from_ctr = geocalc.distance_between(
                new_lat, new_lon, tracon.ctr_lat, tracon.ctr_lon
            )
            
            # Outside valid range
            if dist_from_ctr > tracon.range + 10:
                continue
                
            # In uncontrolled area but not allowed
            if dist_from_ctr > tracon.range and not unctrl_area:
                continue
                
            # Altitude out of range
            if new_alt < 0 or new_alt > tracon.max_alt + 3000:
                continue
                
            # In restricted area but should avoid
            if avoid_restrict:
                in_restrict = False
                for area in tracon.restrict:
                    if area.in_area(new_lat, new_lon, new_alt):
                        in_restrict = True
                        break
                        
                if in_restrict:
                    continue
            
            # Valid position found
            break
        
        # Determine aircraft class and type
        if acf_class is None:
            if acf_type is not None:
                # Try to determine class from type
                acf_class = self.tracon_manager.determine_aircraft_class(acf_type)
                
            if acf_class is None:
                # Randomize class
                acf_class = random.choices(
                    ["light", "medium", "heavy"],
                    weights=[0.5, 5.0, 4.5]
                )[0]
        
        # If class specified but not type, randomize type within class
        if acf_type is None:
            acf_type = self.tracon_manager.get_random_aircraft_type(acf_class)
        
        # Generate speed and track
        cas = random.randint(160, 250)
        trk = random.randint(0, 359)
        
        # Generate vertical profile
        if altchange:
            # Decide climb or descent
            climb = random.getrandbits(1)
            if climb:
                # Climbing
                target_alt = random.randint(new_alt+1000, tracon.max_alt+5000)
                temp_vs = random.randint(0, 3000)
                vs = random.choice([0, temp_vs, 1500])
                target_vs = 1500
            else:
                # Descending
                if new_alt <= 1000:
                    target_alt = new_alt
                    vs = 0
                    target_vs = 0
                else:
                    target_alt = random.randint(0, new_alt-1000)
                    temp_vs = random.randint(-3000, 0)
                    vs = random.choice([0, temp_vs, -1500])
                    target_vs = -1500
        else:
            # Level flight
            vs = 0
            target_vs = 0
            target_alt = new_alt
        
        # Generate track changes
        if trkchange:
            # Turn within 120 degrees either way
            delta_trk = random.randint(-120, 120)
            target_trk = (trk + delta_trk) % 360
        else:
            target_trk = trk
        
        # Generate speed changes
        if spdchange:
            target_cas = random.randint(160, 250)
        else:
            target_cas = cas
        
        # Create aircraft
        aircraft = Aircraft(
            callsign=acf_id,
            lat=new_lat,
            lon=new_lon,
            curr_alt=new_alt,
            curr_cas=cas,
            curr_trk=trk,
            curr_vs=vs,
            acf_type=acf_type,
            acf_class=acf_class,
            target_alt=target_alt,
            target_vs=target_vs,
            target_cas=target_cas,
            target_trk=target_trk,
            intention=random.choice(["arrival", "departure"]),
        )
        
        # Add to TRACON
        return self.tracon_manager.add_aircraft(aircraft)