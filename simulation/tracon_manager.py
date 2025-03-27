"""
ApproachBS Tracon Manager
----------------------
This module contains the TraconManager class which maintains the state
of the TRACON airspace and handles aircraft management in the plugin.
"""

import numpy as np
import random
from datetime import datetime
import time
import json
import os

import bluesky as bs
from bluesky import stack, traf, sim, scr, tools
from bluesky.tools.aero import ft, kts, nm, fpm
from bluesky.tools import datalog

from .aircraft import Aircraft
from .tracon import Tracon
from .utils import geocalc

# Aircraft types by weight class
AIRCRAFT_CLASSES = {
    'light': ['C172', 'C152', 'P28A', 'BE58', 'C208', 'C510', 'E55P'],
    'medium': ['B738', 'A320', 'E190', 'CRJ9', 'B737', 'A319', 'A321', 'B752', 'B753'],
    'heavy': ['B744', 'B748', 'B77L', 'B77W', 'B778', 'B779', 'A346', 'A345', 'A359', 'A35K', 'B788', 'B789', 'A333', 'A339'],
    'A380': ['A388', 'A38F']
}

class TraconManager:
    """
    Manager for TRACON operations and airspace in the ApproachBS plugin.
    """
    
    def __init__(self):
        """Initialize the TRACON manager."""
        # Activation state
        self.active = False
        
        # Current TRACON configuration
        self.current_tracon = None
        
        # Raw TRACON templates
        self.raw_tracons = []
        
        # Custom aircraft data tracking
        self.aircraft_data = {}
        
        # Conflict detection matrix
        # First axis: aircraft index (0-49)
        # Second axis: conflict target (0=out of bounds, 1-20=restricted areas, 21-70=aircraft)
        self.conflict = np.zeros(shape=(50, 71), dtype=np.int8)
        self.conflict_in_one_min = np.zeros(shape=(50, 71), dtype=np.int8)
        
        # Spawn parameters
        self.arrival_rate = 5.0    # aircraft per minute
        self.departure_rate = 4.0  # aircraft per minute
        self.min_aircraft_separation = 5 * nm  # minimum separation for spawning
        self.max_aircraft = 50     # maximum number of aircraft in simulation
        
        # Timing
        self.last_arrival_time = 0
        self.last_departure_time = 0
        
        # Statistics
        self.arrival_count = 0
        self.departure_count = 0
        
        # Logging
        self.log_handle = None
        
    def reset(self):
        """Reset manager state when simulation is reset."""
        self.aircraft_data = {}
        self.last_arrival_time = 0
        self.last_departure_time = 0
        self.arrival_count = 0
        self.departure_count = 0
        self.conflict = np.zeros(shape=(50, 71), dtype=np.int8)
        self.conflict_in_one_min = np.zeros(shape=(50, 71), dtype=np.int8)
    
    def reset_aircraft_tracking(self):
        """Reset only the aircraft tracking portion."""
        self.aircraft_data = {}
        self.last_arrival_time = 0
        self.last_departure_time = 0
        self.arrival_count = 0
        self.departure_count = 0
        self.conflict = np.zeros(shape=(50, 71), dtype=np.int8)
        self.conflict_in_one_min = np.zeros(shape=(50, 71), dtype=np.int8)
    
    def load_raw_tracons(self, tracons_data):
        """
        Load raw TRACON definitions.
        
        Args:
            tracons_data: List of TRACON dictionaries or objects
        """
        self.raw_tracons = []
        
        # Convert data to Tracon objects if needed
        for data in tracons_data:
            if isinstance(data, dict):
                tracon = Tracon.from_dict(data)
                self.raw_tracons.append(tracon)
            else:
                self.raw_tracons.append(data)
        
        scr.echo(f"Loaded {len(self.raw_tracons)} raw TRACONs")
    
    def get_raw_tracon_by_id(self, identifier):
        """
        Get a raw TRACON by identifier.
        
        Args:
            identifier: TRACON identifier
            
        Returns:
            Tracon: TRACON object or None if not found
        """
        for tracon in self.raw_tracons:
            if tracon.identifier == identifier:
                return tracon
        return None
    
    def get_random_raw_tracon(self):
        """
        Get a random raw TRACON.
        
        Returns:
            Tracon: Random TRACON object or None if none available
        """
        if not self.raw_tracons:
            return None
        
        return random.choice(self.raw_tracons)
    
    def set_current_tracon(self, tracon):
        """
        Set the current active TRACON.
        
        Args:
            tracon: Tracon object
            
        Returns:
            bool: Success flag
        """
        if tracon is None:
            return False
            
        self.current_tracon = tracon
        return True
    
    def pre_update(self):
        """Operations to perform before the main update cycle."""
        # Check for new aircraft in the simulation not in our tracking
        self._check_new_aircraft()
    
    def update(self):
        """Main update function called by BlueSky."""
        # Get current simulation time
        current_time = sim.simt
        
        # Update custom data for existing aircraft
        self._update_aircraft_data()
        
        # Update conflict detection
        self._update_conflict_detection()
        
        # Check for aircraft to remove
        self._check_aircraft_removal()
        
        # Auto-generate aircraft if enabled
        if self.active and len(traf.id) < self.max_aircraft:
            self._auto_generate_aircraft(current_time)
        
        # Log TRACON data
        self._log_data()
    
    def _check_new_aircraft(self):
        """Check for new aircraft not in our tracking."""
        for i, callsign in enumerate(traf.id):
            if callsign not in self.aircraft_data:
                # Initialize data for this new aircraft
                self._init_aircraft_data(callsign)
    
    def _init_aircraft_data(self, callsign):
        """Initialize aircraft data structure for a new aircraft."""
        if callsign in self.aircraft_data:
            return
            
        i = traf.id.index(callsign)
        
        # Determine if this is likely an arrival or departure based on altitude and position
        ac_x, ac_y = traf.lat[i], traf.lon[i]
        
        if self.current_tracon:
            distance = geocalc.distance_between(ac_x, ac_y, self.current_tracon.ctr_lat, self.current_tracon.ctr_lon)
            # If it's far from center or at higher altitude, probably an arrival
            is_arrival = distance > 15 * nm or traf.alt[i] > 8000
        else:
            # Default if no TRACON is defined
            is_arrival = traf.alt[i] > 8000
        
        # For demo purposes, can also determine from callsign prefix
        if callsign.startswith("ARR"):
            is_arrival = True
        elif callsign.startswith("DEP"):
            is_arrival = False
        
        # Get current time
        current_time = sim.simt
        
        # Create aircraft data
        aircraft = Aircraft(
            callsign=callsign,
            lat=ac_x,
            lon=ac_y,
            curr_alt=traf.alt[i],
            curr_cas=traf.cas[i] if hasattr(traf, 'cas') else traf.tas[i],
            curr_trk=traf.hdg[i],
            curr_vs=traf.vs[i],
            acf_type=traf.type[i] if hasattr(traf, 'type') else "B738",
            acf_class=self.determine_aircraft_class(traf.type[i] if hasattr(traf, 'type') else "B738"),
            intention="arrival" if is_arrival else "departure",
            under_ctrl=False,  # Will be updated based on position check
            time_entered=current_time,  # Assume just entered
            time_stayed=0,
            time_last_command=current_time,
            time_since_last_command=0,
            status="Active"
        )
        
        # Add to our tracking
        self.aircraft_data[callsign] = aircraft
        
        # Update control status
        self._update_control_status(callsign)
    
    def _update_aircraft_data(self):
        """Update custom data for all aircraft."""
        current_time = sim.simt
        
        # Update existing aircraft
        for callsign in list(self.aircraft_data.keys()):
            if callsign in traf.id:
                ac_idx = traf.id.index(callsign)
                acdata = self.aircraft_data[callsign]
                
                # Update position and state
                acdata.lat = traf.lat[ac_idx]
                acdata.lon = traf.lon[ac_idx]
                acdata.curr_alt = traf.alt[ac_idx]
                acdata.curr_cas = traf.cas[ac_idx] if hasattr(traf, 'cas') else traf.tas[ac_idx]
                acdata.curr_trk = traf.hdg[ac_idx]
                acdata.curr_vs = traf.vs[ac_idx]
                
                # Update control status
                self._update_control_status(callsign)
                
                # Update timing information
                if acdata.under_ctrl and acdata.time_entered is not None:
                    acdata.time_stayed = current_time - acdata.time_entered
                
                if acdata.time_last_command is not None:
                    acdata.time_since_last_command = current_time - acdata.time_last_command
                
                # Update waypoint information
                acdata.update_dist_to_wpt()
                acdata.update_trk_to_wpt()
            else:
                # Aircraft no longer exists, remove from tracking
                self.aircraft_data.pop(callsign, None)
    
    def _update_control_status(self, callsign):
        """
        Update whether an aircraft is under TRACON control.
        
        Args:
            callsign: Aircraft callsign
        """
        if callsign not in self.aircraft_data or callsign not in traf.id:
            return
            
        acdata = self.aircraft_data[callsign]
        
        # Check if TRACON is defined
        if self.current_tracon is None:
            acdata.under_ctrl = False
            return
            
        # Check if in TRACON control zone
        distance = geocalc.distance_between(
            acdata.lat, acdata.lon, 
            self.current_tracon.ctr_lat, self.current_tracon.ctr_lon
        )
        
        in_control_zone = (
            distance <= self.current_tracon.range and 
            acdata.curr_alt >= self.current_tracon.min_alt and 
            acdata.curr_alt <= self.current_tracon.max_alt
        )
        
        # Aircraft just entered TRACON control
        current_time = sim.simt
        if in_control_zone and not acdata.under_ctrl:
            acdata.under_ctrl = True
            acdata.time_entered = current_time
        
        # Aircraft just exited TRACON control
        elif not in_control_zone and acdata.under_ctrl:
            acdata.under_ctrl = False
    
    def _update_conflict_detection(self):
        """Update conflict detection matrix."""
        # Reset conflict matrices
        self.conflict = np.zeros(shape=(50, 71), dtype=np.int8)
        self.conflict_in_one_min = np.zeros(shape=(50, 71), dtype=np.int8)
        
        # Map aircraft callsigns to indices in conflict matrix
        callsign_to_idx = {}
        for i, callsign in enumerate(self.aircraft_data.keys()):
            if i < 50:  # Only track up to 50 aircraft
                callsign_to_idx[callsign] = i
        
        # Check if TRACON is defined
        if self.current_tracon is None:
            return
        
        # Check each aircraft
        for callsign, idx in callsign_to_idx.items():
            if callsign not in traf.id:
                continue
                
            ac_idx = traf.id.index(callsign)
            
            # 1. Check if aircraft is out of TRACON bounds
            acdata = self.aircraft_data[callsign]
            distance = geocalc.distance_between(
                acdata.lat, acdata.lon, 
                self.current_tracon.ctr_lat, self.current_tracon.ctr_lon
            )
            
            if (distance > self.current_tracon.range or 
                acdata.curr_alt < self.current_tracon.min_alt or 
                acdata.curr_alt > self.current_tracon.max_alt):
                self.conflict[idx, 0] = 1
            else:
                self.conflict[idx, 0] = 0
            
            # 2. Check restricted areas
            for j, area in enumerate(self.current_tracon.restrict):
                if j >= 20:  # Maximum 20 restricted areas
                    break
                    
                if area.in_area(acdata.lat, acdata.lon, acdata.curr_alt):
                    self.conflict[idx, j+1] = 1
                else:
                    self.conflict[idx, j+1] = 0
            
            # 3. Check conflicts with other aircraft
            for other_callsign, other_idx in callsign_to_idx.items():
                if callsign == other_callsign:
                    continue
                    
                if other_callsign not in traf.id:
                    continue
                    
                other_ac_idx = traf.id.index(other_callsign)
                
                # Calculate horizontal and vertical separation
                other_acdata = self.aircraft_data[other_callsign]
                
                # Horizontal distance
                hor_dist = geocalc.distance_between(
                    acdata.lat, acdata.lon,
                    other_acdata.lat, other_acdata.lon
                )
                
                # Vertical separation
                vert_dist = abs(acdata.curr_alt - other_acdata.curr_alt)
                
                # Check for loss of separation (5nm horizontal, 1000ft vertical)
                if hor_dist < 5 and vert_dist < 1000:
                    self.conflict[idx, other_idx + 21] = 1
                    self.conflict[other_idx, idx + 21] = 1
                else:
                    self.conflict[idx, other_idx + 21] = 0
                    self.conflict[other_idx, idx + 21] = 0
        
        # Check for conflicts that will happen in one minute
        # This would require trajectory prediction - simplified version here
        self.conflict_in_one_min = self.conflict.copy()
    
    def _check_aircraft_removal(self):
        """Check for aircraft that should be automatically removed."""
        if not self.active or not self.current_tracon:
            return
        
        # Check each aircraft
        for callsign in list(self.aircraft_data.keys()):
            if callsign not in traf.id:
                continue
                
            ac_idx = traf.id.index(callsign)
            acdata = self.aircraft_data[callsign]
            
            # Default: don't remove
            should_remove = False
            removal_reason = ""
            
            # 1. Check if aircraft has left TRACON airspace by a significant margin
            distance = geocalc.distance_between(
                acdata.lat, acdata.lon, 
                self.current_tracon.ctr_lat, self.current_tracon.ctr_lon
            )
            
            if distance > self.current_tracon.range * 1.1:  # 10% buffer
                should_remove = True
                removal_reason = "Left TRACON airspace"
            
            # 2. For arrivals, check if aircraft has "landed" (close to runway at low altitude)
            elif acdata.intention == "arrival" and acdata.target_wp_lat is not None:
                # Calculate distance to target waypoint (assumed to be runway threshold)
                dist_to_target = acdata.dist_to_wpt
                
                # Check if close to runway threshold and at low altitude
                if dist_to_target is not None and dist_to_target < 0.5 and acdata.curr_alt < 1000:
                    # Check if aligned with runway (this is simplified)
                    # In a full implementation, we would check against actual runway heading
                    # and verify we're at the correct runway
                    
                    # For now, just assume we've landed if close to target point
                    should_remove = True
                    removal_reason = f"Landed at arrival waypoint"
                    
                    # Count as successful arrival
                    self.arrival_count += 1
            
            # 3. For departures, check if aircraft has reached exit fix
            elif acdata.intention == "departure" and acdata.target_wp_lat is not None:
                # Distance to target waypoint (exit fix)
                dist_to_target = acdata.dist_to_wpt
                
                # Check if reached fix
                if dist_to_target is not None and dist_to_target < 2:
                    should_remove = True
                    removal_reason = "Reached departure fix"
                    
                    # Count as successful departure
                    self.departure_count += 1
            
            # 4. Check for very low ground speed (possible stall)
            if hasattr(traf, 'gs') and traf.gs[ac_idx] < 50:  # knots
                should_remove = True
                removal_reason = "Possible stall (low ground speed)"
            
            # 5. Check for very low altitude away from airports
            if acdata.curr_alt < 500:
                # Check if near any airport
                near_airport = False
                for i in range(len(self.current_tracon.apt_id)):
                    apt_dist = geocalc.distance_between(
                        acdata.lat, acdata.lon, 
                        self.current_tracon.apt_lat[i], self.current_tracon.apt_lon[i]
                    )
                    if apt_dist < 3:  # Within 3nm of an airport
                        near_airport = True
                        break
                
                if not near_airport:
                    should_remove = True
                    removal_reason = "Too low altitude away from airport"
            
            # Remove aircraft if conditions met
            if should_remove:
                # Update status before removal
                acdata.status = removal_reason
                
                # Log removal reason
                scr.echo(f"ApproachBS: Removing {callsign} - {removal_reason}")
                
                # Delete the aircraft
                stack.stack(f"DEL {callsign}")
    
    def _auto_generate_aircraft(self, current_time):
        """Automatically generate new aircraft based on configured rates."""
        # Check if it's time to generate a new arrival
        time_since_last_arrival = current_time - self.last_arrival_time
        if time_since_last_arrival > 60 / self.arrival_rate:
            # Try to generate a new arrival
            if self._can_spawn_new_aircraft(True):
                self.generate_aircraft(is_arrival=True)
                self.last_arrival_time = current_time
        
        # Check if it's time to generate a new departure
        time_since_last_departure = current_time - self.last_departure_time
        if time_since_last_departure > 60 / self.departure_rate:
            # Try to generate a new departure
            if self._can_spawn_new_aircraft(False):
                self.generate_aircraft(is_arrival=False)
                self.last_departure_time = current_time
    
    def _can_spawn_new_aircraft(self, is_arrival):
        """Check if a new aircraft can be spawned without conflicts."""
        # Check if we've reached the maximum number of aircraft
        if len(traf.id) >= self.max_aircraft:
            return False
        
        # For now, just ensure we're not too close to the limit
        return len(traf.id) < self.max_aircraft * 0.9
    
    def generate_aircraft(self, callsign=None, acf_type=None, is_arrival=True):
        """
        Generate a new aircraft in the simulation.
        
        Args:
            callsign: Optional callsign for the new aircraft
            acf_type: Optional aircraft type
            is_arrival: Whether to generate an arrival (True) or departure (False)
            
        Returns:
            str: Callsign of created aircraft or None if failed
        """
        # Check if we have a TRACON defined
        if not self.current_tracon:
            return None
        
        # Generate callsign if not provided
        if callsign is None:
            prefix = "ARR" if is_arrival else "DEP"
            suffix = random.randint(1000, 9999)
            callsign = f"{prefix}{suffix}"
            
            # Ensure callsign is unique
            while callsign in traf.id:
                suffix = random.randint(1000, 9999)
                callsign = f"{prefix}{suffix}"
        
        # Select aircraft type if not provided
        if acf_type is None:
            # Choose weight class with bias toward medium/heavy
            acf_class = random.choices(
                ["light", "medium", "heavy", "A380"],
                weights=[0.1, 0.5, 0.35, 0.05]
            )[0]
            
            acf_type = self.get_random_aircraft_type(acf_class)
        
        if is_arrival:
            return self._generate_arrival(callsign, acf_type)
        else:
            return self._generate_departure(callsign, acf_type)
    
    def _generate_arrival(self, callsign, acf_type):
        """Generate an arrival aircraft."""
        # Check for TRACON
        if not self.current_tracon:
            return None
        
        # Random entry point around TRACON boundary
        entry_angle = random.uniform(0, 2*np.pi)
        entry_distance = self.current_tracon.range * random.uniform(0.95, 1.0)
        
        entry_lat, entry_lon = geocalc.destination_by_bearing(
            self.current_tracon.ctr_lat, self.current_tracon.ctr_lon, 
            np.degrees(entry_angle), entry_distance
        )
        
        # Random altitude between 7000ft and 12000ft
        entry_alt = random.uniform(7000, 12000)
        
        # Heading toward airport
        entry_hdg = (geocalc.bearing_from_to(
            entry_lat, entry_lon, 
            self.current_tracon.ctr_lat, self.current_tracon.ctr_lon
        ) + 360) % 360
        
        # Speed between 180kts and 250kts
        entry_spd = random.uniform(180, 250)
        
        # Assign a runway if available
        target_apt = None
        target_rwy = None
        target_lat = self.current_tracon.ctr_lat
        target_lon = self.current_tracon.ctr_lon
        target_alt = 0
        
        if len(self.current_tracon.apt_id) > 0:
            apt_idx = random.randint(0, len(self.current_tracon.apt_id) - 1)
            target_apt = self.current_tracon.apt_id[apt_idx]
            
            if target_apt in self.current_tracon.runway_id and len(self.current_tracon.runway_id[target_apt]) > 0:
                rwy_idx = random.randint(0, len(self.current_tracon.runway_id[target_apt]) - 1)
                target_rwy = self.current_tracon.runway_id[target_apt][rwy_idx]
                target_lat = self.current_tracon.runway_thres_lat[target_apt][rwy_idx]
                target_lon = self.current_tracon.runway_thres_lon[target_apt][rwy_idx]
                target_alt = self.current_tracon.apt_ele[apt_idx]
        
        # Create BlueSky aircraft
        stack.stack(f"CRE {acf_type} {callsign} {entry_lat} {entry_lon} {entry_hdg} {entry_alt} {entry_spd}")
        
        # Set destination in BlueSky if runway is assigned
        if target_apt and target_rwy:
            stack.stack(f"DEST {callsign} {target_apt}/{target_rwy}")
        
        # Wait for BlueSky to create the aircraft
        # In a real implementation, we would check traf.id, but for simplicity we'll assume it was created
        
        # Get weight class
        acf_class = self.determine_aircraft_class(acf_type)
        
        # Create aircraft data
        aircraft = Aircraft(
            callsign=callsign,
            lat=entry_lat,
            lon=entry_lon,
            curr_alt=entry_alt,
            curr_cas=entry_spd,
            curr_trk=entry_hdg,
            curr_vs=0,
            acf_type=acf_type,
            acf_class=acf_class,
            intention="arrival",
            target_wp_lat=target_lat,
            target_wp_lon=target_lon,
            target_wp_alt=target_alt,
            under_ctrl=False,  # Will be updated in next cycle
            time_entered=None,  # Will be set when aircraft enters TRACON
            time_stayed=0,
            time_last_command=sim.simt,
            time_since_last_command=0,
            status="Active"
        )
        
        # Calculate distance and track to target
        aircraft.dist_to_wpt = geocalc.distance_between(
            entry_lat, entry_lon, target_lat, target_lon
        )
        aircraft.trk_to_wpt = geocalc.bearing_from_to(
            entry_lat, entry_lon, target_lat, target_lon
        )
        
        # Add to tracking
        self.aircraft_data[callsign] = aircraft
        
        # Log creation
        scr.echo(f"ApproachBS: Generated arrival {callsign} from {np.degrees(entry_angle):.1f}° at {entry_distance:.1f}nm")
        
        return callsign
    
    def _generate_departure(self, callsign, acf_type):
        """Generate a departure aircraft."""
        # Check for TRACON
        if not self.current_tracon:
            return None
        
        # Select origin airport and runway if available
        origin_apt = None
        origin_rwy = None
        origin_lat = self.current_tracon.ctr_lat
        origin_lon = self.current_tracon.ctr_lon
        origin_hdg = random.uniform(0, 359)
        origin_alt = 1500
        
        if len(self.current_tracon.apt_id) > 0:
            apt_idx = random.randint(0, len(self.current_tracon.apt_id) - 1)
            origin_apt = self.current_tracon.apt_id[apt_idx]
            origin_lat = self.current_tracon.apt_lat[apt_idx]
            origin_lon = self.current_tracon.apt_lon[apt_idx]
            
            if origin_apt in self.current_tracon.runway_id and len(self.current_tracon.runway_id[origin_apt]) > 0:
                rwy_idx = random.randint(0, len(self.current_tracon.runway_id[origin_apt]) - 1)
                origin_rwy = self.current_tracon.runway_id[origin_apt][rwy_idx]
                origin_lat = self.current_tracon.runway_thres_lat[origin_apt][rwy_idx]
                origin_lon = self.current_tracon.runway_thres_lon[origin_apt][rwy_idx]
                origin_hdg = self.current_tracon.runway_bearing[origin_apt][rwy_idx]
                origin_alt = self.current_tracon.apt_ele[apt_idx] + 500  # 500ft above airport
        
        # Initial speed
        origin_spd = 160  # kts
        
        # Generate exit fix at TRACON boundary
        exit_angle = random.uniform(0, 2*np.pi)
        exit_lat, exit_lon = geocalc.destination_by_bearing(
            self.current_tracon.ctr_lat, self.current_tracon.ctr_lon, 
            np.degrees(exit_angle), self.current_tracon.range
        )
        
        # Create BlueSky aircraft
        stack.stack(f"CRE {acf_type} {callsign} {origin_lat} {origin_lon} {origin_hdg} {origin_alt} {origin_spd}")
        
        # Create exit waypoint and set as destination
        wp_name = f"EXIT{callsign[-3:]}"
        stack.stack(f"DEFWPT {wp_name} {exit_lat} {exit_lon}")
        stack.stack(f"DEST {callsign} {wp_name}")
        
        # Get weight class
        acf_class = self.determine_aircraft_class(acf_type)
        
        # Create aircraft data
        aircraft = Aircraft(
            callsign=callsign,
            lat=origin_lat,
            lon=origin_lon,
            curr_alt=origin_alt,
            curr_cas=origin_spd,
            curr_trk=origin_hdg,
            curr_vs=0,
            acf_type=acf_type,
            acf_class=acf_class,
            intention="departure",
            target_wp_lat=exit_lat,
            target_wp_lon=exit_lon,
            target_wp_alt=10000,  # Default exit altitude
            under_ctrl=True,  # Departures start under control
            time_entered=sim.simt,
            time_stayed=0,
            time_last_command=sim.simt,
            time_since_last_command=0,
            status="Active"
        )
        
        # Calculate distance and track to target
        aircraft.dist_to_wpt = geocalc.distance_between(
            origin_lat, origin_lon, exit_lat, exit_lon
        )
        aircraft.trk_to_wpt = geocalc.bearing_from_to(
            origin_lat, origin_lon, exit_lat, exit_lon
        )
        
        # Add to tracking
        self.aircraft_data[callsign] = aircraft
        
        # Log creation
        scr.echo(f"ApproachBS: Generated departure {callsign} from {origin_apt if origin_apt else 'center'}")
        
        return callsign
    
    def add_aircraft(self, aircraft):
        """
        Add an existing Aircraft object to tracking.
        
        Args:
            aircraft: Aircraft object
            
        Returns:
            bool: Success flag
        """
        # Check if at maximum aircraft limit
        if len(self.aircraft_data) >= 50:
            return False
        
        # Check if callsign already exists
        if aircraft.callsign in self.aircraft_data:
            return False
        
        # Check if in valid position (if TRACON is defined)
        if self.current_tracon:
            dist = geocalc.distance_between(
                aircraft.lat, aircraft.lon,
                self.current_tracon.ctr_lat, self.current_tracon.ctr_lon
            )
            
            # Too far from TRACON center
            if dist > self.current_tracon.range + 10.1:
                return False
                
            # Altitude out of range
            if aircraft.curr_alt > self.current_tracon.max_alt + 5000:
                return False
                
            if aircraft.curr_alt < 0:
                return False
        
        # Create BlueSky aircraft
        stack.stack(f"CRE {aircraft.acf_type} {aircraft.callsign} {aircraft.lat} {aircraft.lon} {aircraft.curr_trk} {aircraft.curr_alt} {aircraft.curr_cas}")
        
        # Create waypoint for target if available
        if aircraft.target_wp_lat is not None and aircraft.target_wp_lon is not None:
            wp_name = f"WP{aircraft.callsign}"
            stack.stack(f"DEFWPT {wp_name} {aircraft.target_wp_lat} {aircraft.target_wp_lon}")
            stack.stack(f"DEST {aircraft.callsign} {wp_name}")
        
        # Add to tracking
        self.aircraft_data[aircraft.callsign] = aircraft
        
        # Update control status
        self._update_control_status(aircraft.callsign)
        
        return True
    
    def record_command(self, callsign):
        """
        Record that a command was issued to an aircraft.
        
        Args:
            callsign: Aircraft callsign
        """
        if callsign in self.aircraft_data:
            self.aircraft_data[callsign].time_last_command = sim.simt
            self.aircraft_data[callsign].time_since_last_command = 0.0
    
    def determine_aircraft_class(self, acf_type):
        """
        Determine the weight class of an aircraft type.
        
        Args:
            acf_type: Aircraft type code
            
        Returns:
            str: Weight class (light, medium, heavy, A380)
        """
        for cls, types in AIRCRAFT_CLASSES.items():
            if acf_type in types:
                return cls
        
        # Default to medium if unknown
        return "medium"
    
    def get_random_aircraft_type(self, acf_class):
        """
        Get a random aircraft type from a weight class.
        
        Args:
            acf_class: Weight class
            
        Returns:
            str: Aircraft type code
        """
        if acf_class in AIRCRAFT_CLASSES and AIRCRAFT_CLASSES[acf_class]:
            return random.choice(AIRCRAFT_CLASSES[acf_class])
        
        # Default to B738 if class not found
        return "B738"
    
    def get_scenario_tokens(self):
        """
        Convert the current TRACON state into token arrays for ML models.
        Matches the token format in preprocess.py.
        
        Returns:
            list: [tracon_token, restrict_tokens, aircraft_tokens]
        """
        # Check if we have a valid TRACON
        if not self.current_tracon:
            return [np.zeros(21), np.zeros((20, 20)), np.zeros((50, 20))]
        
        # Part 1: TRACON info (token0) - 21 values
        tracon_token = np.zeros(shape=(21))
        tracon_token[0] = self.current_tracon.ctr_lat
        tracon_token[1] = self.current_tracon.ctr_lon
        tracon_token[2] = self.current_tracon.range
        tracon_token[3] = self.current_tracon.min_alt
        tracon_token[4] = self.current_tracon.max_alt
        tracon_token[5] = len(self.aircraft_data)
        
        # Airport info: bearing from center, dist from center, elevation
        for i in range(min(len(self.current_tracon.apt_id), 5)):  # Up to 5 airports
            bfc = geocalc.bearing_from_to(
                self.current_tracon.ctr_lat, self.current_tracon.ctr_lon,
                self.current_tracon.apt_lat[i], self.current_tracon.apt_lon[i]
            )
            dfc = geocalc.distance_between(
                self.current_tracon.ctr_lat, self.current_tracon.ctr_lon,
                self.current_tracon.apt_lat[i], self.current_tracon.apt_lon[i]
            )
            tracon_token[6+i*3] = bfc
            tracon_token[7+i*3] = dfc
            tracon_token[8+i*3] = self.current_tracon.apt_ele[i]

        # Part 2: Restricted areas (token1 to token20) - 20x20 values
        res_tokens = np.zeros(shape=(20, 20))
        for i, area in enumerate(self.current_tracon.restrict):
            if i >= 20:  # Maximum 20 areas
                break
                
            # Area type (one hot): [1, 0] for circle, [0, 1] for polygon
            if area.area_type == "circle":
                res_tokens[i, 0] = 1
            else:  # polygon
                res_tokens[i, 1] = 1
                
            # Bottom and top altitude
            res_tokens[i, 2] = area.area_bottom_alt
            res_tokens[i, 3] = area.area_top_alt
            
            # Area arguments
            for j in range(min(len(area.area_args), 16)):  # Maximum 16 arguments
                res_tokens[i, 4+j] = area.area_args[j]

        # Part 3: Aircraft data (token 21 to token 70) - 50x20 values
        acf_tokens = np.zeros(shape=(50, 20))
        
        ac_idx = 0
        for callsign, acdata in self.aircraft_data.items():
            if ac_idx >= 50:  # Maximum 50 aircraft
                break
                
            # Intention: [1, 0] for arrival, [0, 1] for departure
            if acdata.intention == "arrival":
                acf_tokens[ac_idx, 0] = 1
            else:  # departure
                acf_tokens[ac_idx, 1] = 1
            
            # Control status
            acf_tokens[ac_idx, 2] = 1 if acdata.under_ctrl else 0
            
            # Time in airspace
            acf_tokens[ac_idx, 3] = acdata.time_stayed
            
            # Time since last command
            if acdata.time_since_last_command is not None:
                acf_tokens[ac_idx, 4] = acdata.time_since_last_command
            
            # LNAV mode
            acf_tokens[ac_idx, 5] = 1 if acdata.lnav_on else 0
            
            # Position
            acf_tokens[ac_idx, 6] = acdata.lat
            acf_tokens[ac_idx, 7] = acdata.lon
            
            # Bearing and distance from center
            acf_tokens[ac_idx, 8] = geocalc.bearing_from_to(
                self.current_tracon.ctr_lat, self.current_tracon.ctr_lon,
                acdata.lat, acdata.lon
            )
            acf_tokens[ac_idx, 9] = geocalc.distance_between(
                self.current_tracon.ctr_lat, self.current_tracon.ctr_lon,
                acdata.lat, acdata.lon
            )
            
            # Bearing and distance to waypoint
            if acdata.target_wp_lat is not None and acdata.target_wp_lon is not None:
                acf_tokens[ac_idx, 10] = acdata.trk_to_wpt
                acf_tokens[ac_idx, 11] = acdata.dist_to_wpt
                
                # Target waypoint altitude
                if acdata.target_wp_alt is not None:
                    acf_tokens[ac_idx, 12] = acdata.target_wp_alt
            
            # Current state
            acf_tokens[ac_idx, 13] = acdata.curr_alt
            acf_tokens[ac_idx, 14] = acdata.curr_cas
            acf_tokens[ac_idx, 15] = acdata.curr_trk
            acf_tokens[ac_idx, 16] = acdata.curr_vs
            
            # Target state
            acf_tokens[ac_idx, 17] = acdata.target_alt
            acf_tokens[ac_idx, 18] = acdata.target_cas
            acf_tokens[ac_idx, 19] = acdata.target_trk
            
            ac_idx += 1

        return [tracon_token, res_tokens, acf_tokens]
    
    def _log_data(self):
        """Log TRACON data for analysis."""
        try:
            current_time = sim.simt
            
            # Get logging handle if needed
            if self.log_handle is None:
                self.log_handle = datalog.get_logger('APPROACHBS')
                if self.log_handle is None:
                    return
            
            # Log data for each aircraft
            for callsign, acdata in self.aircraft_data.items():
                if callsign in traf.id:
                    # Format: time,callsign,intention,under_control,lat,lon,alt,cas,trk,vs,
                    #         target_alt,target_cas,target_trk,time_in_airspace,time_since_last_cmd,status
                    self.log_handle.write(
                        f"{current_time},{callsign},"
                        f"{acdata.intention},"
                        f"{1 if acdata.under_ctrl else 0},"
                        f"{acdata.lat},{acdata.lon},{acdata.curr_alt},"
                        f"{acdata.curr_cas},{acdata.curr_trk},{acdata.curr_vs},"
                        f"{acdata.target_alt},{acdata.target_cas},{acdata.target_trk},"
                        f"{acdata.time_stayed},{acdata.time_since_last_command},"
                        f"{acdata.status}"
                    )
        except Exception as e:
            print(f"Error logging ApproachBS data: {str(e)}")