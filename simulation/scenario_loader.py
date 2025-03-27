"""
ApproachBS Scenario Loader
-----------------------
This module provides functions to load and save TRACON scenarios in
both JSON and HDF5 formats, compatible with your existing file formats.
"""

import os
import json
import numpy as np
import h5py
from datetime import datetime
import copy

import bluesky as bs
from bluesky import stack, traf, sim, scr

from .tracon import Tracon
from .aircraft import Aircraft
from .restricted_area import RestrictedArea

def load_scenario_json(tracon_manager, filename):
    """
    Load a TRACON scenario from a JSON file.
    
    Args:
        tracon_manager: TraconManager instance
        filename: Path to JSON scenario file
        
    Returns:
        tuple: (success, message)
    """
    try:
        # Read JSON file
        with open(filename, 'r') as f:
            scenario_data = json.load(f)
        
        # Reset simulator and TRACON manager
        tracon_manager.reset()
        stack.stack("RESET")
        
        # Process TRACON data
        if 'tracon' in scenario_data:
            # Create TRACON object from dictionary
            tracon_dict = scenario_data['tracon']
            tracon = Tracon.from_dict(tracon_dict)
            
            # Set as current TRACON
            tracon_manager.set_current_tracon(tracon)
            
            # If no restricted areas in the data but they are defined in the dictionary,
            # parse them separately (for backwards compatibility)
            if len(tracon.restrict) == 0 and 'restrict' in tracon_dict:
                for area_dict in tracon_dict['restrict']:
                    restricted_area = RestrictedArea.from_dict(area_dict)
                    tracon.add_restricted_area(restricted_area)
            
            scr.echo(f"Loaded TRACON: {tracon.identifier}")
        else:
            return False, "No TRACON data found in file"
        
        # Process aircraft data
        if 'aircrafts' in scenario_data:
            for acf_dict in scenario_data['aircrafts']:
                # Create aircraft from dictionary
                aircraft = Aircraft.from_dict(acf_dict)
                
                # Create in BlueSky
                stack.stack(f"CRE {aircraft.acf_type} {aircraft.callsign} {aircraft.lat} {aircraft.lon} "
                           f"{aircraft.curr_trk} {aircraft.curr_alt} {aircraft.curr_cas}")
                
                # Set vertical speed if non-zero
                if aircraft.curr_vs != 0:
                    stack.stack(f"VS {aircraft.callsign} {aircraft.curr_vs}")
                
                # Set target values if different from current
                if aircraft.target_alt != aircraft.curr_alt:
                    stack.stack(f"ALT {aircraft.callsign} {aircraft.target_alt}")
                
                if aircraft.target_cas != aircraft.curr_cas:
                    stack.stack(f"SPD {aircraft.callsign} {aircraft.target_cas}")
                
                if aircraft.target_trk != aircraft.curr_trk:
                    stack.stack(f"HDG {aircraft.callsign} {aircraft.target_trk}")
                
                # Set destination waypoint if defined
                if aircraft.target_wp_lat is not None and aircraft.target_wp_lon is not None:
                    wp_name = f"WP{aircraft.callsign}"
                    stack.stack(f"DEFWPT {wp_name} {aircraft.target_wp_lat} {aircraft.target_wp_lon}")
                    stack.stack(f"DEST {aircraft.callsign} {wp_name}")
                
                # Add to manager's tracking
                tracon_manager.aircraft_data[aircraft.callsign] = aircraft
            
            scr.echo(f"Loaded {len(scenario_data['aircrafts'])} aircraft")
        
        # Set simulation time if provided
        if 'simt' in scenario_data:
            sim.simt = scenario_data['simt']
        
        # Activate TRACON mode
        tracon_manager.active = True
        
        return True, f"Successfully loaded scenario from {filename}"
    
    except Exception as e:
        return False, f"Error loading scenario: {str(e)}"

def load_scenario_hdf5(tracon_manager, filename):
    """
    Load a TRACON scenario from an HDF5 file.
    Compatible with the token format from your preprocessing.py.
    
    Args:
        tracon_manager: TraconManager instance
        filename: Path to HDF5 scenario file
        
    Returns:
        tuple: (success, message)
    """
    try:
        # Open HDF5 file
        with h5py.File(filename, 'r') as f:
            # Check if this is a tokenized dataset or a full scenario
            is_tokenized = 'tracon_token' in f and 'acfs_tokens' in f
            is_dataset = False
            
            # Dataset-type HDF5 (from your training data)
            if not is_tokenized:
                # Check if it's a dataset with multiple scenarios
                keys = list(f.keys())
                if keys and keys[0].isdigit():
                    is_dataset = True
                    # Take the first scenario by default (can be extended to select specific one)
                    scen_key = keys[0]
                    scenario_group = f[scen_key]
                    
                    # Extract token arrays
                    if 'scen' in scenario_group and 'areas' in scenario_group and 'acfs' in scenario_group:
                        tracon_token = scenario_group['scen'][:]
                        areas_token = scenario_group['areas'][:]
                        acfs_token = scenario_group['acfs'][:]
                        
                        # Process the token arrays
                        success = _process_token_arrays(tracon_manager, tracon_token, areas_token, acfs_token)
                        
                        if success:
                            return True, f"Successfully loaded scenario from dataset {filename}[{scen_key}]"
                        else:
                            return False, "Failed to process token arrays from dataset"
            
            # Direct token arrays
            if is_tokenized or 'tracon_token' in f:
                tracon_token = f['tracon_token'][:]
                areas_token = f['res_tokens'][:] if 'res_tokens' in f else f['areas'][:]
                acfs_token = f['acf_tokens'][:] if 'acf_tokens' in f else f['acfs'][:]
                
                # Process the token arrays
                success = _process_token_arrays(tracon_manager, tracon_token, areas_token, acfs_token)
                
                if success:
                    return True, f"Successfully loaded scenario from token arrays in {filename}"
                else:
                    return False, "Failed to process token arrays"
            
            return False, "Unsupported HDF5 file format"
    
    except Exception as e:
        return False, f"Error loading scenario from HDF5: {str(e)}"

def _process_token_arrays(tracon_manager, tracon_token, areas_token, acfs_token):
    """
    Process token arrays to create a TRACON scenario.
    Reverse of the tokenization in your preprocessing.py.
    
    Args:
        tracon_manager: TraconManager instance
        tracon_token: TRACON token array (shape=(21))
        areas_token: Restricted areas token array (shape=(20, 20) or (20, 52))
        acfs_token: Aircraft token array (shape=(50, 20) or (50, 15))
        
    Returns:
        bool: Success flag
    """
    try:
        # Reset simulator and TRACON manager
        tracon_manager.reset()
        stack.stack("RESET")
        
        # Parse TRACON token
        ctr_lat = tracon_token[0]
        ctr_lon = tracon_token[1]
        tracon_range = tracon_token[2]
        min_alt = tracon_token[3]
        max_alt = tracon_token[4]
        
        # Denormalize if needed (in normalized format, lat is divided by pi/2, range by 50)
        if abs(ctr_lat) <= 1.0:  # Normalized lat
            ctr_lat = ctr_lat * (np.pi/2) * (180/np.pi)  # Convert back to degrees
        
        if tracon_range <= 1.0:  # Normalized range
            tracon_range = tracon_range * 50  # Range in nm
        
        # Create TRACON object
        tracon = Tracon(
            identifier="LOADED_TRACON",
            ctr_lat=ctr_lat,
            ctr_lon=ctr_lon,
            tracon_range=tracon_range,
            elevation=0
        )
        
        # Set altitude limits
        tracon.min_alt = min_alt
        tracon.max_alt = max_alt
        
        # Parse airport information (if available)
        for i in range(5):  # Up to 5 airports in token
            if 6+i*3 < len(tracon_token) and tracon_token[6+i*3] != 0:
                bearing = tracon_token[6+i*3]
                distance = tracon_token[7+i*3]
                elevation = tracon_token[8+i*3]
                
                # Calculate position
                from .utils import geocalc
                apt_lat, apt_lon = geocalc.destination_by_bearing(
                    ctr_lat, ctr_lon, bearing, distance
                )
                
                # Add airport to TRACON
                apt_id = f"APT{i+1}"
                tracon.apt_id.append(apt_id)
                tracon.apt_lat.append(apt_lat)
                tracon.apt_lon.append(apt_lon)
                tracon.apt_ele.append(elevation)
        
        # Parse restricted areas
        for i in range(areas_token.shape[0]):
            if areas_token[i, 0] != 0 or areas_token[i, 1] != 0:
                area_type = "circle" if areas_token[i, 0] > 0 else "polygon"
                bottom_alt = areas_token[i, 2]
                top_alt = areas_token[i, 3]
                
                # Extract area arguments based on type
                if area_type == "circle":
                    # For circle: [center_lat, center_lon, radius]
                    center_lat = areas_token[i, 4]
                    
                    # Handle different token formats
                    if areas_token.shape[1] == 20:  # Original format
                        center_lon = areas_token[i, 5]
                        radius = areas_token[i, 6]
                        area_args = [center_lat, center_lon, radius]
                    else:  # Normalized format with separate absolute/relative
                        # Convert from normalized lat, lon_cos, lon_sin, radius
                        center_lat = center_lat * (np.pi/2) * (180/np.pi)  # Convert back to degrees
                        lon_cos = areas_token[i, 5]
                        lon_sin = areas_token[i, 6]
                        center_lon = np.degrees(np.arctan2(lon_sin, lon_cos))
                        radius = areas_token[i, 7] * 50  # Denormalize radius
                        area_args = [center_lat, center_lon, radius]
                
                elif area_type == "polygon":
                    # For polygon: extract vertices
                    area_args = []
                    
                    if areas_token.shape[1] == 20:  # Original format
                        # Extract up to 8 vertices (16 values)
                        for j in range(8):
                            if 4+j*2 < areas_token.shape[1] and areas_token[i, 4+j*2] != 0:
                                lat = areas_token[i, 4+j*2]
                                lon = areas_token[i, 4+j*2+1]
                                area_args.extend([lat, lon])
                    else:  # Normalized format with separate absolute/relative
                        # Extract up to 8 vertices (24 values, 3 per vertex)
                        for j in range(8):
                            if 4+j*3 < areas_token.shape[1] and areas_token[i, 4+j*3] != 0:
                                lat = areas_token[i, 4+j*3] * (np.pi/2) * (180/np.pi)  # Convert back to degrees
                                lon_cos = areas_token[i, 5+j*3]
                                lon_sin = areas_token[i, 6+j*3]
                                lon = np.degrees(np.arctan2(lon_sin, lon_cos))
                                area_args.extend([lat, lon])
                
                # Create and add restricted area
                area_id = f"R{i+1}"
                restricted_area = RestrictedArea(
                    area_id=area_id,
                    area_type=area_type,
                    bottom=bottom_alt,
                    top=top_alt,
                    area_args=area_args
                )
                
                tracon.add_restricted_area(restricted_area)
        
        # Set as current TRACON
        tracon_manager.set_current_tracon(tracon)
        
        # Parse aircraft tokens
        from .utils import geocalc
        for i in range(acfs_token.shape[0]):
            # Check if this is a valid aircraft (non-zero values)
            if (acfs_token[i, 0] != 0 or acfs_token[i, 1] != 0) and acfs_token[i, 6] != 0:
                # Determine intention
                intention = "arrival" if acfs_token[i, 0] > 0 else "departure"
                
                # Extract position and state
                if acfs_token.shape[1] == 20:  # Original format
                    lat = acfs_token[i, 6]
                    lon = acfs_token[i, 7]
                    curr_alt = acfs_token[i, 13]
                    curr_cas = acfs_token[i, 14]
                    curr_trk = acfs_token[i, 15]
                    curr_vs = acfs_token[i, 16]
                    target_alt = acfs_token[i, 17]
                    target_cas = acfs_token[i, 18]
                    target_trk = acfs_token[i, 19]
                else:  # Normalized format (shape 15)
                    # Denormalize values
                    lat_norm = acfs_token[i, 0]
                    lon_cos = acfs_token[i, 1]
                    lon_sin = acfs_token[i, 2]
                    lat = lat_norm * (np.pi/2) * (180/np.pi)  # Convert back to degrees
                    lon = np.degrees(np.arctan2(lon_sin, lon_cos))
                    
                    curr_alt = acfs_token[i, 6] * 20000  # Denormalize altitude
                    curr_cas = acfs_token[i, 7] * 250    # Denormalize speed
                    curr_trk = np.degrees(np.arctan2(acfs_token[i, 9], acfs_token[i, 8]))  # Convert from cos/sin
                    curr_vs = acfs_token[i, 10] * 3000   # Denormalize vertical speed
                    
                    target_alt = acfs_token[i, 11] * 20000  # Denormalize target altitude
                    target_cas = acfs_token[i, 12] * 250    # Denormalize target speed
                    target_trk = np.degrees(np.arctan2(acfs_token[i, 14], acfs_token[i, 13]))  # Convert from cos/sin
                
                # Generate callsign
                prefix = "ARR" if intention == "arrival" else "DEP"
                callsign = f"{prefix}{i+1:03d}"
                
                # Determine if under control
                under_ctrl = bool(acfs_token[i, 2])
                
                # Determine waypoint if available
                target_wp_lat = None
                target_wp_lon = None
                target_wp_alt = None
                
                if acfs_token.shape[1] == 20 and acfs_token[i, 10] != 0 and acfs_token[i, 11] != 0:
                    # Calculate waypoint position from bearing and distance
                    bearing = acfs_token[i, 10]
                    distance = acfs_token[i, 11]
                    target_wp_lat, target_wp_lon = geocalc.destination_by_bearing(lat, lon, bearing, distance)
                    target_wp_alt = acfs_token[i, 12] if acfs_token[i, 12] != 0 else None
                
                # Create aircraft in BlueSky
                stack.stack(f"CRE B738 {callsign} {lat} {lon} {curr_trk} {curr_alt} {curr_cas}")
                
                # Set vertical speed if non-zero
                if curr_vs != 0:
                    stack.stack(f"VS {callsign} {curr_vs}")
                
                # Set target values if different from current
                if target_alt != curr_alt:
                    stack.stack(f"ALT {callsign} {target_alt}")
                
                if target_cas != curr_cas:
                    stack.stack(f"SPD {callsign} {target_cas}")
                
                if target_trk != curr_trk:
                    stack.stack(f"HDG {callsign} {target_trk}")
                
                # Set destination waypoint if defined
                if target_wp_lat is not None and target_wp_lon is not None:
                    wp_name = f"WP{callsign}"
                    stack.stack(f"DEFWPT {wp_name} {target_wp_lat} {target_wp_lon}")
                    stack.stack(f"DEST {callsign} {wp_name}")
                
                # Create aircraft data object
                aircraft = Aircraft(
                    callsign=callsign,
                    lat=lat,
                    lon=lon,
                    curr_alt=curr_alt,
                    curr_cas=curr_cas,
                    curr_trk=curr_trk,
                    curr_vs=curr_vs,
                    acf_type="B738",  # Default type
                    acf_class="medium",  # Default class
                    target_alt=target_alt,
                    target_cas=target_cas,
                    target_trk=target_trk,
                    intention=intention,
                    target_wp_lat=target_wp_lat,
                    target_wp_lon=target_wp_lon,
                    target_wp_alt=target_wp_alt,
                    under_ctrl=under_ctrl,
                    time_entered=sim.simt if under_ctrl else None,
                    time_stayed=acfs_token[i, 3] if under_ctrl else 0,
                    time_last_command=sim.simt,
                    time_since_last_command=acfs_token[i, 4] if acfs_token[i, 4] != 0 else 0,
                    lnav_on=bool(acfs_token[i, 5])
                )
                
                # Add to manager's tracking
                tracon_manager.aircraft_data[callsign] = aircraft
        
        # Activate TRACON mode
        tracon_manager.active = True
        
        return True
    
    except Exception as e:
        scr.echo(f"Error processing token arrays: {str(e)}")
        return False

def save_scenario_json(tracon_manager, filename):
    """
    Save current TRACON scenario to a JSON file.
    
    Args:
        tracon_manager: TraconManager instance
        filename: Path to save JSON scenario file
        
    Returns:
        tuple: (success, message)
    """
    try:
        # Check if we have a TRACON defined
        if not tracon_manager.current_tracon:
            return False, "No TRACON area selected"
        
        # Create scenario data structure
        scenario_data = {
            'tracon': tracon_manager.current_tracon.to_dict(),
            'aircrafts': [],
            'simt': sim.simt,
        }
        
        # Add aircraft data
        for callsign, acdata in tracon_manager.aircraft_data.items():
            if callsign in traf.id:
                scenario_data['aircrafts'].append(acdata.to_dict())
        
        # Write to file
        with open(filename, 'w') as f:
            json.dump(scenario_data, f, indent=2)
        
        return True, f"Successfully saved scenario to {filename}"
    
    except Exception as e:
        return False, f"Error saving scenario: {str(e)}"

def save_scenario_hdf5(tracon_manager, filename):
    """
    Save current TRACON scenario to an HDF5 file in token format.
    
    Args:
        tracon_manager: TraconManager instance
        filename: Path to save HDF5 scenario file
        
    Returns:
        tuple: (success, message)
    """
    try:
        # Check if we have a TRACON defined
        if not tracon_manager.current_tracon:
            return False, "No TRACON area selected"
        
        # Get token representation
        tokens = tracon_manager.get_scenario_tokens()
        
        # Create HDF5 file
        with h5py.File(filename, 'w') as f:
            # Save tokens
            f.create_dataset('tracon_token', data=tokens[0])
            f.create_dataset('res_tokens', data=tokens[1])
            f.create_dataset('acf_tokens', data=tokens[2])
            
            # Save metadata
            f.attrs['simt'] = sim.simt
            f.attrs['num_acfs'] = len(tracon_manager.aircraft_data)
            f.attrs['created'] = datetime.now().strftime("%Y-%m-%d %H:%M:%S")
        
        return True, f"Successfully saved scenario to {filename}"
    
    except Exception as e:
        return False, f"Error saving scenario to HDF5: {str(e)}"