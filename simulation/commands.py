"""
ApproachBS Commands Module
-----------------------
This module registers and handles custom commands for the ApproachBS plugin.
"""

import numpy as np
import random
import os
import datetime

import bluesky as bs
from bluesky import stack, traf, sim, scr, tools

from .utils import geocalc

def register_commands(tracon_manager, scenario_generator):
    """
    Register all custom commands for the ApproachBS plugin.
    
    Args:
        tracon_manager: Instance of TraconManager
        scenario_generator: Instance of ScenarioGenerator
        
    Returns:
        dict: Dictionary of commands to be registered
    """
    # Command registry
    commands = {
        'APPROACHBS': [
            'APPROACHBS [ON/OFF]',
            '[on/off]',
            'Activate or deactivate ApproachBS mode',
            lambda *args: approachbs_mode(tracon_manager, *args)
        ],
        'APPROACHSETTING': [
            'APPROACHSETTING [AIRSPACE/SPAWN] name value',
            'txt,txt,float',
            'Set ApproachBS parameters',
            lambda *args: approachbs_setting(tracon_manager, *args)
        ],
        'APPROACHTRACON': [
            'APPROACHTRACON tracon_id',
            'txt',
            'Select a TRACON area by ID',
            lambda *args: select_tracon(tracon_manager, *args)
        ],
        'APPROACHSCENARIO': [
            'APPROACHSCENARIO gentype',
            'txt',
            'Generate scenario with specified density (e.g., MMLU)',
            lambda *args: generate_scenario(tracon_manager, scenario_generator, *args)
        ],
        'APPROACHGEN': [
            'APPROACHGEN [ARRIVAL/DEPARTURE] n',
            'txt,int',
            'Generate n arrival or departure aircraft',
            lambda *args: gen_aircraft(tracon_manager, *args)
        ],
        'APPROACHLOS': [
            'APPROACHLOS [acid]',
            '[acid]',
            'Generate aircraft with potential Loss of Separation',
            lambda *args: gen_los_aircraft(tracon_manager, scenario_generator, *args)
        ],
        'APPROACHINTENT': [
            'APPROACHINTENT acid [ARRIVAL/DEPARTURE] [waypoint/runway name]',
            'acid,txt,txt',
            'Set aircraft intent (arrival or departure) and destination',
            lambda *args: set_intent(tracon_manager, *args)
        ],
        'APPROACHINFO': [
            'APPROACHINFO [aircraft/ALL]',
            '[acid]',
            'Get ApproachBS information about aircraft',
            lambda *args: approachbs_info(tracon_manager, *args)
        ],
        'APPROACHCONFLICTS': [
            'APPROACHCONFLICTS',
            '',
            'Show current conflicts in TRACON',
            lambda *args: show_conflicts(tracon_manager, *args)
        ],
        'APPROACHSAVE': [
            'APPROACHSAVE filename',
            'txt',
            'Save current scenario to file (JSON or HDF5)',
            lambda *args: save_scenario(tracon_manager, *args)
        ],
        'APPROACHLOAD': [
            'APPROACHLOAD filename',
            'txt',
            'Load scenario from file (JSON or HDF5)',
            lambda *args: load_scenario(tracon_manager, *args)
        ],
        'APPROACHTOKEN': [
            'APPROACHTOKEN [SAVE filename]',
            '[txt,txt]',
            'Display or save current scenario tokens',
            lambda *args: token_scenario(tracon_manager, *args)
        ],
    }
    
    return commands

def approachbs_mode(tracon_manager, flag=None):
    """
    Set ApproachBS mode on or off.
    
    Args:
        tracon_manager: TraconManager instance
        flag: Mode flag (ON/OFF)
        
    Returns:
        tuple: (success, message)
    """
    if flag is None:
        return True, f"ApproachBS mode is currently {'ON' if tracon_manager.active else 'OFF'}"
    
    # Get value from flag string
    if flag.upper() in ('ON', 'TRUE', '1', 'YES'):
        if tracon_manager.current_tracon is None:
            return False, "No TRACON area selected. Use APPROACHTRACON to select one."
            
        tracon_manager.active = True
        return True, "ApproachBS mode activated"
    
    elif flag.upper() in ('OFF', 'FALSE', '0', 'NO'):
        tracon_manager.active = False
        return True, "ApproachBS mode deactivated"
    
    return False, f"Invalid flag {flag}, use ON or OFF"

def approachbs_setting(tracon_manager, param=None, name=None, value=None):
    """
    Set ApproachBS parameters.
    
    Args:
        tracon_manager: TraconManager instance
        param: Parameter group (AIRSPACE/SPAWN)
        name: Parameter name
        value: Parameter value
        
    Returns:
        tuple: (success, message)
    """
    if not param or not name or value is None:
        # Return current settings
        settings_str = "Current ApproachBS settings:\n"
        
        if tracon_manager.current_tracon:
            settings_str += f"  AIRSPACE identifier: {tracon_manager.current_tracon.identifier}\n"
            settings_str += f"  AIRSPACE radius: {tracon_manager.current_tracon.range:.1f} nm\n"
            settings_str += f"  AIRSPACE center: ({tracon_manager.current_tracon.ctr_lat:.4f}, {tracon_manager.current_tracon.ctr_lon:.4f})\n"
            settings_str += f"  AIRSPACE min_alt: {tracon_manager.current_tracon.min_alt} ft\n"
            settings_str += f"  AIRSPACE max_alt: {tracon_manager.current_tracon.max_alt} ft\n"
        else:
            settings_str += "  No TRACON area selected\n"
            
        settings_str += f"  SPAWN arrival_rate: {tracon_manager.arrival_rate:.2f} aircraft/min\n"
        settings_str += f"  SPAWN departure_rate: {tracon_manager.departure_rate:.2f} aircraft/min\n"
        
        return True, settings_str
    
    param = param.upper()
    if param == 'AIRSPACE':
        # Check if TRACON is selected
        if not tracon_manager.current_tracon:
            return False, "No TRACON area selected. Use APPROACHTRACON to select one."
            
        if name.upper() == 'RADIUS':
            tracon_manager.current_tracon.range = float(value)
            return True, f"TRACON radius set to {value} nm"
        elif name.upper() in ('CENTER', 'CENTRE'):
            try:
                # Expecting a comma-separated lat,lon pair
                lat, lon = value.split(',')
                tracon_manager.current_tracon.ctr_lat = float(lat)
                tracon_manager.current_tracon.ctr_lon = float(lon)
                return True, f"TRACON center set to ({lat}, {lon})"
            except:
                return False, "Invalid center format, use 'lat,lon'"
        elif name.upper() == 'MIN_ALT':
            tracon_manager.current_tracon.min_alt = float(value)
            return True, f"TRACON minimum altitude set to {value} ft"
        elif name.upper() == 'MAX_ALT':
            tracon_manager.current_tracon.max_alt = float(value)
            return True, f"TRACON maximum altitude set to {value} ft"
        else:
            return False, f"Unknown AIRSPACE parameter: {name}"
    
    elif param == 'SPAWN':
        if name.upper() == 'ARRIVAL_RATE':
            tracon_manager.arrival_rate = float(value)
            return True, f"Arrival rate set to {value} aircraft/min"
        elif name.upper() == 'DEPARTURE_RATE':
            tracon_manager.departure_rate = float(value)
            return True, f"Departure rate set to {value} aircraft/min"
        elif name.upper() == 'MIN_SEPARATION':
            tracon_manager.min_aircraft_separation = float(value) * tools.aero.nm
            return True, f"Minimum aircraft separation set to {value} nm"
        else:
            return False, f"Unknown SPAWN parameter: {name}"
    
    return False, f"Unknown parameter group: {param}, use AIRSPACE or SPAWN"

def select_tracon(tracon_manager, tracon_id=None):
    """
    Select a TRACON area by ID.
    
    Args:
        tracon_manager: TraconManager instance
        tracon_id: TRACON identifier
        
    Returns:
        tuple: (success, message)
    """
    if tracon_id is None:
        # List available TRACONs
        if not tracon_manager.raw_tracons:
            return False, "No TRACONs available. Please check configuration."
            
        tracon_list = "\nAvailable TRACONs:\n"
        for i, tracon in enumerate(tracon_manager.raw_tracons):
            tracon_list += f"  {i+1}. {tracon.identifier}\n"
            
        return True, tracon_list
    
    # Try to find TRACON by ID
    tracon = tracon_manager.get_raw_tracon_by_id(tracon_id)
    if not tracon:
        return False, f"TRACON '{tracon_id}' not found. Use APPROACHTRACON without arguments to see available TRACONs."
    
    # Create a copy of the TRACON and use it
    import copy
    tracon_manager.set_current_tracon(copy.deepcopy(tracon))
    
    # Reset aircraft tracking
    tracon_manager.reset_aircraft_tracking()
    
    return True, f"Selected TRACON area: {tracon_id}"

def generate_scenario(tracon_manager, scenario_generator, gentype=None):
    """
    Generate a complete scenario.
    
    Args:
        tracon_manager: TraconManager instance
        scenario_generator: ScenarioGenerator instance
        gentype: Generation type code
        
    Returns:
        tuple: (success, message)
    """
    if not gentype:
        # Show help if no type provided
        help_str = """
Scenario generation types:
  First character: Restricted area density
    S: Sparse (0-5 areas)
    M: Medium (6-12 areas)
    D: Dense (13-20 areas)
    F: Full (20 areas)
    R: Random (0-20 areas)
    
  Second character: Aircraft density
    S: Sparse (1-10 aircraft)
    M: Medium (11-30 aircraft)
    D: Dense (31-50 aircraft)
    F: Full (50 aircraft)
    R: Random (0-50 aircraft)
    
  Optional flags:
    L: Include Loss of Separation (conflict) aircraft
    U: Allow uncontrolled area aircraft

Examples:
  APPROACHSCENARIO MM    - Medium restricted areas, medium aircraft
  APPROACHSCENARIO SFL   - Sparse restricted areas, full aircraft, with conflicts
  APPROACHSCENARIO DRU   - Dense restricted areas, random aircraft, uncontrolled area
"""
        return True, help_str
    
    # Validate generation type
    if len(gentype) < 2:
        return False, "Generation type must be at least 2 characters. See APPROACHSCENARIO without arguments for help."
    
    # Check first two characters
    valid_density_codes = ['S', 'M', 'D', 'F', 'R']
    if gentype[0].upper() not in valid_density_codes or gentype[1].upper() not in valid_density_codes:
        return False, f"Invalid density code. Must be one of: {', '.join(valid_density_codes)}"
    
    # Generate the scenario
    success = scenario_generator.generate_scenario(gentype.upper())
    
    if success:
        tracon_manager.active = True
        return True, f"Generated scenario with type: {gentype}"
    else:
        return False, "Failed to generate scenario"

def gen_aircraft(tracon_manager, ac_type=None, count=1):
    """
    Generate new aircraft in the simulation.
    
    Args:
        tracon_manager: TraconManager instance
        ac_type: Type of aircraft to generate (ARRIVAL/DEPARTURE)
        count: Number of aircraft to generate
        
    Returns:
        tuple: (success, message)
    """
    if not tracon_manager.current_tracon:
        return False, "No TRACON area selected. Use APPROACHTRACON to select one."
    
    if not ac_type or ac_type.upper() not in ('ARRIVAL', 'DEPARTURE'):
        return False, "Specify ARRIVAL or DEPARTURE"
    
    try:
        count = int(count)
    except:
        return False, "Invalid count, must be an integer"
    
    if count <= 0:
        return False, "Count must be greater than zero"
    
    is_arrival = ac_type.upper() == 'ARRIVAL'
    
    # Generate aircraft
    created = []
    for _ in range(count):
        callsign = tracon_manager.generate_aircraft(is_arrival=is_arrival)
        if callsign:
            created.append(callsign)
    
    return True, f"Generated {len(created)} {ac_type.lower()} aircraft: {', '.join(created)}"

def gen_los_aircraft(tracon_manager, scenario_generator, acid=None):
    """
    Generate an aircraft with potential Loss of Separation.
    
    Args:
        tracon_manager: TraconManager instance
        scenario_generator: ScenarioGenerator instance
        acid: Aircraft ID to create conflict with (random if None)
        
    Returns:
        tuple: (success, message)
    """
    if not tracon_manager.current_tracon:
        return False, "No TRACON area selected. Use APPROACHTRACON to select one."
    
    # If no specific aircraft provided, pick a random one
    if acid is None or acid not in tracon_manager.aircraft_data:
        if tracon_manager.aircraft_data:
            acid = random.choice(list(tracon_manager.aircraft_data.keys()))
        else:
            # No existing aircraft, fall back to creating a random one
            return gen_aircraft(tracon_manager, "ARRIVAL", 1)
    
    # Generate aircraft with potential conflict
    success = scenario_generator.generate_los_aircraft(
        existing_acf_id=acid,
        trkchange=random.getrandbits(1) == 1,
        altchange=random.getrandbits(1) == 1,
        spdchange=random.getrandbits(1) == 1,
        unctrl_area=True
    )
    
    if success:
        return True, f"Generated aircraft with potential conflict with {acid}"
    else:
        return False, "Failed to generate aircraft with potential conflict"

def set_intent(tracon_manager, acid=None, intent=None, destination=None):
    """
    Set aircraft intent (arrival/departure) and destination.
    
    Args:
        tracon_manager: TraconManager instance
        acid: Aircraft identifier
        intent: Intent (ARRIVAL/DEPARTURE)
        destination: Destination waypoint or runway
        
    Returns:
        tuple: (success, message)
    """
    if acid not in traf.id:
        return False, f"Aircraft {acid} not found"
    
    if not intent or intent.upper() not in ('ARRIVAL', 'DEPARTURE'):
        return False, "Specify ARRIVAL or DEPARTURE"
    
    # Get aircraft data
    if acid in tracon_manager.aircraft_data:
        acdata = tracon_manager.aircraft_data[acid]
        
        # Update intent
        acdata.intention = "arrival" if intent.upper() == 'ARRIVAL' else "departure"
        
        # Update destination if provided
        if destination:
            # For arrivals, destination should be a runway
            if acdata.intention == "arrival":
                # Check if this is an airport/runway designation
                if '/' in destination:
                    apt, rwy = destination.split('/')
                    
                    # Check if current TRACON has this airport/runway
                    if tracon_manager.current_tracon and apt in tracon_manager.current_tracon.apt_id:
                        apt_idx = tracon_manager.current_tracon.apt_id.index(apt)
                        rwy_idx = tracon_manager.current_tracon.runway_id[apt].index(rwy) if rwy in tracon_manager.current_tracon.runway_id[apt] else -1
                        
                        if rwy_idx >= 0:
                            # Set runway as destination
                            acdata.target_wp_lat = tracon_manager.current_tracon.runway_thres_lat[apt][rwy_idx]
                            acdata.target_wp_lon = tracon_manager.current_tracon.runway_thres_lon[apt][rwy_idx]
                            acdata.target_wp_alt = tracon_manager.current_tracon.apt_ele[apt_idx]
                            
                            # Update waypoint data
                            acdata.update_dist_to_wpt()
                            acdata.update_trk_to_wpt()
                            
                            # Update BlueSky destination
                            stack.stack(f"DEST {acid} {destination}")
                            
                            # Record command
                            tracon_manager.record_command(acid)
                            
                            return True, f"Set {acid} intent to {intent} with destination {destination}"
                
                return False, f"Invalid destination format for arrival: {destination}"
            
            # For departures, destination should be a waypoint/fix
            else:
                # Create exit waypoint at TRACON boundary in this direction
                if tracon_manager.current_tracon:
                    # Parse bearing if given as a number
                    try:
                        exit_bearing = float(destination)
                        
                        # Create exit point on TRACON boundary
                        exit_lat, exit_lon = geocalc.destination_by_bearing(
                            tracon_manager.current_tracon.ctr_lat,
                            tracon_manager.current_tracon.ctr_lon,
                            exit_bearing,
                            tracon_manager.current_tracon.range
                        )
                        
                        # Create waypoint
                        wp_name = f"EXIT{acid[-3:]}"
                        stack.stack(f"DEFWPT {wp_name} {exit_lat} {exit_lon}")
                        
                        # Set as destination
                        acdata.target_wp_lat = exit_lat
                        acdata.target_wp_lon = exit_lon
                        acdata.target_wp_alt = 10000  # Default exit altitude
                        
                        # Update waypoint data
                        acdata.update_dist_to_wpt()
                        acdata.update_trk_to_wpt()
                        
                        # Update BlueSky destination
                        stack.stack(f"DEST {acid} {wp_name}")
                        
                        # Record command
                        tracon_manager.record_command(acid)
                        
                        return True, f"Set {acid} intent to {intent} with exit bearing {exit_bearing}°"
                    except ValueError:
                        # Not a bearing, treat as waypoint name
                        pass
                
                # For existing waypoint, set as destination
                stack.stack(f"DEST {acid} {destination}")
                
                # Reset target waypoint (will be updated from actual dest in next cycle)
                acdata.target_wp_lat = None
                acdata.target_wp_lon = None
                
                # Record command
                tracon_manager.record_command(acid)
                
                return True, f"Set {acid} intent to {intent} with destination {destination}"
        
        # Record command
        tracon_manager.record_command(acid)
        
        return True, f"Set {acid} intent to {intent}"
    
    return False, f"No ApproachBS data for {acid}"

def approachbs_info(tracon_manager, acid=None):
    """
    Get ApproachBS-specific information about aircraft.
    
    Args:
        tracon_manager: TraconManager instance
        acid: Aircraft identifier (optional)
        
    Returns:
        tuple: (success, message)
    """
    if acid is None or acid.upper() == 'ALL':
        # Return info for all aircraft
        if len(tracon_manager.aircraft_data) == 0:
            return True, "No aircraft in ApproachBS tracking"
        
        info_str = "ApproachBS Aircraft Information:\n"
        info_str += f"{'Callsign':<8} {'Intent':<10} {'Control':<8} {'Time In':<10} {'Last Cmd':<10} {'Status':<15}\n"
        
        for callsign, acdata in tracon_manager.aircraft_data.items():
            if callsign in traf.id:  # Only show aircraft that still exist
                info_str += f"{callsign:<8} "
                info_str += f"{acdata.intention:<10} "
                info_str += f"{'YES' if acdata.under_ctrl else 'NO':<8} "
                info_str += f"{acdata.time_stayed:<10.1f} "
                info_str += f"{acdata.time_since_last_command:<10.1f} "
                info_str += f"{acdata.status:<15}\n"
        
        return True, info_str
    
    # Return info for specific aircraft
    if acid not in traf.id:
        return False, f"Aircraft {acid} not found"
    
    if acid not in tracon_manager.aircraft_data:
        return False, f"No ApproachBS data for {acid}"
    
    acdata = tracon_manager.aircraft_data[acid]
    
    info_str = f"ApproachBS information for {acid}:\n"
    info_str += f"  Type: {acdata.acf_type} ({acdata.acf_class})\n"
    info_str += f"  Intent: {acdata.intention}\n"
    info_str += f"  Under TRACON control: {'Yes' if acdata.under_ctrl else 'No'}\n"
    
    if acdata.target_wp_lat is not None and acdata.target_wp_lon is not None:
        info_str += f"  Target waypoint: ({acdata.target_wp_lat:.4f}, {acdata.target_wp_lon:.4f})\n"
        if acdata.target_wp_alt is not None:
            info_str += f"  Target waypoint altitude: {acdata.target_wp_alt} ft\n"
        info_str += f"  Distance to target: {acdata.dist_to_wpt:.1f} nm\n"
        info_str += f"  Track to target: {acdata.trk_to_wpt:.1f}°\n"
    
    info_str += f"  Current altitude: {acdata.curr_alt} ft\n"
    info_str += f"  Current speed: {acdata.curr_cas} kts\n"
    info_str += f"  Current track: {acdata.curr_trk}°\n"
    info_str += f"  Current vertical speed: {acdata.curr_vs} fpm\n"
    
    info_str += f"  Target altitude: {acdata.target_alt} ft\n"
    info_str += f"  Target speed: {acdata.target_cas} kts\n"
    info_str += f"  Target track: {acdata.target_trk}°\n"
    
    if acdata.under_ctrl:
        info_str += f"  Time in airspace: {acdata.time_stayed:.1f} seconds\n"
    
    info_str += f"  Time since last command: {acdata.time_since_last_command:.1f} seconds\n"
    info_str += f"  Status: {acdata.status}\n"
    
    # Check if in any conflicts
    if tracon_manager.conflict.size > 0:
        acid_idx = list(tracon_manager.aircraft_data.keys()).index(acid)
        
        # Check for TRACON boundary conflict
        if tracon_manager.conflict[acid_idx, 0] == 1:
            info_str += "  CONFLICT: Outside TRACON boundary\n"
        
        # Check for restricted area conflicts
        for i in range(len(tracon_manager.current_tracon.restrict)):
            if i >= 20:  # Maximum 20 restricted areas
                break
                
            if tracon_manager.conflict[acid_idx, i+1] == 1:
                info_str += f"  CONFLICT: In restricted area {tracon_manager.current_tracon.restrict[i].area_id}\n"
        
        # Check for conflicts with other aircraft
        for i, other_acid in enumerate(tracon_manager.aircraft_data.keys()):
            if i >= 50:  # Maximum 50 aircraft
                break
                
            if other_acid != acid and tracon_manager.conflict[acid_idx, i+21] == 1:
                info_str += f"  CONFLICT: Loss of separation with {other_acid}\n"
    
    return True, info_str

def show_conflicts(tracon_manager):
    """
    Show current conflicts in TRACON.
    
    Args:
        tracon_manager: TraconManager instance
        
    Returns:
        tuple: (success, message)
    """
    if not tracon_manager.current_tracon:
        return False, "No TRACON area selected. Use APPROACHTRACON to select one."
    
    # Check if we have any aircraft
    if not tracon_manager.aircraft_data:
        return True, "No aircraft in simulation"
    
    # Ensure conflict matrix is up to date
    tracon_manager._update_conflict_detection()
    
    # Count conflicts
    boundary_conflicts = []
    restrict_conflicts = []
    separation_conflicts = []
    
    for i, acid in enumerate(tracon_manager.aircraft_data.keys()):
        if i >= 50:  # Maximum 50 aircraft
            break
            
        # Check for TRACON boundary conflict
        if tracon_manager.conflict[i, 0] == 1:
            boundary_conflicts.append(acid)
        
        # Check for restricted area conflicts
        for j in range(len(tracon_manager.current_tracon.restrict)):
            if j >= 20:  # Maximum 20 restricted areas
                break
                
            if tracon_manager.conflict[i, j+1] == 1:
                restrict_conflicts.append((acid, tracon_manager.current_tracon.restrict[j].area_id))
        
        # Check for conflicts with other aircraft
        for j, other_acid in enumerate(tracon_manager.aircraft_data.keys()):
            if j >= 50:  # Maximum 50 aircraft
                break
                
            if other_acid != acid and tracon_manager.conflict[i, j+21] == 1:
                # Only count each pair once
                if i < j:
                    separation_conflicts.append((acid, other_acid))
    
    # Build report
    report = "ApproachBS Conflict Report:\n"
    
    if not boundary_conflicts and not restrict_conflicts and not separation_conflicts:
        report += "No conflicts detected\n"
    else:
        if boundary_conflicts:
            report += "\nAircraft outside TRACON boundary:\n"
            for acid in boundary_conflicts:
                report += f"  {acid}\n"
        
        if restrict_conflicts:
            report += "\nAircraft in restricted areas:\n"
            for acid, area_id in restrict_conflicts:
                report += f"  {acid} in {area_id}\n"
        
        if separation_conflicts:
            report += "\nLoss of separation conflicts:\n"
            for acid1, acid2 in separation_conflicts:
                report += f"  {acid1} and {acid2}\n"
    
    return True, report

def save_scenario(tracon_manager, filename=None):
    """
    Save current scenario to file.
    
    Args:
        tracon_manager: TraconManager instance
        filename: Path to save scenario file
        
    Returns:
        tuple: (success, message)
    """
    if not tracon_manager.current_tracon:
        return False, "No TRACON area selected. Use APPROACHTRACON to select one."
    
    if not filename:
        return False, "Filename required"
    
    # Make sure output directory exists
    os.makedirs(os.path.dirname(os.path.abspath(filename)) if os.path.dirname(filename) else 'output/approachbs', exist_ok=True)
    
    # Determine file type and save accordingly
    from .scenario_loader import save_scenario_json, save_scenario_hdf5
    
    if filename.lower().endswith('.json'):
        success, message = save_scenario_json(tracon_manager, filename)
    elif filename.lower().endswith('.h5') or filename.lower().endswith('.hdf5'):
        success, message = save_scenario_hdf5(tracon_manager, filename)
    else:
        # Default to JSON if no extension
        success, message = save_scenario_json(tracon_manager, filename + '.json')
    
    return success, message

def load_scenario(tracon_manager, filename=None):
    """
    Load scenario from file.
    
    Args:
        tracon_manager: TraconManager instance
        filename: Path to scenario file
        
    Returns:
        tuple: (success, message)
    """
    if not filename:
        return False, "Filename required"
    
    # Check if file exists
    if not os.path.exists(filename):
        return False, f"File not found: {filename}"
    
    # Determine file type and load accordingly
    from .scenario_loader import load_scenario_json, load_scenario_hdf5
    
    if filename.lower().endswith('.json'):
        success, message = load_scenario_json(tracon_manager, filename)
    elif filename.lower().endswith('.h5') or filename.lower().endswith('.hdf5'):
        success, message = load_scenario_hdf5(tracon_manager, filename)
    else:
        return False, "Unsupported file format. Use .json or .h5/.hdf5"
    
    return success, message

def token_scenario(tracon_manager, arg1=None, arg2=None):
    """
    Display or save current scenario tokens.
    
    Args:
        tracon_manager: TraconManager instance
        arg1: Optional 'SAVE' command
        arg2: Optional filename if saving
        
    Returns:
        tuple: (success, message)
    """
    if not tracon_manager.current_tracon:
        return False, "No TRACON area selected. Use APPROACHTRACON to select one."
    
    # Get token representation
    tokens = tracon_manager.get_scenario_tokens()
    
    # Check if we're saving to file
    if arg1 and arg1.upper() == 'SAVE':
        if not arg2:
            return False, "Filename required when using SAVE"
        
        # Make sure output directory exists
        os.makedirs(os.path.dirname(os.path.abspath(arg2)) if os.path.dirname(arg2) else 'output/approachbs', exist_ok=True)
        
        try:
            import numpy as np
            import h5py
            
            with h5py.File(arg2, 'w') as f:
                f.create_dataset('tracon_token', data=tokens[0])
                f.create_dataset('res_tokens', data=tokens[1])
                f.create_dataset('acf_tokens', data=tokens[2])
                
                # Add metadata
                f.attrs['simt'] = sim.simt
                f.attrs['num_acfs'] = len(tracon_manager.aircraft_data)
                f.attrs['created'] = datetime.now().strftime("%Y-%m-%d %H:%M:%S")
            
            return True, f"Saved token representation to {arg2}"
        except Exception as e:
            return False, f"Error saving tokens: {str(e)}"
    
    # Display token summary
    summary = "Token Representation Summary:\n"
    summary += f"  TRACON token shape: {tokens[0].shape}\n"
    summary += f"  Restricted areas token shape: {tokens[1].shape}\n"
    summary += f"  Aircraft token shape: {tokens[2].shape}\n"
    
    # Show first few values of each
    summary += "\nTRACON token (first 10 values):\n"
    summary += str(tokens[0][:10]) + "\n"
    
    summary += "\nRestricted areas (first area, first 10 values):\n"
    if tokens[1].size > 0:
        summary += str(tokens[1][0, :10]) + "\n"
    else:
        summary += "No restricted areas\n"
    
    summary += "\nAircraft tokens (first aircraft, first 10 values):\n"
    if tokens[2].size > 0 and len(tracon_manager.aircraft_data) > 0:
        summary += str(tokens[2][0, :10]) + "\n"
    else:
        summary += "No aircraft\n"
    
    summary += "\nUse 'APPROACHTOKEN SAVE filename.h5' to save the full token representation."
    
    return True, summary