"""
ApproachBS Core Plugin for BlueSky
---------------------------------
This module contains the core functionality for the ApproachBS plugin,
handling plugin initialization, hooks, and core operations.
"""

import numpy as np
import random
import time
from datetime import datetime
import itertools
import os
import json

import bluesky as bs
from bluesky import stack, traf, sim, scr, tools
from bluesky.tools.aero import ft, kts, nm, fpm
from bluesky.tools import datalog

# Import local modules
from .tracon_manager import TraconManager
from .commands import register_commands
from .scenario_generator import ScenarioGenerator

# Global manager instance
tracon_manager = None
scenario_generator = None

def init_plugin():
    """Initialize the ApproachBS plugin."""
    # Configuration parameters
    config = {
        'plugin_name': 'APPROACHBS',
        'plugin_type': 'sim',
        'update_interval': 1.0,  # Update interval in seconds
        'update': update,
        'reset': reset,
        'preupdate': preupdate
    }
    
    # Initialize manager singletons
    global tracon_manager, scenario_generator
    tracon_manager = TraconManager()
    scenario_generator = ScenarioGenerator(tracon_manager)
    
    # Register custom commands
    config['commands'] = register_commands(tracon_manager, scenario_generator)
    
    # Set up data logging
    setup_logging()
    
    # Make sure the output directories exist
    os.makedirs('output/approachbs', exist_ok=True)
    
    # Load raw TRACON definitions
    load_raw_tracons()
    
    return config

def setup_logging():
    """Set up data logging for ApproachBS operations."""
    with datalog.crelog('APPROACHBS', 'LogAPPROACHBS.csv', datalog.CSV) as tracon_log:
        fields = ['time', 'callsign', 'intention', 'under_control',
                  'lat', 'lon', 'alt', 'cas', 'trk', 'vs',
                  'target_alt', 'target_cas', 'target_trk',
                  'time_in_airspace', 'time_since_last_cmd', 'status']
        tracon_log.write(','.join(fields))

def load_raw_tracons():
    """Load raw TRACON definitions from embedded data."""
    # Use raw TRACON definitions from traconlist.py
    global tracon_manager
    
    # Check if we have a custom TRACON file
    custom_file = os.path.join('output', 'approachbs', 'rawtracon.json')
    if os.path.exists(custom_file):
        try:
            with open(custom_file, 'r') as f:
                raw_tracons = json.load(f)
            tracon_manager.load_raw_tracons(raw_tracons)
            scr.echo(f"Loaded {len(raw_tracons)} custom TRACONs from {custom_file}")
            return
        except Exception as e:
            scr.echo(f"Error loading custom TRACONs: {str(e)}")
    
    # Otherwise, load the built-in TRACONs
    tracons = create_default_tracons()
    tracon_manager.load_raw_tracons(tracons)
    scr.echo(f"Loaded {len(tracons)} default TRACONs")
    
    # Save them to the custom file for future reference and editing
    os.makedirs(os.path.dirname(custom_file), exist_ok=True)
    try:
        tracon_dicts = [tracon.to_dict() for tracon in tracons]
        with open(custom_file, 'w') as f:
            json.dump(tracon_dicts, f, indent=4)
        scr.echo(f"Saved default TRACONs to {custom_file}")
    except Exception as e:
        scr.echo(f"Error saving default TRACONs: {str(e)}")

def create_default_tracons():
    """Create default TRACON definitions from embedded data."""
    from .tracon import Tracon
    
    # These match the definitions in traconlist.py
    tracons = [
        Tracon(identifier="KSFO RAW1", ctr_lat=37.619, ctr_lon=-122.375, tracon_range=40, elevation=0, apt_id=["KSFO", "KOAK", "KSJC"]),
        Tracon(identifier="KSFO RAW2", ctr_lat=37.5, ctr_lon=-122.23, tracon_range=30, elevation=0, apt_id=["KSFO", "KOAK", "KSJC"]),
        Tracon(identifier="KLAX RAW1", ctr_lat=33.943, ctr_lon=-118.408, tracon_range=40, elevation=100, apt_id=["KLAX", "KSNA", "KLGB"]),
        Tracon(identifier="KLAX RAW2", ctr_lat=33.943, ctr_lon=-118.408, tracon_range=30, elevation=100, apt_id=["KLAX", "KLGB"]),
        Tracon(identifier="KDEN RAW", ctr_lat=39.862, ctr_lon=-104.673, tracon_range=20, elevation=5400, apt_id=["KDEN"]),
        Tracon(identifier="KJFK RAW1", ctr_lat=40.64, ctr_lon=-73.779, tracon_range=30, elevation=0, apt_id=["KJFK", "KEWR", "KLGA"]),
        Tracon(identifier="KJFK RAW2", ctr_lat=40.7, ctr_lon=-74.0, tracon_range=20, elevation=0, apt_id=["KJFK", "KEWR", "KLGA"]),
        Tracon(identifier="KORD RAW", ctr_lat=41.979, ctr_lon=-87.905, tracon_range=30, elevation=700, apt_id=["KORD", "KMDW"]),
        Tracon(identifier="EGLL RAW1", ctr_lat=51.477, ctr_lon=-0.461, tracon_range=30, elevation=100, apt_id=["EGLL", "EGLC", "EGKK", "EGKB"]),
        Tracon(identifier="EGLL RAW2", ctr_lat=51.477, ctr_lon=-0.461, tracon_range=40, elevation=100, apt_id=["EGLL", "EGLC", "EGKK", "EGKB"]),
        Tracon(identifier="EGLL RAW3", ctr_lat=51.36, ctr_lon=-0.24, tracon_range=20, elevation=100, apt_id=["EGLL", "EGLC", "EGKK", "EGKB"]),
        Tracon(identifier="EGLL RAW4", ctr_lat=51.36, ctr_lon=-0.24, tracon_range=30, elevation=100, apt_id=["EGLL", "EGLC", "EGKK", "EGKB"]),
        Tracon(identifier="EGLL RAW5", ctr_lat=51.36, ctr_lon=-0.24, tracon_range=40, elevation=100, apt_id=["EGLL", "EGLC", "EGKK", "EGKB"]),
        Tracon(identifier="EIDW RAW", ctr_lat=53.421, ctr_lon=-6.27, tracon_range=20, elevation=200, apt_id=["EIDW"]),
        Tracon(identifier="LFPG RAW", ctr_lat=49.013, ctr_lon=2.55, tracon_range=30, elevation=400, apt_id=["LFPG", "LFPB", "LFPO"]),
        Tracon(identifier="LIMC RAW", ctr_lat=45.631, ctr_lon=8.728, tracon_range=30, elevation=800, apt_id=["LIMC", "LIML"]),
        Tracon(identifier="EDDF RAW1", ctr_lat=50.026, ctr_lon=8.543, tracon_range=20, elevation=400, apt_id=["EDDF"]),
        Tracon(identifier="EDDF RAW2", ctr_lat=50.026, ctr_lon=8.543, tracon_range=30, elevation=400, apt_id=["EDDF"]),
        Tracon(identifier="EDDF RAW3", ctr_lat=50.026, ctr_lon=8.543, tracon_range=40, elevation=400, apt_id=["EDDF"]),
        Tracon(identifier="OMDB RAW", ctr_lat=25.253, ctr_lon=55.364, tracon_range=30, elevation=100, apt_id=["OMDB"]),
        Tracon(identifier="VIDP RAW", ctr_lat=28.567, ctr_lon=77.103, tracon_range=30, elevation=800, apt_id=["VIDP", "VIDX"]),
        Tracon(identifier="ZSSS RAW", ctr_lat=31.198, ctr_lon=121.336, tracon_range=30, elevation=0, apt_id=["ZSSS", "ZSPD"]),
        Tracon(identifier="RCTP RAW", ctr_lat=25.078, ctr_lon=121.233, tracon_range=30, elevation=100, apt_id=["RCTP", "RCSS"]),
        Tracon(identifier="RJTT RAW", ctr_lat=35.552, ctr_lon=139.78, tracon_range=30, elevation=0, apt_id=["RJTT"]),
        Tracon(identifier="YSSY RAW", ctr_lat=-33.946, ctr_lon=151.177, tracon_range=20, elevation=0, apt_id=["YSSY", "YSBK"]),
        Tracon(identifier="SBGL RAW", ctr_lat=-22.809, ctr_lon=-43.244, tracon_range=30, elevation=0, apt_id=["SBGL"]),
        Tracon(identifier="SAEZ RAW1", ctr_lat=-34.822, ctr_lon=-58.536, tracon_range=30, elevation=100, apt_id=["SAEZ", "SABE", "SADM"]),
        Tracon(identifier="SAEZ RAW2", ctr_lat=-34.67, ctr_lon=-58.5, tracon_range=20, elevation=100, apt_id=["SAEZ", "SABE", "SADM"]),
        Tracon(identifier="SAEZ RAW3", ctr_lat=-34.67, ctr_lon=-58.5, tracon_range=30, elevation=100, apt_id=["SAEZ", "SABE", "SADM"]),
        Tracon(identifier="SAEZ RAW4", ctr_lat=-34.67, ctr_lon=-58.5, tracon_range=40, elevation=100, apt_id=["SAEZ", "SABE", "SADM"]),
        Tracon(identifier="KDFW RAW1", ctr_lat=32.897, ctr_lon=-97.038, tracon_range=30, elevation=600, apt_id=["KDFW", "KDAL", "KFTW", "KAFW"]),
        Tracon(identifier="KDFW RAW2", ctr_lat=32.897, ctr_lon=-97.038, tracon_range=40, elevation=600, apt_id=["KDFW", "KDAL", "KFTW", "KAFW"]),
        Tracon(identifier="KDFW RAW3", ctr_lat=32.8, ctr_lon=-97.15, tracon_range=40, elevation=600, apt_id=["KDFW", "KDAL", "KFTW", "KAFW"])
    ]
    
    return tracons

def preupdate():
    """Called before each update cycle."""
    if tracon_manager and tracon_manager.active:
        # Time-critical operations before the regular update
        tracon_manager.pre_update()

def update():
    """Regular update function, called by BlueSky simulation."""
    if tracon_manager and tracon_manager.active:
        tracon_manager.update()

def reset():
    """Reset plugin state when simulation is reset."""
    if tracon_manager:
        tracon_manager.reset()