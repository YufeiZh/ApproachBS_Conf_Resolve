"""
ApproachBS Plugin for BlueSky
============================

This plugin enhances BlueSky with TRACON (Terminal Radar Approach Control) features,
allowing for more realistic air traffic control simulations in terminal airspace.

The plugin integrates with transformer-based conflict prediction and 
implements the same logic as the standalone TRACON simulator for consistency.

Features:
- TRACON airspace definition (supports pre-defined real-world TRACONs)
- Restricted areas (circle and polygon geometries)
- Advanced aircraft tracking with custom attributes
- Automatic aircraft generation based on scenario templates
- Conflict detection (separation violations and restricted area invasions)
- Scenario generation with configurable density parameters
- Load/Save scenarios in JSON and HDF5 formats compatible with ML models

Usage:
1. Enable the plugin in BlueSky
2. Load a predefined TRACON: APPROACHBS LOAD <TRACON_ID>
3. Generate a scenario: APPROACHBS SCENARIO <RA_DENSITY><AC_DENSITY>[LU]
4. Activate the TRACON mode: APPROACHBS ON
5. Generate more traffic if needed: APPROACHBS GEN ARRIVAL/DEPARTURE <count>
6. Check aircraft info: APPROACHBS INFO <acid/ALL>
7. Check conflicts: APPROACHBS CONFLICTS
8. Load/Save scenarios: APPROACHBS LOAD/SAVE <filename>

Density indicators:
- S (Sparse): 0-5 restricted areas or 1-10 aircraft
- M (Medium): 6-12 restricted areas or 11-30 aircraft
- D (Dense): 13-20 restricted areas or 31-50 aircraft
- F (Full): 20 restricted areas or 50 aircraft
- R (Random): Completely random density

Optional flags:
- L: Include possible Loss of Separation (conflict) aircraft
- U: Enable generation of aircraft in uncontrolled areas
"""

from .core import *