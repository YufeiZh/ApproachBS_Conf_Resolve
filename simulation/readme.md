# ApproachBS Plugin Installation Guide

This guide explains how to install and use the ApproachBS plugin with BlueSky simulator.

## Installation

### Method 1: Direct Installation in BlueSky plugins directory

1. Find your BlueSky installation directory:
```python
import bluesky
print(bluesky.__path__)
```

2. Inside the BlueSky directory, navigate to the `plugins` folder.

3. Create a new folder called `approachbs` inside the plugins directory.

4. Copy all the plugin files to this new directory:
   - `__init__.py`
   - `core.py`
   - `tracon_manager.py`
   - `tracon.py`
   - `aircraft.py`
   - `restricted_area.py`
   - `commands.py`
   - `utils.py`
   - `geocalc.py`
   - `scenario_generator.py`
   - `scenario_loader.py`

### Method 2: For pip-installed BlueSky

If you installed BlueSky using pip (`pip install bluesky-simulator[headless]`), follow these steps:

```python
import os
import shutil
from pathlib import Path
import bluesky

# Find BlueSky plugins directory
plugins_dir = Path(bluesky.__path__[0]) / 'plugins'
print(f"Plugins directory: {plugins_dir}")

# Create approachbs plugin directory
approachbs_dir = plugins_dir / 'approachbs'
approachbs_dir.mkdir(exist_ok=True)

# Copy plugin files to the directory
# Assuming all plugin files are in current directory
files = [
    '__init__.py', 'core.py', 'tracon_manager.py', 
    'tracon.py', 'aircraft.py', 'restricted_area.py',
    'commands.py', 'utils.py', 'geocalc.py',
    'scenario_generator.py', 'scenario_loader.py'
]

for file in files:
    src = Path(file)
    dst = approachbs_dir / file
    shutil.copy(src, dst)

print("ApproachBS plugin installed successfully!")
```

## Verifying Installation

1. Start BlueSky
2. Check if the plugin is loaded:
```
PLUGINS
```

You should see `APPROACHBS` in the list of loaded plugins.

## Basic Usage

### Setting up a TRACON environment

1. List available TRACON templates:
```
APPROACHTRACON
```

2. Select a TRACON:
```
APPROACHTRACON KSFO RAW1
```

3. Activate ApproachBS mode:
```
APPROACHBS ON
```

### Generating Scenarios

1. Generate a complete scenario with medium density:
```
APPROACHSCENARIO MM
```

The first character specifies the density of restricted areas:
- S: Sparse (0-5 areas)
- M: Medium (6-12 areas)
- D: Dense (13-20 areas)
- F: Full (20 areas)
- R: Random (0-20 areas)

The second character specifies the density of aircraft:
- S: Sparse (1-10 aircraft)
- M: Medium (11-30 aircraft)
- D: Dense (31-50 aircraft)
- F: Full (50 aircraft)
- R: Random (0-50 aircraft)

Additional flags:
- L: Include aircraft with potential Loss of Separation
- U: Allow generation in uncontrolled areas

### Adding Individual Aircraft

1. Add arrivals:
```
APPROACHGEN ARRIVAL 5
```

2. Add departures:
```
APPROACHGEN DEPARTURE 3
```

3. Add aircraft with potential conflict:
```
APPROACHLOS KLM123
```

### Checking Aircraft Information and Conflicts

1. Get information about a specific aircraft:
```
APPROACHINFO KLM123
```

2. Get information about all aircraft:
```
APPROACHINFO ALL
```

3. Check for conflicts:
```
APPROACHCONFLICTS
```

### Saving and Loading Scenarios

1. Save the current scenario:
```
APPROACHSAVE output/scenario1.json
```

2. Load a scenario:
```
APPROACHLOAD output/scenario1.json
```

Both JSON and HDF5 formats are supported:
```
APPROACHSAVE output/scenario2.h5
APPROACHLOAD output/scenario2.h5
```

### Getting Token Representation for ML Models

1. Display token summary:
```
APPROACHTOKEN
```

2. Save tokens to a file:
```
APPROACHTOKEN SAVE output/tokens.h5
```

## Troubleshooting

### Plugin Not Loading

- Check that all files are in the correct directory
- Ensure all required modules are present
- Check for syntax errors in the plugin code

### Aircraft Generation Issues

- Verify that the TRACON area is defined
- Check if you've reached the maximum aircraft limit
- Ensure there's a valid runway or fix for the aircraft to target

### Conflict Detection Problems

- Make sure the APPROACHBS mode is activated (`APPROACHBS ON`)
- Verify that the airspace parameters are properly set
- Check if aircraft are within the defined TRACON area

## Integration with Reinforcement Learning

The plugin provides ML-ready state representation through the token system:

1. Get token representation with `APPROACHTOKEN SAVE filename.h5` command
2. Tokens are in the format expected by your transformer encoder
3. Access token matrices directly through the `tracon_manager.get_scenario_tokens()` method
4. The token representation matches your preprocessing.py implementation

For CTDE (Centralized Training, Decentralized Execution) SAC training, the plugin maintains the same state representation as your standalone transformer project, ensuring compatibility with your existing ML architecture.