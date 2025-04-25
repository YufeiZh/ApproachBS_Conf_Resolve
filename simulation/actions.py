"""
Actions module for Autonomous TRACON

This module defines the action space and handles conversion between
model outputs and BlueSky simulator commands.
"""
import bluesky as bs


class ActionSpace:
    def __init__(self):
        """Initialize the action space for the Autonomous TRACON"""
        # Access the approachbs plugin
        self.approach_bs = bs.plugins.approachbs.approach_bs
        
        # Define action bounds
        self.min_heading = 0
        self.max_heading = 359
        self.min_altitude = None  # Dynamic based on TRACON bottom
        self.max_altitude = None  # Dynamic based on TRACON top
        self.min_speed = 160     # knots
        self.max_speed = 280     # knots
        
        # Action components (to be set at runtime)
        self.send_command = None  # binary
        self.direct_to = None     # binary
        self.change_heading = None  # binary
        self.change_altitude = None # binary
        self.change_speed = None    # binary
        self.heading = None       # continuous [0, 359]
        self.altitude = None      # continuous [min_alt, max_alt]
        self.speed = None         # continuous [160, 280]

    def update_altitude_bounds(self):
        """Update the altitude bounds based on current TRACON settings"""
        if self.approach_bs and self.approach_bs.tracon:
            self.min_altitude = self.approach_bs.tracon.bottom
            self.max_altitude = self.approach_bs.tracon.top
        else:
            # Default values if TRACON is not activated
            self.min_altitude = 1000
            self.max_altitude = 11000

    def validate_and_process_action(self, action_vector):
        """
        Validate and process an action vector for a specific aircraft
        
        Args:
            action_vector: Vector of continuous action values from the model
            acid: Aircraft ID (string)
        
        Returns:
            dict: Dictionary containing the processed action components
            bool: Flag indicating if the action is valid
        """
        if len(action_vector) < 8:
            return None, False
        
        # Update altitude bounds
        self.update_altitude_bounds()
        
        # Process each component of the action vector
        self.send_command = bool(action_vector[0] > 0.5)
        self.direct_to = bool(action_vector[1] > 0.5)
        self.change_heading = bool(action_vector[2] > 0.5)
        self.change_altitude = bool(action_vector[3] > 0.5)
        self.change_speed = bool(action_vector[4] > 0.5)
        
        # Round heading to the nearest integer and ensure it's in [0, 359]
        self.heading = int(round(self.map_to_range(action_vector[5], 0, 1, 
                                                   self.min_heading, self.max_heading))) % 360
        
        # Round altitude to the nearest 100 feet and ensure it's in [min_alt, max_alt]
        raw_altitude = self.map_to_range(action_vector[6], 0, 1, 
                                         self.min_altitude, self.max_altitude)
        self.altitude = int(round(raw_altitude / 100) * 100)
        
        # Round speed to the nearest integer and ensure it's in [min_speed, max_speed]
        self.speed = int(round(self.map_to_range(action_vector[7], 0, 1, 
                                                 self.min_speed, self.max_speed)))
        
        # Return the processed action components as a dictionary
        action_dict = {
            'send_command': self.send_command,
            'direct_to': self.direct_to,
            'change_heading': self.change_heading,
            'change_altitude': self.change_altitude,
            'change_speed': self.change_speed,
            'heading': self.heading,
            'altitude': self.altitude,
            'speed': self.speed
        }
        
        return action_dict, True

    def map_to_range(self, value, in_min, in_max, out_min, out_max):
        """Map a value from one range to another"""
        return (value - in_min) * (out_max - out_min) / (in_max - in_min) + out_min

    def execute_action(self, action_dict, acid):
        """
        Execute the given action for the specified aircraft
        
        Args:
            action_dict: Dictionary containing the processed action components
            acid: Aircraft ID (string)
        
        Returns:
            command: The executed command (string) or None if no command was sent
        """
        idx = bs.traf.id2idx(acid)
        if idx < 0:
            return None  # Aircraft doesn't exist
        
        # If no command should be sent, return immediately
        if not action_dict['send_command']:
            return None
        
        commands = []
        
        # Handle direct-to command based on aircraft intention
        if action_dict['direct_to']:
            if self.approach_bs.intention[idx] == 0:  # Arrival aircraft
                apt, rwy = self.approach_bs.wp_id[idx].split('/')
                commands.append(f"DEST {acid}, {apt}, RW{rwy}")
            elif self.approach_bs.intention[idx] == 1:  # Departure aircraft
                waypoint = self.approach_bs.wp_id[idx]
                commands.append(f"ADDWPT {acid}, {waypoint}")
        else:
            # Handle heading, altitude, and speed changes
            if action_dict['change_heading']:
                commands.append(f"HDG {acid}, {action_dict['heading']}")
            if action_dict['change_altitude']:
                commands.append(f"ALT {acid}, {action_dict['altitude']}")
            if action_dict['change_speed']:
                commands.append(f"SPD {acid}, {action_dict['speed']}")
        
        # Join the commands with semicolons
        command = "; ".join(commands)
        
        # Execute the command in BlueSky
        bs.stack.stack(command)
        
        return command


def process_model_output(model_output, acid_list):
    """
    Process the raw model output into a dictionary of actions for each aircraft
    
    Args:
        model_output: Raw output from the model, a numpy array of shape (num_aircraft, action_dim)
        acid_list: List of aircraft IDs (strings)
        
    Returns:
        dict: Dictionary mapping aircraft IDs to action dictionaries
    """
    action_space = ActionSpace()
    actions = {}
    
    for i, acid in enumerate(acid_list):
        if i < len(model_output):
            action_dict, valid = action_space.validate_and_process_action(model_output[i])
            if valid:
                actions[acid] = action_dict
    
    return actions


def execute_actions(actions):
    """
    Execute a dictionary of actions for multiple aircraft
    
    Args:
        actions: Dictionary mapping aircraft IDs to action dictionaries
        
    Returns:
        dict: Dictionary mapping aircraft IDs to executed commands
    """
    action_space = ActionSpace()
    executed_commands = {}
    
    for acid, action_dict in actions.items():
        command = action_space.execute_action(action_dict, acid)
        if command:
            executed_commands[acid] = command
    
    return executed_commands