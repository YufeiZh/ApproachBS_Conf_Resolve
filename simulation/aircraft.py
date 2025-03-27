"""
ApproachBS Aircraft Class
---------------------
This module contains the Aircraft class that defines aircraft within the
ApproachBS plugin, closely matching your original aircraft.py.
"""

class Aircraft:
    """
    Class representing an aircraft within a TRACON scenario.
    Matches the implementation from the original aircraft.py.
    """
    
    def __init__(self, 
                 callsign,
                 lat,
                 lon,
                 curr_alt,
                 curr_cas,
                 curr_trk,
                 curr_vs=0,
                 acf_type="B738",
                 acf_class="medium",
                 target_alt=None,
                 target_vs=None,
                 target_cas=None,
                 target_trk=None,
                 intention="departure",
                 target_wp_lat=None,
                 target_wp_lon=None,
                 target_wp_alt=None,
                 under_ctrl=False,
                 time_entered=None,
                 time_stayed=0,
                 time_last_command=None,
                 time_since_last_command=None,
                 lnav_on=False,
                 vnav_on=False,
                 vnav_spd_on=False,
                 dist_to_wpt=None,
                 trk_to_wpt=None,
                 status="Active"):
        """
        Initialize an aircraft.
        
        Args:
            callsign: Aircraft identifier
            lat: Current latitude (degrees)
            lon: Current longitude (degrees)
            curr_alt: Current altitude (feet)
            curr_cas: Current calibrated airspeed (knots)
            curr_trk: Current track/heading (degrees)
            curr_vs: Current vertical speed (feet per minute)
            acf_type: Aircraft type (e.g., B738, A320)
            acf_class: Aircraft weight class (light, medium, heavy, A380)
            target_alt: Target/instructed altitude (feet)
            target_vs: Target/instructed vertical speed (feet per minute)
            target_cas: Target/instructed airspeed (knots)
            target_trk: Target/instructed track/heading (degrees)
            intention: Aircraft intention (arrival or departure)
            target_wp_lat: Target waypoint latitude (degrees)
            target_wp_lon: Target waypoint longitude (degrees)
            target_wp_alt: Target waypoint altitude (feet)
            under_ctrl: Whether aircraft is under TRACON control
            time_entered: Time when aircraft entered TRACON airspace
            time_stayed: Duration in TRACON airspace
            time_last_command: Time of last ATC command
            time_since_last_command: Time elapsed since last command
            lnav_on: Whether lateral navigation mode is active
            vnav_on: Whether vertical navigation mode is active
            vnav_spd_on: Whether VNAV speed mode is active
            dist_to_wpt: Distance to target waypoint (nm)
            trk_to_wpt: Track to target waypoint (degrees)
            status: Current status of the aircraft
        """
        # Identifier
        self.callsign = callsign
        
        # Aircraft type and class
        self.acf_type = acf_type
        self.acf_class = acf_class
        
        # Intention and routing
        self.intention = intention
        self.target_wp_lat = target_wp_lat
        self.target_wp_lon = target_wp_lon
        self.target_wp_alt = target_wp_alt
        
        # Control status
        self.under_ctrl = under_ctrl
        self.time_entered = time_entered
        self.time_stayed = time_stayed
        self.time_last_command = time_last_command
        self.time_since_last_command = time_since_last_command
        
        # Navigation modes
        self.lnav_on = lnav_on
        self.vnav_on = vnav_on
        self.vnav_spd_on = vnav_spd_on
        
        # Current position and state
        self.lat = lat
        self.lon = lon
        self.curr_alt = curr_alt
        self.curr_cas = curr_cas
        self.curr_trk = curr_trk
        self.curr_vs = curr_vs
        
        # Instructed/target values
        self.target_alt = target_alt if target_alt is not None else curr_alt
        self.target_vs = target_vs if target_vs is not None else curr_vs
        self.target_cas = target_cas if target_cas is not None else curr_cas
        self.target_trk = target_trk if target_trk is not None else curr_trk
        
        # Waypoint tracking
        self.dist_to_wpt = dist_to_wpt
        self.trk_to_wpt = trk_to_wpt
        
        # Status
        self.status = status
    
    def update_dist_to_wpt(self):
        """Update distance to target waypoint."""
        if self.target_wp_lat is not None and self.target_wp_lon is not None:
            from .utils import geocalc
            self.dist_to_wpt = geocalc.distance_between(
                self.lat, self.lon, self.target_wp_lat, self.target_wp_lon
            )
    
    def update_trk_to_wpt(self):
        """Update track to target waypoint."""
        if self.target_wp_lat is not None and self.target_wp_lon is not None:
            from .utils import geocalc
            self.trk_to_wpt = geocalc.bearing_from_to(
                self.lat, self.lon, self.target_wp_lat, self.target_wp_lon
            )
    
    def update_time_stayed(self, curr_time):
        """Update time spent in TRACON airspace."""
        if self.time_entered is not None:
            self.time_stayed = curr_time - self.time_entered
    
    def update_time_since_last_command(self, curr_time):
        """Update time since last command."""
        if self.time_last_command is not None:
            self.time_since_last_command = curr_time - self.time_last_command
    
    def to_dict(self):
        """Convert to dictionary for serialization."""
        return self.__dict__.copy()
    
    @classmethod
    def from_dict(cls, data_dict):
        """Create instance from dictionary."""
        # Create a new instance using minimal required parameters
        instance = cls(
            callsign=data_dict.get('callsign', 'UNKNOWN'),
            lat=data_dict.get('lat', 0),
            lon=data_dict.get('lon', 0),
            curr_alt=data_dict.get('curr_alt', 5000),
            curr_cas=data_dict.get('curr_cas', 200),
            curr_trk=data_dict.get('curr_trk', 0)
        )
        
        # Update with all dictionary values
        for key, value in data_dict.items():
            setattr(instance, key, value)
            
        return instance