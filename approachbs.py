"""
ApproachBS plugin for BlueSky

This plugin adds terminal area (TRACON) operations to BlueSky.
"""
import random
import numpy as np
from bluesky import core, traf, stack, sim
from bluesky.tools.aero import ft, nm, kts, fpm
from bluesky.tools.geo import qdrpos, latlondist

# Constants
DELETE_BELOW = 200  # Aircraft (except taking off) below this field elevation will be automatically deleted [ft].
DELETE_DISTANCE = 5 # Aircraft (except arrival acf not in airspace yet) whose distance from airspace is greater than
                    # this number will be automatically deleted [NM].

approach_bs = None

# Plugin initialization function
def init_plugin():
    """Initialize plugin"""
    # Create the plugin instance
    global approach_bs
    approach_bs = ApproachBS()
    
    # Configuration parameters
    config = {
        'plugin_name': 'APPROACHBS',
        'plugin_type': 'sim'
    }
    
    return config


# Plugin class
class ApproachBS(core.Entity):
    ''' Entity object for BlueSky. '''
    def __init__(self):
        super().__init__()

        # Tracon
        self.tracon = Tracon()

        # Scenario info
        self.n_abnormal_acf = 0

        with self.settrafarrays():
            # Additional aircraft attributes
            self.target_alt = np.array([])  # ATC assigned altitude [m]
            self.target_vs = np.array([])   # ATC assigned vertical speed [m/s]
            self.target_cas = np.array([])  # ATC assigned cas [m/s]
            self.target_trk = np.array([])  # ATC assigned track [deg]

            self.under_ctrl = np.array([])  # 0 for controlled, 1 for uncontrolled
            self.radar_vector = np.array([])  # 0 if acf is in own-navigation, 1 if acf is being radar vectored

            # Additional status
            self.take_over = np.array([], dtype=bool)   # Radar take-over or not. Not take-over aircrafts will not be deleted even if deletion condition is met
                                                        # All aircrafts are not take-over when spawning, until they enter the TRACON airspace.
            self.out_of_range = np.array([], dtype=bool)    # out-of-range acfs will be deleted if it is a radar take-over aircraft

            # Constants
            self.intention = np.array([])   # 0 for arrival, 1 for departure [deg]
            self.wp_lat = np.array([])      # Latitude of the target waypoint [deg]
            self.wp_lon = np.array([])      # Longitude of the target waypoint [deg]
            self.wp_alt = np.array([])      # Designated altitude of the target waypoint [m]
            self.rwy_course = np.array([])  # True heading of assigned runway (for arrival acfs) [deg]
            self.time_entered = np.array([])    # when aircraft entered the controlled airspace [s]
            self.time_last_cmd = np.array([])   # when aircraft recieved the previous command [s]


    def save_to_dict(self):
        pass


    def reset_from_dict(self, dict_data):
        self.tracon.reload_from_dict(dict_data('tracon'))
        pass


    def my_cre_acf(self, acfinfo: dict) -> None:
        """
        Create an aircraft
        Parameter acfinfo can be found in aircraft.py
        """

        if len(traf.id) >= 50:
            print("Failed to create aircraft. The number of aircrafts has reached the upper limit 50.")
            return
        
        if not self.tracon.active:
            print("Failed to create aircraft. Load a valid TRACON scenario first.")
        
        # Create new aircraft using traf.cre
        traf.cre(acid=acfinfo['callsign'],
                 actype=acfinfo['acf_type'],
                 aclat=acfinfo['lat'],
                 aclon=acfinfo['lon'],
                 achdg=acfinfo['curr_trk'],
                 acalt=acfinfo['curr_alt']*ft,
                 acspd=acfinfo['curr_cas']*kts)
        
        # Change altitude if target_alt neq curr_alt
        if acfinfo['target_alt'] != acfinfo['curr_alt']:
            stack.stack(f"ALT {acfinfo['callsign']} {acfinfo['target_alt']}")
            traf.vs[-1] = acfinfo['curr_vs'] * fpm
            self.target_alt[-1] = acfinfo['target_alt'] * ft

        # Change airspeed if target_cas neq curr_cas
        if acfinfo['target_cas'] != acfinfo['curr_cas']:
            stack.stack(f"SPD {acfinfo['callsign']} {acfinfo['target_cas']}")
            self.target_cas[-1] = acfinfo['target_cas'] * kts
        
        # Change track if target_trk neq curr_trk
        if acfinfo['target_trk'] != acfinfo['curr_trk']:
            stack.stack(f"HDG {acfinfo['callsign']} {acfinfo['target_trk']}")
            self.target_trk[-1] = acfinfo['target_trk']
        
        # Set aircraft to inactive
        self.take_over[-1] = False

        self.intention[-1] = 0 if acfinfo['intention'] == "arrival" else 1
        self.wp_lat[-1] = acfinfo['target_wp_lat']
        self.wp_lon[-1] = acfinfo['target_wp_lon']
        self.wp_alt[-1] = acfinfo['target_wp_alt']
        self.rwy_course[-1] = acfinfo['target_rwy_crs']

        pass


    def create(self, n=1):
        ''' This function gets called automatically when new aircraft are created. '''
        super().create(n)
        # Initialize attributes
        self.target_alt[-n:] = traf.alt[-n:]
        self.target_vs[-n:] = traf.alt[-n:]
        self.target_cas[-n:] = traf.cas[-n:]
        self.target_trk[-n:] = traf.trk[-n:]

        self.under_ctrl[-n:] = 0
        self.radar_vector[-n:] = 1.0

        self.take_over[-n:] = True
        self.out_of_range[-n:] = False
        
        self.intention[-n:] = 0
        self.wp_lat[-n:] = -1000.0
        self.wp_lon[-n:] = -1000.0
        self.wp_alt[-n:] = -1000.0
        self.rwy_course[-n:] = 0.0
        self.time_entered[-n:] = -60.0
        self.time_last_cmd[-n:] = -60.0


    @core.timed_function(name='delete_excess_acfs', dt=1)
    def delete_excess_acfs(self):
        ''' Periodically delete aircrafts if there re more than 50 aircrafts. '''
        while len(traf.id) > 50:
            traf.delete(-1)

    
    @core.timed_function(name='auto_delete_acfs', dt=0.5)
    def auto_delete_acfs(self):
        ''' Periodically delete low aircrafts and out-of-range aircrafts. '''
        pass


    @core.timed_function(name='update_additional_info', dt=0.5)
    def update_additional_info(self):
        ''' Periodically update additional attributes defined in this plugin for all aircrafts. '''
        pass


    # Stack commands
    @stack.command
    def assign_runway(self, acid: 'acid', runway: str = ""):
        ''' Assign the runway to aircraft or return the assigned waypoint. '''
        if not runway:
            msg = f'Aircraft {traf.id[acid]} is not assigned with a waypoint. ' if self.wp_alt[acid] == -1000.0 \
                    else f'The assigned waypoint of Aircraft {traf.id[acid]} is ({self.wp_lat[acid]}, {self.wp_lon[acid]}).'
            return True, msg
        return True, f'The assigned waypoint of Aircraft {traf.id[acid]} is set to ({self.wp_lat[acid]}, {self.wp_lon[acid]}).'
    

class Tracon:
    """
    The class representing TRACON airspace.
    """
    
    def __init__(self, 
                 active=False,
                 identifier="DEFAULT", 
                 ctr_lat=0, 
                 ctr_lon=0,
                 tracon_range=30, 
                 top=10000,
                 bottom=500,
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
                 open_rwy_id=None,
                 depart_wp_id=None,
                 depart_wp_lat=None,
                 depart_wp_lon=None,
                 fap_lat=None, 
                 fap_lon=None,
                 fap_alt=None,
                 restrict=None):
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
        # TRACON status
        self.active = active

        # TRACON Name
        self.identifier = identifier
        
        # Center location, elevation, airspace range, top altitude, bottom altitude
        self.ctr_lat = ctr_lat
        self.ctr_lon = ctr_lon
        self.range = tracon_range
        self.elevation = elevation
        self.top = top
        self.bottom = bottom
        
        # Airport information
        self.apt_id = apt_id if apt_id else []
        self.apt_lat = apt_lat if apt_lat else []
        self.apt_lon = apt_lon if apt_lon else []
        self.apt_ele = apt_ele if apt_ele else []
        
        # Runway information
        self.runway_id = runway_id if runway_id else []
        self.runway_thres_lat = runway_thres_lat if runway_thres_lat else {}
        self.runway_thres_lon = runway_thres_lon if runway_thres_lon else {}
        self.runway_bearing = runway_bearing if runway_bearing else {}
        self.runway_length = runway_length if runway_length else {}
        self.open_rwy_id = open_rwy_id if open_rwy_id else []

        # Departure waypoints
        self.depart_wp_id = depart_wp_id if depart_wp_id else []
        self.depart_wp_lat = depart_wp_lat if depart_wp_lat else {}
        self.depart_wp_lon = depart_wp_lon if depart_wp_lon else {}
        
        # Final approach points
        self.fap_lat = fap_lat if fap_lat else {}
        self.fap_lon = fap_lon if fap_lon else {}
        self.fap_alt = fap_alt if fap_alt else {}
        
        # Restricted areas
        self.restrict = restrict if restrict else np.zeros(shape=(20,20))
    
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
        """
        # Create a new instance
        instance = cls()
        
        # Update with dictionary data
        for key, value in dict_data.items():
            setattr(instance, key, value)

        instance.active = True
        
        return instance
    
    def reload_from_dict(self, dict_data):
        """
        Reload TRACON from a dictionary.
        """
        for key, value in dict_data.items():
            setattr(self, key, value)
        self.active = True

    


