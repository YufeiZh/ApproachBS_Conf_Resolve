"""
ApproachBS plugin for BlueSky

This plugin adds terminal area (TRACON) operations to BlueSky.
"""
import random
import numpy as np
from bluesky import core, stack, sim, traf
from bluesky.tools.aero import ft, nm, kts, fpm
from bluesky.tools.geo import qdrpos, latlondist, qdrdist
from bluesky import navdb

# Constants
DELETE_BELOW = 200  # Aircraft (except taking off) below this field elevation will be automatically deleted [ft].
DELETE_DISTANCE = 5 # Aircraft (except arrival acf not in airspace yet) whose distance from the airspace is greater than
                    # this number will be automatically deleted [NM].
FAP_ELEVATION = 2000    # Final approach point elevation [ft].
GLIDE_SLOPE = 3.0   # Glide slope [deg].

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

    Member attributes:
        _active: Whether the TRACON is active or not
        identifier: TRACON identifier/name
        ctr_lat: Center latitude (degrees)
        ctr_lon: Center longitude (degrees)
        top: Top altitude (ft)
        bottom: Bottom altitude (ft)
        tracon_range: Radius of TRACON area (nm)
        elevation: Ground elevation (ft)
        apt: Dictionary mapping airport IDs to an ndarray [lat, lon, ele]
        runway_thres: Dictionary mapping airport IDs to dictinaries
            runway_thres[apt_id]: Dictionary of runway IDs to an ndarray [lat, lon, bearing]
        dep_rwy_id: List of open departure runway IDs, e.g., ["KATL 26L", "KATL 26R"]
        arr_rwy_id: List of open arrival runway IDs, e.g., ["KATL 26L", "KATL 26R"]
        depart_wp: Dictionary mapping departure waypoint IDs to an ndarray [lat, lon, ele], 
            e.g., {"EAONE": ndarray([33.8744, -83.8000, 10000]), "WEONE": ndarray([33.52569, -85.1224, 10000])}
        fap: Dictionary mapping airport IDs to dictionaries
            fap[apt_id]: Dictionary of runway IDs to an ndarray [lat, lon, ele]
        restrict: Dictionary mapping restricted area IDs to an ndarray of length 20
    """
    
    def __init__(self, 
                 identifier="DEFAULT", 
                 ctr_lat=0, 
                 ctr_lon=0,
                 tracon_range=30, 
                 top=10000,
                 bottom=500,
                 apt_id=None,
                 dep_rwy_id=None,
                 arr_rwy_id=None,
                 depart_wp=None,
                 restrict=None,
                 activate=False):
        """
        Initialize the TRACON object.
        """
        self._active = False

        self.identifier = identifier
        
        self.ctr_lat = ctr_lat
        self.ctr_lon = ctr_lon
        self.top = top
        self.bottom = bottom
        self.range = tracon_range
        self.elevation = 0
        
        self.apt = dict()
        for apt in apt_id:
            self.apt[apt] = None
        self.runway_thres = None

        self.dep_rwy_id = dep_rwy_id
        self.arr_rwy_id = arr_rwy_id
        self.fap = None
        self.depart_wp = depart_wp

        self.restrict = restrict

        # Activate
        if activate:
            self.activate_tracon()
    

    def is_active(self):
        """ Check if the TRACON is active. """
        return self._active


    def activate_tracon(self):
        """ Activate the TRACON if it is qualified. """

        # Load airports, runways, FAPs information from the database
        if not self._obtain_airport_info():
            print(f"Warning: Failed to load information from database for at least one airport. TRACON will not be activated.")
            return False

        if not self.identifier:
            print("Warning: TRACON identifier is not set. TRACON will not be activated.")
            return False

        if self.ctr_lat < -180 or self.ctr_lat > 180 or self.ctr_lon < -90 or self.ctr_lon > 90:
            print("Warning: TRACON center position is not valid. TRACON will not be activated.")
            return False

        if self.range < 10 or self.range > 100:
            print("Warning: TRACON range is not valid ([10, 100] in NM). TRACON will not be activated.")
            return False

        if self.top < 3000 or self.top > 18000:
            print("Warning: TRACON top altitude is not valid ([3000, 18000] in ft). TRACON will not be activated.")
            return False

        if self.bottom < DELETE_BELOW or self.bottom > self.top - 1000:
            print(f"Warning: TRACON bottom altitude is not valid ([{DELETE_BELOW}, top altitude - 1000] in ft). TRACON will not be activated.")
            return False

        if not self.apt:
            print("Warning: There is no valid airport. TRACON will not be activated.")
            return False
        for apt in self.apt:
            if not self._is_valid_apt(apt):
                print(f"Warning: Airport {apt} is not valid. TRACON will not be activated.")
                return False
            
        if not self.open_rwy_id:
            print("Warning: TRACON does not have any open runways. TRACON will not be activated.")
            return False
        for open_rwy in self.open_rwy_id:
            apt, rwy = open_rwy.upper().split(" ")
            if apt not in self.apt or rwy not in self.runway_thres[apt]:
                print(f"Warning: {open_rwy} is invalid. TRACON will not be activated.")
                return False
            try:
                _, fap_dist_from_ctr = qdrdist(self.ctr_lat, self.ctr_lon, self.fap[apt][rwy][0], self.fap[apt][rwy][1])
            except KeyError:
                print(f"Error: Final Approach Point of runway {open_rwy} is not valid. TRACON will not be activated.")
                return False
            if fap_dist_from_ctr > self.range - 2:
                print(f"Warning: Final Approach Point of runway {open_rwy} is too close to the boundary (<2NM). TRACON will not be activated.")
                return False
            
        if self.depart_wp is None or not self.depart_wp.any():
            print("Warning: TRACON does not have any departure waypoints. TRACON will not be activated.")
            return False
        for depart_wp in self.depart_wp_id:
            if depart_wp not in self.depart_wp_lat or depart_wp not in self.depart_wp_lon:
                print(f"Warning: Departure waypoint ID {depart_wp} is not valid. TRACON will not be activated.")
                return False
            _, dist_from_ctr = qdrdist(self.ctr_lat, self.ctr_lon, self.depart_wp_lat[depart_wp], self.depart_wp_lon[depart_wp])
            if dist_from_ctr < self.range:
                print(f"Warning: Departure waypoint ID {depart_wp} is too close. TRACON will not be activated.")
                return False
            if dist_from_ctr > self.range + DELETE_DISTANCE:
                print(f"Warning: Departure waypoint ID {depart_wp} is too far. TRACON will not be activated.")
                return False
            
        self._active = True
        print(f"TRACON {self.identifier} is activated.")
        return True


    def _random_departure_wp(self):
        """
        Generate departure waypoints.
        """
        pass


    def _obtain_airport_info(self):
        """
        Update airport, runway, fap info from the database.
        
        Args:
            apt_id: Airport ID to obtain information for
        """
        # Get airport information:
        

        # Get runway ids:
        self.runway_id = dict()
        try:
            for apt in self.apt:
                self.runway_id[apt] = list(navdb.rwythresholds[apt].keys())
        except KeyError:
            print(f"Error: Airport ID {apt} is not in database.")
            return False
        
        # Get runway threshold lat, lon and bearing:
        self.runway_thres = dict()
        try:
            for apt in self.apt_id:
                self.runway_thres[apt] = navdb.rwythresholds[apt]
        except KeyError:
            print(f"Error: Airport ID {apt} is not in database.")
            return False
        
        # Get final approach points:
        self.fap = dict()
        try:
            for apt in self.apt_id:
                self.fap[apt] = dict()
                for rwy in self.runway_id[apt]:
                    rwy_thres_lat, rwy_thres_lon, rwy_bearing = self.runway_thres[apt][rwy]
                    fap_lat, fap_lon = self._calculate_fap(rwy_thres_lat, rwy_thres_lon, rwy_bearing)
                    self.fap[apt][rwy] = np.array([fap_lat, fap_lon, self.apt_ele[apt] + FAP_ELEVATION], dtype=float)
        except KeyError:
            print(f"Error: Unable to load FAPs.")
            return False

        return True


    def _calculate_fap(self, rwy_thres_lat, rwy_thres_lon, rwy_bearing):
        """
        Calculate the final approach point (FAP) based on runway threshold latitude, longitude, and bearing.
        """

        # Calculate bearing and distance from runway threshold to FAP
        bearing_thres_to_fap = rwy_bearing + 180.0
        dist_thres_to_fap = FAP_ELEVATION * ft / np.tan(np.radians(GLIDE_SLOPE)) / nm
        
        # Calculate FAP latitude and longitude
        fap_lat, fap_lon = qdrpos(rwy_thres_lat, rwy_thres_lon, bearing_thres_to_fap, dist_thres_to_fap)
        
        return fap_lat, fap_lon


    def _is_valid_apt(self, apt_id):
        """
        Check if the airport ID is within the TRACON range.
        
        Args:
            apt_id: Airport ID to check
        
        Returns:
            bool: True if valid, False otherwise
        """
        if apt_id not in self.apt_id:
            print(f"Warning: Airport {apt_id} is not in the TRACON list.")

        if apt_id not in self.apt:
            print(f"Warning: Airport {apt_id} information is not complete.")
            return False

        _, dist_from_ctr = qdrdist(self.ctr_lat, self.ctr_lon, self.apt[apt_id][0], self.apt[apt_id][1])
        if dist_from_ctr > self.range:
            print(f"Airport {apt_id} is out of range.")
            return False

        return True


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
        self.activate_tracon()


# List of airport codes for testing
airport_codes = [
    "KATL", "KLAX", "KORD", "EGLL", "RJTT", "LFPG", "OMDB", "ZSPD", "VHHH", "EDDF",
    "WSSS", "KJFK", "RKSI", "VTBS", "KDFW", "LTBA", "EHAM", "WMKK", "VIDP", "KLAS",
    "KSFO", "YSSY", "KSEA", "CYYZ", "SBGR", "LEMD", "UUEE", "FACT", "EKCH", "LIRF",
    "YMML", "VABB", "RJAA", "KIAH", "EDDM", "KMIA", "LSZH", "OMAA", "KPHX", "LEBL",
    "KEWR", "VTSF", "KPDX", "KDTW", "RCKH", "ZBAA", "RCTP", "KPHL", "KCLT", "SAEZ",
    "MMMX", "LTAC", "LGAV", "BIKF", "EGKK", "RJOO", "KDEN", "LIMC", "TNCM", "CYUL",
    "KMSY", "LOWW", "VECC", "EFHK", "KDCA", "SBGL", "ENGM", "KSTL", "LPPT", "KMCO",
    "WIII", "VTBD", "LFPO", "RJCC", "ZGGG", "ZSHC", "VOBL", "VOTV", "KTPA", "LTFE",
    "LIPZ", "EDDH", "EDLW", "EDDS", "LTAI", "LGTS", "LIRN", "LBSF", "LZIB", "EPWA",
    "LJLJ", "EIDW", "LPPR", "LEAL", "ZSAM", "GCFV", "GCLP", "LPFR", "LGRP", "LGIR",
    "LHBP", "LKPR", "EPKT", "LROP", "ZUUU", "HECA", "LOWI", "HAAB", "LFLL", "LTFJ",
    "PANC", "LFBO", "CYVR", "EGCC"
]

