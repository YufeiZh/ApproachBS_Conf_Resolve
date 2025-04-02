"""
ApproachBS plugin for BlueSky

This plugin adds terminal area (TRACON) operations to BlueSky.
"""
import random
import numpy as np
import bluesky as bs
from bluesky import core, stack, sim, traf
from bluesky.traffic.asas import ConflictDetection
from bluesky.tools.areafilter import basic_shapes, deleteArea, defineArea, checkInside
from bluesky.tools.aero import ft, nm, kts, fpm
from bluesky.tools.geo import qdrpos, qdrdist, qdrdist_matrix, kwikqdrdist_matrix
from bluesky.navdatabase import Navdatabase

# Constants
DELETE_BELOW = 200  # Aircraft (except taking off) below this field elevation will be automatically deleted [ft].
DELETE_DISTANCE = 5 # Aircraft (except arrival acf not in airspace yet) whose distance from the airspace is greater than
                    # this number will be automatically deleted [NM].
FAP_ELEVATION = 2000    # Final approach point elevation [ft].
GLIDE_SLOPE = 3.0   # Glide slope [deg].

navdb = Navdatabase()
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

        # Simulator settings
        stack.stack("ASAS ON")

        self.mode = 'debug'         # Simulation mode: default, debug, or test

        # Tracon
        self.tracon = Tracon()

        # Conflict detectors
        # Conflict in one minute
        self.conflict_one_minute = MyConflictDetection()
        self.conflict_one_minute.rpz_def = 5.0 * nm  # Horizontal separation [m]
        self.conflict_one_minute.hpz_def = 1000.0 * ft  # Vertical separation [m]
        self.conflict_one_minute.dtlookahead_def = 60.0  # Lookahead time [s]
        # Conflict in three minutes
        self.conflict_three_minute = MyConflictDetection()
        self.conflict_three_minute.rpz_def = 5.0 * nm  # Horizontal separation [m]
        self.conflict_three_minute.hpz_def = 1000.0 * ft  # Vertical separation [m]
        self.conflict_three_minute.dtlookahead_def = 180.0  # Lookahead time [s]

        # Scenario info
        self.n_abnormal_acf = 0

        with self.settrafarrays():

            self.under_ctrl = np.array([], dtype=bool)  # Whether acf is under control (in airspace)
            self.radar_vector = np.array([], dtype=bool)  # Whether acf is being radar vectored or on self-navigation
                                                            # This will be used to assess whether an acf leaves airspace unexpectly
            self.intention = np.array([])   # 0 for arrival, 1 for departure [deg]
            self.wp_lat = np.array([])      # Latitude of the target waypoint [deg]
            self.wp_lon = np.array([])      # Longitude of the target waypoint [deg]
            self.wp_alt = np.array([])      # Designated altitude of the target waypoint [ft]
            self.rwy_course = np.array([])  # True heading (magnatic, aka "bearing" in BlueSky) of assigned runway (for arrival acfs) [deg]
            self.dist_to_wp = np.array([])  # Distance to the target waypoint [nm]
            self.bear_to_wp = np.array([])  # Bearing to the target waypoint [deg]
            self.time_entered = np.array([])    # when aircraft entered the controlled airspace [s]
            self.time_last_cmd = np.array([])   # when aircraft recieved the previous command [s]

            # Additional status
            self.take_over = np.array([], dtype=bool)   # Radar take-over or not. Not take-over aircrafts will not be deleted even if deletion condition is met
                                                        # All aircrafts are not take-over when spawning, until they enter the TRACON airspace.
            self.out_of_range = np.array([], dtype=bool)    # out-of-range acfs will be deleted if it is a radar take-over aircraft

            # Copy attibutes to monitor changes
            self.prev_acf_id = traf.id.copy()   # Previous aircraft IDs
            self.prev_selspd = traf.selspd.copy()   # Previous selected speed [m/s]
            self.prev_selalt = traf.selalt.copy()   # Previous selected altitude [m]
            self.prev_seltrk = traf.aporasas.trk.copy() # Previous selected track [deg]
            self.prev_under_ctrl = self.under_ctrl.copy()   # Previous under control status
            self.prev_radar_vector = self.radar_vector.copy()   # Previous radar vector status

        # Current area invasion)

        # Aircraft generation settings
        self.auto_gen_setting = {
            "acf_auto_gen": False,  # Whether to automatically generate aircrafts or not
            "dep_gen_rate": 0.0,    # Aircraft generation rate [aircraft/min]
            "arr_gen_rate": 0.0     # Aircraft generation rate [aircraft/min]
        }
        self.last_dep_gen_time = 0.0    # Last time aircraft was generated [s]
        self.last_arr_gen_time = 0.0    # Last time aircraft was generated [s]


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


    @core.timed_function(name='my_asas_2', dt=0.1)
    def update(self):
        ''' This function gets called automatically every 0.1 second. '''

        # Delete excess aircrafts
        if self.appbs_delete_excess_acfs.__manualtimer__.readynext() and self.tracon.is_active():
            self.appbs_delete_excess_acfs()

        # Update aircraft status
        if self.appbs_update.__manualtimer__.readynext() and self.tracon.is_active():
            self.appbs_update()

        # Delete out-of-range aircrafts
        if self.appbs_auto_delete_acfs.__manualtimer__.readynext() and self.tracon.is_active():
            self.appbs_auto_delete_acfs()

        # Update conflict detectors
        if self.update_conflict_detectors.__manualtimer__.readynext() and self.tracon.is_active():
            self.update_conflict_detectors()


    @core.timed_function(name='my_asas', dt=0.5, manual=True)
    def update_conflict_detectors(self):
        ''' This function gets called automatically every 0.5 second if the airspace is active. '''
        # Check for conflicts in one minute
        self.conflict_one_minute.update(traf, traf)


    @core.timed_function(name='appbs_update', dt=0.1, manual=True)
    def appbs_update(self):
        ''' Periodically update aircraft status. '''

        if not self.tracon.active:
            return
        
        # Check if the aircraft is out of range
        dist_from_ctr = qdrdist_matrix(self.tracon.ctr_lat, self.tracon.ctr_lon, traf.lat, traf.lon)[1]
        self.out_of_range[:] = (dist_from_ctr > self.tracon.range + DELETE_DISTANCE) | (traf.alt < self.tracon.elevation + DELETE_BELOW)

        # under_ctrl
        self.under_ctrl[:] = (dist_from_ctr < self.tracon.range) & (self.tracon.bottom <= traf.alt <= self.tracon.top)

        # Take over aircrafts in the TRACON airspace
        self.take_over[:] = self.take_over | self.under_ctrl

        # Radar vector
        # Do not vector if the aircraft has LNAV on
        self.radar_vector[traf.swlnav] = False
        # vector aircrafts that just entered the TRACON airspace
        self.radar_vector[self.under_ctrl & ~self.prev_under_ctrl] = True
        # vector in-airspace aircrafts that has LNAV off
        self.radar_vector[self.under_ctrl & ~traf.swlnav] = True

        # Record enter time
        self.time_entered = np.where(self.under_ctrl & ~self.prev_under_ctrl, sim.simt, self.time_entered)

        # Check if the aircraft received a command
        received_cmd = np.zeros_like(traf.id, dtype=bool)
        for i in range(len(self.prev_acf_id)):
            if self.prev_acf_id[i] not in traf.id:
                # Aircraft is deleted
                continue
            new_index = traf.id.index(self.prev_acf_id[i])
            if self.prev_selspd[i] != traf.selspd[new_index] or \
                self.prev_selalt[i] != traf.selalt[new_index] or \
                self.prev_seltrk[i] != traf.aporasas.trk[new_index] or \
                self.prev_radar_vector[i] != self.radar_vector[new_index]:
                # Aircraft received a command
                received_cmd[new_index] = True
        self.time_last_cmd = np.where(received_cmd, sim.simt, self.time_last_cmd)

        # Update previous status
        self.prev_selspd[:] = traf.selspd
        self.prev_selalt[:] = traf.selalt
        self.prev_seltrk[:] = traf.aporasas.trk
        self.prev_under_ctrl[:] = self.under_ctrl
        self.prev_radar_vector[:] = self.radar_vector


    @core.timed_function(name='appbs_delete_excess_acfs', dt=0.5, manual=True)
    def appbs_delete_excess_acfs(self):
        ''' Periodically delete aircrafts if there re more than 50 aircrafts. '''
        while len(traf.id) > 50:
            traf.delete(-1)

    
    @core.timed_function(name='appbs_auto_delete_acfs', dt=0.5, manual=True)
    def appbs_auto_delete_acfs(self):
        ''' Periodically delete out-of-range aircrafts. '''
        if not self.tracon.active:
            return
        i = 0
        while i < len(traf.id):
            if self.take_over[i] and self.out_of_range[i]:
                if self.intention[i] == 0 and traf.alt[i] < DELETE_BELOW:
                    # Arrival aircraft landing
                    stack.stack(f"ECHO Arrival aircraft {traf.id[i]} is deleted.")
                elif self.intention[i] == 1 \
                    and qdrdist(self.tracon.ctr_lat, self.tracon.ctr_lon, traf.lat[i], traf.lon[i])[1] \
                        > self.tracon.range + DELETE_DISTANCE:
                    # Departure aircraft leaving the airspace
                    stack.stack(f"ECHO Departure aircraft {traf.id[i]} is deleted.")
                else:
                    # Abnormal deletion
                    stack.stack(f"ECHO Aircraft {traf.id[i]} is deleted due to out-of-range.")
                traf.delete(i)
                i -= 1
            elif self.mode == 'default':
                if qdrdist(self.tracon.ctr_lat, self.tracon.ctr_lon, traf.lat[i], traf.lon[i])[1] > self.tracon.range + 50:
                    # Force deleting aircrafts too far away from the TRACON
                    stack.stack(f"ECHO Aircraft {traf.id[i]} is deleted due to out-of-range.")
                    traf.delete(i)
                    i -= 1
            i += 1


    # Stack commands
    @stack.command
    def assign_runway(self, acid: 'acid', runway: str = ""):
        ''' Assign the runway to aircraft or return the assigned waypoint. '''
        if not runway:
            msg = f'Aircraft {traf.id[acid]} is not assigned with a waypoint. ' if self.wp_alt[acid] == -1000.0 \
                    else f'The assigned waypoint of Aircraft {traf.id[acid]} is ({self.wp_lat[acid]}, {self.wp_lon[acid]}).'
            return True, msg
        return True, f'The assigned waypoint of Aircraft {traf.id[acid]} is set to ({self.wp_lat[acid]}, {self.wp_lon[acid]}).'
    

class MyConflictDetection(ConflictDetection):
    """
    Custom conflict detection class for TRACON operations.
    """

    def __init__(self):
        super().__init__()
        self.use_global_asas_setting = False


    def reset(self):
        """ 
        Override the reset function to avoid resetting the global settings. 
        Does not reset horizontal and vertical minimum separations and lookahead time
        """
        core.Entity.reset(self)
        self.clearconfdb()
        self.confpairs_all.clear()
        self.lospairs_all.clear()


    @staticmethod
    def setmethod():
        """ Override the setmethod function to avoid stack commands handling. """
        pass


    def setrpz(self):
        """ Override the setrpz function to avoid stack commands handling. """
        pass


    def sethpz(self):
        """ Override the sethpz function to avoid stack commands handling. """
        pass


    def setdtlook(self):
        """ Override the setdtlook function to avoid stack commands handling. """
        pass


    def setdtnolook(self):
        """ Override the setdtnolook function to avoid stack commands handling. """
        pass


    def detect(self, ownship, intruder, rpz, hpz, dtlookahead):
        ''' State-based conflict detection. Copied from the original code. '''
        # Identity matrix of order ntraf: avoid ownship-ownship detected conflicts
        I = np.eye(ownship.ntraf)

        # Horizontal conflict ------------------------------------------------------

        # qdrlst is for [i,j] qdr from i to j, from perception of ADSB and own coordinates
        qdr, dist = kwikqdrdist_matrix(np.asmatrix(ownship.lat), np.asmatrix(ownship.lon),
                                    np.asmatrix(intruder.lat), np.asmatrix(intruder.lon))

        # Convert back to array to allow element-wise array multiplications later on
        # Convert to meters and add large value to own/own pairs
        qdr = np.asarray(qdr)
        dist = np.asarray(dist) * nm + 1e9 * I

        # Calculate horizontal closest point of approach (CPA)
        qdrrad = np.radians(qdr)
        dx = dist * np.sin(qdrrad)  # is pos j rel to i
        dy = dist * np.cos(qdrrad)  # is pos j rel to i

        # Ownship track angle and speed
        owntrkrad = np.radians(ownship.trk)
        ownu = ownship.gs * np.sin(owntrkrad).reshape((1, ownship.ntraf))  # m/s
        ownv = ownship.gs * np.cos(owntrkrad).reshape((1, ownship.ntraf))  # m/s

        # Intruder track angle and speed
        inttrkrad = np.radians(intruder.trk)
        intu = intruder.gs * np.sin(inttrkrad).reshape((1, ownship.ntraf))  # m/s
        intv = intruder.gs * np.cos(inttrkrad).reshape((1, ownship.ntraf))  # m/s

        du = ownu - intu.T  # Speed du[i,j] is perceived eastern speed of i to j
        dv = ownv - intv.T  # Speed dv[i,j] is perceived northern speed of i to j

        dv2 = du * du + dv * dv
        dv2 = np.where(np.abs(dv2) < 1e-6, 1e-6, dv2)  # limit lower absolute value
        vrel = np.sqrt(dv2)

        tcpa = -(du * dx + dv * dy) / dv2 + 1e9 * I

        # Calculate distance^2 at CPA (minimum distance^2)
        dcpa2 = np.abs(dist * dist - tcpa * tcpa * dv2)

        # Check for horizontal conflict
        # RPZ can differ per aircraft, get the largest value per aircraft pair
        rpz = np.asarray(np.maximum(np.asmatrix(rpz), np.asmatrix(rpz).transpose()))
        R2 = rpz * rpz
        swhorconf = dcpa2 < R2  # conflict or not

        # Calculate times of entering and leaving horizontal conflict
        dxinhor = np.sqrt(np.maximum(0., R2 - dcpa2))  # half the distance travelled inzide zone
        dtinhor = dxinhor / vrel

        tinhor = np.where(swhorconf, tcpa - dtinhor, 1e8)  # Set very large if no conf
        touthor = np.where(swhorconf, tcpa + dtinhor, -1e8)  # set very large if no conf

        # Vertical conflict --------------------------------------------------------

        # Vertical crossing of disk (-dh,+dh)
        dalt = ownship.alt.reshape((1, ownship.ntraf)) - \
            intruder.alt.reshape((1, ownship.ntraf)).T  + 1e9 * I

        dvs = ownship.vs.reshape(1, ownship.ntraf) - \
            intruder.vs.reshape(1, ownship.ntraf).T
        dvs = np.where(np.abs(dvs) < 1e-6, 1e-6, dvs)  # prevent division by zero

        # Check for passing through each others zone
        # hPZ can differ per aircraft, get the largest value per aircraft pair
        hpz = np.asarray(np.maximum(np.asmatrix(hpz), np.asmatrix(hpz).transpose()))
        tcrosshi = (dalt + hpz) / -dvs
        tcrosslo = (dalt - hpz) / -dvs
        tinver = np.minimum(tcrosshi, tcrosslo)
        toutver = np.maximum(tcrosshi, tcrosslo)

        # Combine vertical and horizontal conflict----------------------------------
        tinconf = np.maximum(tinver, tinhor)
        toutconf = np.minimum(toutver, touthor)

        swconfl = np.array(swhorconf * (tinconf <= toutconf) * (toutconf > 0.0) *
                           np.asarray(tinconf < np.asmatrix(dtlookahead).T) * (1.0 - I), dtype=bool)

        # --------------------------------------------------------------------------
        # Update conflict lists
        # --------------------------------------------------------------------------
        # Ownship conflict flag and max tCPA
        inconf = np.any(swconfl, 1)
        tcpamax = np.max(tcpa * swconfl, 1)

        # Select conflicting pairs: each a/c gets their own record
        confpairs = [(ownship.id[i], ownship.id[j]) for i, j in zip(*np.where(swconfl))]
        swlos = (dist < rpz) * (np.abs(dalt) < hpz)
        lospairs = [(ownship.id[i], ownship.id[j]) for i, j in zip(*np.where(swlos))]

        return confpairs, lospairs, inconf, tcpamax, \
            qdr[swconfl], dist[swconfl], np.sqrt(dcpa2[swconfl]), \
                tcpa[swconfl], tinconf[swconfl]


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
        rwy_thres: Dictionary mapping airport IDs to dictinaries
            rwy_thres[apt_id]: Dictionary of runway IDs to an ndarray [lat, lon, bearing]
        fap: Dictionary mapping airport IDs to dictionaries
            fap[apt_id]: Dictionary of runway IDs to an ndarray [lat, lon, ele]
        dep_rwy_id: Dictionary mapping airport IDs to list of open departure runway IDs, e.g., ["KSFO": ["28L", "28R"], "KSJC": ["30R"]]
        arr_rwy_id: Dictionary mapping airport IDs to list of open arrival runway IDs, e.g., ["KSFO": ["28L", "28R"], "KSJC": ["30R"]]
        depart_wp: Dictionary mapping departure waypoint IDs to an ndarray [lat, lon, ele], 
            e.g., {"EAONE": ndarray([33.8744, -83.8000, 10000]), "WEONE": ndarray([33.52569, -85.1224, 10000])}
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

        self._gui_draw()
        
        self.apt = dict()
        if apt_id is not None:
            for apt in apt_id:
                self.apt[apt] = None
        self.rwy_thres = None
        self.fap = None

        # Extract airport information from the database
        self._obtain_airport_info()

        # Add departure runways
        self.dep_rwy_id = dict()
        if dep_rwy_id is not None:
            for dep_rwy in dep_rwy_id:
                apt, rwy = dep_rwy.upper().split(' ')
                self.add_arr_rwy(apt, rwy)

        # Add arrival runways
        self.arr_rwy_id = dict()
        if arr_rwy_id is not None:
            for arr_rwy in arr_rwy_id:
                apt, rwy = arr_rwy.upper().split(' ')
                self.add_arr_rwy(apt, rwy)
        

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
            
        if not self.dep_rwy_id:
            print("Warning: TRACON does not have any open runways. TRACON will not be activated.")
            return False
        if not self.arr_rwy_id:
            print("Warning: TRACON does not have any open runways. TRACON will not be activated.")
            return False
        if not self.depart_wp:
            print("Warning: TRACON does not have any departure waypoints. TRACON will not be activated.")
            return False
            
        self._active = True
        print(f"TRACON {self.identifier} is activated.")
        return True


    def add_restricted(self, args: list):
        """
        Add restricted area to the TRACON.
        
        Args:
            args: a list of length no greater than 20
                args[0]: restricted area ID (string)
                args[1]: type of restricted area (0: circle, 1: polygon)
                args[2]: bottom altitude (ft)
                args[3]: top altitude (ft)
                If type is 0:
                    args[4]: center latitude (degrees)
                    args[5]: center longitude (degrees)
                    args[6]: radius (nm)
                If type is 1:
                    args[4:]: coordinates of vertices, [lat1, lat2, lon1, lon2, ...] (degrees), 
                              not exceeding 8 vertices
        """
        # Check if the TRACON is active
        if self.active:
            print("Error: Cannot add restricted area to an active TRACON.")
            return False

        # At most 20 restricted areas
        if len(self.restrict) >= 20:
            print("Error: The number of restricted areas exceeds the limit (20).")
            return False
        
        # Check if the area ID already exists
        if args[0] in self.restrict:
            print(f"Error: Restricted area ID {args[0]} already exists.")
            return False
        
        # Check the format
        valid = True
        if args[1] == 0:
            if len(args) != 7:
                valid = False
        elif args[1] == 1:
            if len(args) < 10 or len(args) > 20 or len(args) % 2 != 0:
                valid = False
        else:
            valid = False
        if not valid:
            print("Error: Invalid arguments for restricted area.")
            return False
        
        # Add the restricted area
        self.restrict[args[0]] = np.array(args[1:], dtype=float)

        # Draw the restricted area
        # Later

        return True


    def remove_restricted(self, area_id: str):
        """
        Remove a restricted area from the TRACON.
        
        Args:
            area_id: ID of the restricted area to remove
        """
        # Check if the TRACON is active
        if self.active:
            print("Error: Cannot remove restricted area from an active TRACON.")

        if area_id in self.restrict:
            del self.restrict[area_id]
            print(f"Restricted area {area_id} removed.")
            return True
        else:
            print(f"Error: Restricted area {area_id} does not exist.")
            return False


    def add_dep_rwy(self, apt_id: str, rwy_id: str):
        """
        Add a departure runway to the TRACON.
        """
        
        # Check if the airport ID and rwy id is valid
        if not self.apt or apt_id not in self.apt or rwy_id not in self.rwy_thres[apt_id]:
            return False
        
        # Add the departure runway
        if apt_id not in self.dep_rwy_id:
            self.dep_rwy_id[apt_id] = []
        if rwy_id not in self.dep_rwy_id[apt_id]:
            self.dep_rwy_id[apt_id].append(rwy_id)
        return True


    def remove_dep_rwy(self, apt_id: str, rwy_id: str):
        """
        Remove a departure runway from the TRACON.
        """

        # If the TRACON is active, there needs to be at least one runway open
        if self._active and len(self.dep_rwy_id[apt_id]) <= 1:
            print("Error: Cannot remove departure runway from an active TRACON. At least one runway must be open.")
            return False
        
        # Remove the departure runway
        if apt_id in self.dep_rwy_id and rwy_id in self.dep_rwy_id[apt_id]:
            self.dep_rwy_id[apt_id].remove(rwy_id)
            if len(self.dep_rwy_id[apt_id]) == 0:
                del self.dep_rwy_id[apt_id]
            return True
        else:
            print(f"Error: Departure runway {rwy_id} does not exist.")
            return False


    def add_arr_rwy(self, apt_id: str, rwy_id: str):
        """
        Add an arrival runway to the TRACON.
        """
        
        # Check if the airport ID and rwy id is valid
        if not self.apt or apt_id not in self.apt or rwy_id not in self.rwy_thres[apt_id]:
            return False
        
        # Add the arrival runway
        if apt_id not in self.arr_rwy_id:
            self.arr_rwy_id[apt_id] = []
        if rwy_id not in self.arr_rwy_id[apt_id]:
            self.arr_rwy_id[apt_id].append(rwy_id)
        return True


    def remove_arr_rwy(self, apt_id: str, rwy_id: str):
        """
        Remove an arrival runway from the TRACON.
        """

        # If the TRACON is active, there needs to be at least one runway open
        if self._active and len(self.arr_rwy_id[apt_id]) <= 1:
            print("Error: Cannot remove arrival runway from an active TRACON. At least one runway must be open.")
            return False
        
        # Remove the arrival runway
        if apt_id in self.arr_rwy_id and rwy_id in self.arr_rwy_id[apt_id]:
            self.arr_rwy_id[apt_id].remove(rwy_id)
            if len(self.arr_rwy_id[apt_id]) == 0:
                del self.arr_rwy_id[apt_id]
            return True
        else:
            print(f"Error: Arrival runway {rwy_id} does not exist.")
            return False


    def add_departure_wp(self, wp_id: str, lat: float, lon: float, ele: float):
        """
        Add a departure waypoint
        """
        
        # At most 8 departure waypoints
        if len(self.depart_wp) >= 8:
            print("Error: The number of departure waypoints exceeds the limit (8).")
            return False
        
        # Check if the waypoint ID already exists
        if wp_id in self.depart_wp:
            print(f"Error: Departure waypoint ID {wp_id} already exists.")
            return False
        
        # Check distance and altitude
        _, dist_from_ctr = qdrdist(self.ctr_lat, self.ctr_lon, lat, lon)
        if dist_from_ctr > self.range + DELETE_DISTANCE:
            print(f"Error: Departure waypoint {wp_id} is out of range.")
            return False
        if dist_from_ctr < self.range:
            print(f"Error: Departure waypoint {wp_id} is too close to the center.")
            return False
        if ele < self.elevation + 3000:
            print(f"Error: Departure waypoint {wp_id} is too low.")
            return False
        
        # Add the departure waypoint
        self.depart_wp[wp_id] = np.array([lat, lon, ele], dtype=float)
        return True
        

    def remove_departure_wp(self, wp_id: str):
        """
        Remove a departure waypoint
        """
        
        # If the TRACON is active, there needs to be at least one waypoint open
        if self._active and len(self.depart_wp) <= 1:
            print("Error: Cannot remove departure waypoint from an active TRACON. There needs to be at least one departure waypoint.")
            return False
        
        # Remove the departure waypoint
        if wp_id in self.depart_wp:
            del self.depart_wp[wp_id]
            return True
        else:
            print(f"Error: Departure waypoint {wp_id} does not exist.")
            return False


    def _gui_draw(self):
        """
        Draw the TRACON area on the GUI.
        """
        if bs.mode == 'sim':
            return False
        
        # centralize
        zoom = 1.0 * 30 / self.range
        stack.stack(f"PAN {self.ctr_lat}, {self.ctr_lon}; ZOOM {zoom}")

        # Draw the TRACON area
        stack.stack(f"CIRCLE {self.identifier}, {self.ctr_lat}, {self.ctr_lon}, {self.range}")
        stack.stack(f"COL {self.identifier} GREEN")

        # Turn on aircraft symbol
        stack.stack("SYMBOL")


    def _obtain_airport_info(self):
        """
        Update airport, runway, fap info from the database.
        
        Args:
            apt_id: Airport ID to obtain information for
        """
        
        if not self.apt:
            return True

        # Get airport information:
        for apt in self.apt:
            idx = navdb.getaptidx(apt)
            if idx < 0:
                print(f"Error: Airport ID {apt} is not in database.")
                return False
            self.apt[apt] = np.array([navdb.aptlat[idx], navdb.aptlon[idx], navdb.aptele[idx]], dtype=float)
        
        # Get runway threshold lat, lon and bearing:
        for apt in self.apt:
            try:
                self.rwy_thres[apt] = navdb.rwythresholds[apt]
            except KeyError:
                print(f"Error: Unable to load runway thresholds for airport {apt}.")
                return False
        
        # Get final approach points:
        for apt in self.apt:
            self.fap[apt] = dict()
            for rwy in self.rwy_thres[apt]:
                rwy_thres_lat, rwy_thres_lon, rwy_bearing = self.rwy_thres[apt][rwy]
                fap_lat, fap_lon = self._calculate_fap(rwy_thres_lat, rwy_thres_lon, rwy_bearing)
                self.fap[apt][rwy] = np.array([fap_lat, fap_lon, self.apt_ele[apt] + FAP_ELEVATION], dtype=float)

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

        _, dist_from_ctr = qdrdist(self.ctr_lat, self.ctr_lon, self.apt[apt_id][0], self.apt[apt_id][1])
        if dist_from_ctr > self.range:
            print(f"Airport {apt_id} is out of range.")
            return False
        if self.apt[apt_id][2] > self.top - 3000:
            print(f"Elevation of {apt_id} is too large (> Top Altitude - 3000).")
            return False
        if self.apt[apt_id][2] < self.bottom - 2000:
            print(f"Elevation of {apt_id} is too small (< Bottom Altitude - 2000).")
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

