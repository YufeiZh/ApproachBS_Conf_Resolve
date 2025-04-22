"""
ApproachBS plugin for BlueSky

This plugin adds terminal area (TRACON) operations to BlueSky.
"""
import random
import numpy as np
from matplotlib.path import Path
import bluesky as bs
from bluesky import core, stack, sim, traf, navdb
from bluesky.traffic.asas import ConflictDetection
from bluesky.tools.areafilter import basic_shapes, deleteArea, defineArea, checkInside
from bluesky.tools.aero import ft, nm, kts
from bluesky.tools.geo import qdrpos, qdrdist, kwikqdrdist_matrix

# Constants
DELETE_BELOW = 200  # Aircraft (except taking off) below this field elevation will be automatically deleted [ft].
DELETE_DISTANCE = 5     # Aircraft (except arrival acf not in airspace yet) whose distance from the airspace is greater than
                         # this number will be automatically deleted [NM].
FAP_ELEVATION = 2000    # Final approach point elevation [ft].
GLIDE_SLOPE = 3.0   # Glide slope [deg].

# navdb = Navdatabase()
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

    # Stack commands
    stackfunctions = {
        'APPBS_CRE': [
            'APPBS_CRE acfid, acftype, lat, lon, hdg, alt, spd, intention, target_lat, target_lon',
            'txt,txt,latlon,[hdg,alt,spd,txt,latlon]',
            approach_bs.appbs_cre,
            'Create an aircraft.'
        ],
        'APPBS_ACTIVATE': [
            'APPBS_ACTIVATE',
            '',
            approach_bs.appbs_activate,
            'Activate the TRACON.'
        ],
        'APPBS_BRIEF': [
            'APPBS_BRIEF',
            '',
            approach_bs.appbs_brief,
            'Brief the TRACON.'
        ],
        'APPBS_POS': [
            'APPBS_POS acid',
            '[acid]',
            approach_bs.appbs_pos,
            'Get info of the aircraft.'
        ],
        'APPBS_SET_GENRATE': [
            'APPBS_SET_GENRATE arr_gen_rate, dep_gen_rate',
            '[float,float]',
            approach_bs.appbs_set_genrate,
            'Set the generation rate of arrival and departure aircrafts.'
        ],
        'APPBS_LOS': [
            'APPBS_LOS',
            '',
            approach_bs.appbs_los,
            'Display Loss of Separation.'
        ],
        'APPBS_INVASION': [
            'APPBS_INVASION',
            '',
            approach_bs.appbs_invasion,
            'Display area invasion.'
        ],
        'APPBS_DEBUG': [
            'APPBS_DEBUG',
            '',
            approach_bs.appbs_debug,
            'Debug'
        ]
    }
    
    return config, stackfunctions


# Plugin class
class ApproachBS(core.Entity):
    ''' Entity object for BlueSky. '''
    def __init__(self):
        super().__init__()

        self.mode = 'default'         # Simulation mode: default, debug, or sim
        
        # Simulator settings
        if self.mode == 'default':
            stack.stack("ASAS ON")

        # Tracon
        self.tracon = Tracon()

        # Conflict detector
        self.conflict_detector = MyConflictDetection()
        # Invasion detector
        self.invasion_detector = InvasionDetection()

        with self.settrafarrays():

            self.under_ctrl = np.array([], dtype=bool)  # Whether acf is under control (in airspace)
            self.radar_vector = np.array([], dtype=bool)  # Whether acf is being radar vectored or on self-navigation
                                                            # This will be used to assess whether an acf leaves airspace unexpectly
            self.intention = np.array([])   # 0 for arrival, 1 for departure, -1 for unknown
            self.wp_id = []                 # ID of the target waypoint (for departure acfs) or arrival runway (for arrival acfs)
            self.wp_lat = np.array([])      # Latitude of the target waypoint [deg]
            self.wp_lon = np.array([])      # Longitude of the target waypoint [deg]
            self.wp_alt = np.array([])      # Designated altitude of the target waypoint [ft]
            self.rwy_course = np.array([])  # True heading (magnatic, aka "bearing" in BlueSky) of assigned runway (for arrival acfs only) [deg]
            self.dist_to_wp = np.array([])  # Distance to the target waypoint [nm]
            self.bearing_to_wp = np.array([])  # Bearing to the target waypoint [deg]
            self.time_entered = np.array([])    # when aircraft entered the controlled airspace [s]
            self.time_last_cmd = np.array([])   # when aircraft recieved the previous command [s]

            # Additional status
            self.take_over = np.array([], dtype=bool)   # Radar take-over or not. Not take-over aircrafts will not be deleted even if deletion condition is met
                                                        # All aircrafts are not take-over when spawning, until they enter the TRACON airspace.
            self.out_of_range = np.array([], dtype=bool)    # out-of-range acfs will be deleted if it is a radar take-over aircraft

        # Copy attibutes to monitor changes
        # To monitor whether aircraft received a command
        self.prev_acf_id = traf.id.copy()   # Previous aircraft IDs
        self.prev_selspd = traf.selspd.copy()   # Previous selected speed [m/s]
        self.prev_selalt = traf.selalt.copy()   # Previous selected altitude [m]
        self.prev_seltrk = traf.aporasas.trk.copy() # Previous selected track [deg]
        # To monitor entering/leaving the airspace and vectoring status
        self.prev_under_ctrl = self.under_ctrl.copy()   # Previous under control status
        self.prev_radar_vector = self.radar_vector.copy()   # Previous radar vector status

        # Aircraft generation settings
        self.auto_gen_setting = {
            "dep_gen_rate": 0.0,    # Aircraft generation rate [aircraft/min]
            "arr_gen_rate": 0.0     # Aircraft generation rate [aircraft/min]
        }
        self.last_dep_gen_time = 0.0  # Last generation time for departure aircrafts [s]
        self.last_arr_gen_time = 0.0  # Last generation time for arrival aircrafts [s]
        self.next_dep_gen_sep = float('inf')
        self.next_arr_gen_sep = float('inf')
        self.recent_arr_directions = [] # Record recent incoming aircraft directions to avoid conflicts
        self.recent_arr_directions_max = 1 # Maximum number of recent directions to record
        
        # Timers
        self.update_timer = core.Timer(name='appbs_update', dt=0.1)
        self.confinvasion_timer = core.Timer(name='appbs_invasion', dt=0.5)
        self.deletion_timer = core.Timer(name='appbs_auto_delete_acfs', dt=0.5)


    def appbs_cre(self, 
                  acfid,
                  acftype = "B737",
                  lat = 361.0,
                  lon = 361.0,
                  hdg = None, 
                  alt = 0.0, 
                  spd = 0.0, 
                  intention = "DEPARTURE",
                  target_lat = 361.0,
                  target_lon = 361.0
                  ):
        """ 
        Create an aircraft. Provides stack command APPBS_CRE.
        This is the preferred method to create an aircraft in ApproachBS in place of CRE.
        """

        if not self.tracon.is_active():
            msg = f"Aircraft {acfid} is not created. The TRACON is not active."
            if self.mode != 'sim':
                stack.stack(f"ECHO {msg}")
            return False, msg
        
        if len(traf.id) >= 50:
            msg = f"Aircraft {acfid} is not created. The number of aircrafts has reached the upper limit 50."
            if self.mode != 'sim':
                stack.stack(f"ECHO {msg}")
            return False, msg
        
        if acfid in traf.id:
            msg = f"Aircraft {acfid} is not created. Aircraft {acfid} already exists."
            if self.mode != 'sim':
                stack.stack(f"ECHO {msg}")
            return False, msg
        
        if intention not in ["ARRIVAL", "DEPARTURE"]:
            msg = f"Aircraft {acfid} is not created. The intention {intention} is not valid."
            if self.mode != 'sim':
                stack.stack(f"ECHO {msg}")
            return False, msg

        # Departure aircraft
        if intention == "DEPARTURE":
            # Parse the position
            if lat == 361.0 or lon == 361.0:
                if self.mode != 'sim':
                    stack.stack(f"ECHO Aircraft position is not specified. Using random runway.")
                # Choose a random airport and runway for take-off
                apt = random.choice(list(self.tracon.dep_rwy_id.keys()))
                rwy = random.choice(self.tracon.dep_rwy_id[apt])
                # Get the runway threshold position, runway course, and elevation
                lat, lon = self.tracon.rwy_thres[apt][rwy][:2]
                hdg = self.tracon.rwy_thres[apt][rwy][2]
                alt = self.tracon.apt[apt][2]
                spd = 0.0
            else:
                # Find the nearest runway threshold position
                min_dist, min_apt, min_rwy = float('inf'), None, None
                for apt in self.tracon.dep_rwy_id:
                    for rwy in self.tracon.dep_rwy_id[apt]:
                        rwy_lat, rwy_lon = self.tracon.rwy_thres[apt][rwy][:2]
                        dist = qdrdist(lat, lon, rwy_lat, rwy_lon)[1]
                        if dist < min_dist:
                            min_dist = dist
                            min_apt = apt
                            min_rwy = rwy
                if min_dist >= 1.0:
                    if self.mode != 'sim':
                        stack.stack(f"ECHO Aircraft position is not a valid runway. Using the closest runway.")
                lat, lon = self.tracon.rwy_thres[min_apt][min_rwy][:2]
                hdg = self.tracon.rwy_thres[min_apt][min_rwy][2]
                alt = self.tracon.apt[min_apt][2]
                spd = 0.0
            # Parse the departure waypoint
            if target_lat == 361.0 or target_lon == 361.0:
                if self.mode != 'sim':
                    stack.stack(f"ECHO Aircraft target waypoint is not specified. Using random departure waypoint.")
                # Choose a random departure waypoint
                target_wp_id = random.choice(list(self.tracon.depart_wp.keys()))
            else:
                # Find the nearest departure waypoint
                min_dist, target_wp_id = float('inf'), None
                for wp_id in self.tracon.depart_wp:
                    wp_lat, wp_lon = self.tracon.depart_wp[wp_id][:2]
                    dist = qdrdist(target_lat, target_lon, wp_lat, wp_lon)[1]
                    if dist < min_dist:
                        min_dist = dist
                        target_wp_id = wp_id
                if min_dist >= 2.0:
                    if self.mode != 'sim':
                        stack.stack(f"ECHO Aircraft target waypoint is not valid. Using the closest departure waypoint.")

        # Arrival aircraft
        else:
            # Parse the position
            if lat == 361.0 or lon == 361.0:
                if self.mode != 'sim':
                    stack.stack(f"ECHO Aircraft position is not specified. Using random position.")
                # Same logic as generating random arrival aircrafts
                count = 0
                while True:
                    direction = random.random() * 360.0
                    min_alt, max_alt = self.tracon.get_vertical_envelope(direction)
                    if max_alt <= self.tracon.top - 1100 or min_alt >= self.tracon.top - 900 or count > 20:
                        alt = self.tracon.top - 1000
                        break
                    count += 1
                lat, lon = qdrpos(self.tracon.ctr_lat, self.tracon.ctr_lon, direction, self.tracon.range + 5.0)
                hdg = (direction + 180) % 360
                spd = 250.0 * kts
            else:
                if not hdg:
                    hdg = qdrdist(lat, lon, self.tracon.ctr_lat, self.tracon.ctr_lon)[0]
                if alt < self.tracon.elevation:
                    alt = self.tracon.elevation + 3000.0
                if spd < 180.0 * kts:
                    spd = 180.0 * kts
            # Parse the arrival runway
            if target_lat == 361.0 or target_lon == 361.0:
                if self.mode != 'sim':
                    stack.stack(f"ECHO Arrival airport is not specified. Using random arrival airport.")
                # Choose a random arrival airport and runway
                target_apt = random.choice(list(self.tracon.arr_rwy_id.keys()))
                target_rwy = random.choice(self.tracon.arr_rwy_id[apt])
            else:
                # Find the nearest arrival airport
                min_dist, target_apt = float('inf'), None
                for apt in self.tracon.arr_rwy_id:
                    apt_lat, apt_lon = self.tracon.apt[apt][:2]
                    dist = qdrdist(target_lat, target_lon, apt_lat, apt_lon)[1]
                    if dist < min_dist:
                        min_dist = dist
                        target_apt = apt
                if min_dist >= 1.0:
                    if self.mode != 'sim':
                        stack.stack(f"ECHO Arrival airport is not valid or not open for arrival. Using the closest arrival airport.")
                target_rwy = random.choice(self.tracon.arr_rwy_id[target_apt])
        
        # Create new aircraft using traf.cre
        traf.cre(acid=acfid,
                 actype=acftype,
                 aclat=lat,
                 aclon=lon,
                 achdg=hdg,
                 acalt=alt,
                 acspd=spd)
        
        # Assign departure waypoint if departure
        if intention == "DEPARTURE":
            self.intention[-1] = 1
            # Set target waypoint to the departure waypoint of the assigned runway
            self.wp_id[-1] = target_wp_id
            self.wp_lat[-1], self.wp_lon[-1], self.wp_alt[-1] = self.tracon.depart_wp[target_wp_id]
            if self.mode != 'sim':
                stack.stack(f"ECHO {acfid}: to depart {target_wp_id}.")
            # Take-off clearance
            stack.stack(f"ALT {acfid} {self.tracon.bottom + 1000}; SPD {acfid} 180")

        # Assign runway if arrival
        else:
            self.intention[-1] = 0
            # Set target waypoint to the FAP of the assigned runway
            self.wp_id[-1] = f"{target_apt}/{target_rwy}"
            self.wp_lat[-1], self.wp_lon[-1], self.wp_alt[-1] = self.tracon.fap[target_apt][target_rwy]
            # Set runway course
            self.rwy_course[-1] = self.tracon.rwy_thres[target_apt][target_rwy][2]
            if self.mode != 'sim':
                stack.stack(f"ECHO {acfid}: to land {target_apt}/{target_rwy}.")

        return True


    def cre_acf_from_dict(self, acfinfo: dict) -> None:
        """
        Create an aircraft
        Parameter acfinfo can be found in aircraft.py
        """

        self.appbs_cre(acfid=acfinfo['callsign'].upper(),
                       acftype=acfinfo['acf_type'].upper(),
                       lat=acfinfo['lat'], 
                       lon=acfinfo['lon'],
                       hdg=acfinfo['trk'],
                       alt=acfinfo['alt'] * ft,
                       spd=acfinfo['cas'] * kts,
                       intention=acfinfo['intention'],
                       target_lat=acfinfo['target_lat'],
                       target_lon=acfinfo['target_lon'])


    def create(self, n=1):
        ''' This function gets called automatically when new aircrafts are created. '''
        super().create(n)
        # Initialize additional attributes
        self.under_ctrl[-n:] = False
        self.radar_vector[-n:] = False

        self.take_over[-n:] = False
        self.out_of_range[-n:] = False
        
        self.intention[-n:] = -1
        for i in range(n):
            self.wp_id[-n+i] = "N/A"
        self.wp_lat[-n:] = 361.0
        self.wp_lon[-n:] = 361.0
        self.wp_alt[-n:] = -1000.0
        self.rwy_course[-n:] = 361.0
        self.dist_to_wp[-n:] = -1.0
        self.bearing_to_wp[-n:] = 361.0
        self.time_entered[-n:] = -60.0
        self.time_last_cmd[-n:] = -60.0


    @core.timed_function(name='appbs_update_manager', dt=0.1)
    def update(self):
        ''' This function gets called automatically every 0.1 second. '''

        # Delete excess aircrafts
        if self.deletion_timer.readynext() and self.tracon.is_active():
            self.appbs_delete_excess_acfs()

        # Delete out-of-range aircrafts
        if self.deletion_timer.readynext() and self.tracon.is_active():
            self.appbs_auto_delete_acfs()

        # Update aircraft status
        if self.update_timer.readynext() and self.tracon.is_active():
            self.appbs_update()

        # Update conflict detectors
        if self.confinvasion_timer.readynext() and self.tracon.is_active():
            self.conflict_detector.update(traf, traf)
            self.invasion_detector.update(traf, self.tracon)

        # Generate departure aircrafts
        if self.tracon.is_active() and self.auto_gen_setting['dep_gen_rate'] > 0.0:
            self.generate_dep_acf()

        # Generate arrival aircrafts
        if self.tracon.is_active() and self.auto_gen_setting['arr_gen_rate'] > 0.0:
            self.generate_arr_acf()

        # No intention aircrafts
        if self.update_timer.readynext() and self.tracon.is_active():
            self.handle_no_intention_acfs()


    def appbs_update(self):
        ''' Periodically update aircraft status. '''

        if not self.tracon.is_active():
            return
        
        # out of range
        dist_from_ctr = qdrdist(self.tracon.ctr_lat, self.tracon.ctr_lon, traf.lat, traf.lon)[1]
        self.out_of_range = (dist_from_ctr > self.tracon.range + DELETE_DISTANCE) | (traf.alt < self.tracon.elevation + DELETE_BELOW)

        # under_ctrl
        self.under_ctrl = (dist_from_ctr < self.tracon.range) & (traf.alt >= self.tracon.bottom * ft) & (traf.alt <= self.tracon.top * ft)

        # Take over aircrafts in the TRACON airspace
        self.take_over = self.take_over | self.under_ctrl

        # Radar vector
        # Do not vector if the aircraft has LNAV on
        self.radar_vector[traf.swlnav] = False
        # vector in-airspace aircrafts that has LNAV off
        self.radar_vector[self.under_ctrl & ~traf.swlnav] = True

        # Handle aircrafts that just entered or left the TRACON airspace
        for i in range(len(self.prev_acf_id)):
            if self.prev_acf_id[i] in traf.id:
                new_index = traf.id.index(self.prev_acf_id[i])
                if not self.prev_under_ctrl[i] and self.under_ctrl[new_index]:
                    # Aircraft just entered the TRACON airspace
                    traf.swlnav[new_index] = False  # Turn off LNAV
                    self.radar_vector[new_index] = True # Vectoring
                    self.time_entered[new_index] = sim.simt  # Record the entering time

        # Update distance and bearing to the target waypoint
        mask = (self.wp_lat != 361.0) & (self.wp_lon != 361.0)
        self.bearing_to_wp[mask], self.dist_to_wp[mask] = qdrdist(traf.lat[mask], traf.lon[mask], self.wp_lat[mask], self.wp_lon[mask])
        self.bearing_to_wp[mask] = (self.bearing_to_wp[mask] + 360) % 360  # Normalize to [0, 360)

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
        # Record the time
        self.time_last_cmd = np.where(received_cmd, sim.simt, self.time_last_cmd)

        # Update previous status
        self.prev_acf_id = traf.id.copy()
        self.prev_selspd = traf.selspd.copy()
        self.prev_selalt = traf.selalt.copy()
        self.prev_seltrk = traf.aporasas.trk.copy()
        self.prev_under_ctrl = self.under_ctrl.copy()
        self.prev_radar_vector = self.radar_vector.copy()


    def appbs_delete_excess_acfs(self):
        ''' Periodically delete aircrafts if there re more than 50 aircrafts. '''
        delete_id = []
        while traf.ntraf > 50:
            delete_id.append(traf.id[-1])
            traf.delete(-1)
        if delete_id and self.mode != 'sim':
            stack.stack(f"ECHO Aircrafts {delete_id} are deleted due to excess number of aircrafts (50).")

    
    def appbs_auto_delete_acfs(self):
        ''' Periodically delete out-of-range aircrafts. '''
        if not self.tracon.is_active():
            return
        i = 0
        while i < len(traf.id):
            if self.take_over[i] and self.out_of_range[i]:
                if self.mode != 'sim':
                    stack.stack(f"ECHO Aircraft {traf.id[i]} is deleted due to out-of-range.")
                traf.delete(i)
                i -= 1
            elif self.mode == 'default':
                if qdrdist(self.tracon.ctr_lat, self.tracon.ctr_lon, traf.lat[i], traf.lon[i])[1] > self.tracon.range + 20:
                    # Force deleting aircrafts too far away from the TRACON
                    if self.mode != 'sim':
                        stack.stack(f"ECHO Aircraft {traf.id[i]} is deleted due to out-of-range.")
                    traf.delete(i)
                    i -= 1
            i += 1


    def generate_dep_acf(self):
        ''' Periodically generate departure aircrafts based on generation settings. '''
        if not self.tracon.is_active() \
            or self.auto_gen_setting['dep_gen_rate'] <= 0.0 \
                or traf.ntraf >= 50 \
                    or sim.simt-self.last_dep_gen_time < self.next_dep_gen_sep:
            return
        
        # Choose a random departure airport first
        count = 0
        while True:
            apt = random.choice(list(self.tracon.dep_rwy_id.keys()))
            # Cannot take-off from this airport if there are aircrafts within 5NM
            if traf.ntraf == 0 or np.min(qdrdist(self.tracon.apt[apt][0], self.tracon.apt[apt][1], traf.lat, traf.lon)[1]) > 5.0:
                break
            count += 1
            if count > 5:
                # No valid airpot found, abort generation
                return

        rwy = random.choice(self.tracon.dep_rwy_id[apt])
        
        # Generate a departure aircraft
        # Choose a random departure waypoint
        waypoint = random.choice(list(self.tracon.depart_wp.keys()))
        # Generate a random aircraft ID
        acfid = "DEP"
        for i in range(1, 51):
            if f"{acfid}{i:04}" not in traf.id:
                acfid = f"{acfid}{i:04}"
                break
        # Use default aircraft type B737
        self.appbs_cre(acfid=acfid,
                        acftype="B737",
                        lat=self.tracon.rwy_thres[apt][rwy][0],
                        lon=self.tracon.rwy_thres[apt][rwy][1],
                        intention="DEPARTURE",
                        target_lat=self.tracon.depart_wp[waypoint][0],
                        target_lon=self.tracon.depart_wp[waypoint][1])
        # Update the last generation time
        self.last_dep_gen_time = sim.simt
        # Sample next generation separation time
        gen_separation = 60.0 / self.auto_gen_setting['dep_gen_rate']
        self.next_dep_gen_sep = random.gauss(gen_separation, gen_separation / 10)


    def generate_arr_acf(self):
        ''' Periodically generate arrival aircrafts based on generation settings. '''
        if not self.tracon.is_active() \
            or self.auto_gen_setting['arr_gen_rate'] <= 0.0 \
                or traf.ntraf >= 50 \
                    or sim.simt-self.last_arr_gen_time < self.next_arr_gen_sep:
            return
        
        # Generate an arrival aircraft
        # Determaine the approach direction and altitude
        count = 0
        while True:
            # Choose a direction from the center of the TRACON
            direction = random.random() * 360.0
            # Avoid recent approach directions
            valid_direction = True
            for d in self.recent_arr_directions:
                d_delta = abs(direction - d)
                if d_delta > 180.0:
                    d_delta = 360.0 - d_delta
                if d_delta < 15.0:
                    valid_direction = False
                    break
            if valid_direction:
                # Get the vertical envelope from this direction
                min_alt, max_alt = self.tracon.get_vertical_envelope(direction)
                # Arrive from this direction if the altitude restriction allows
                if max_alt <= self.tracon.top - 1100 or min_alt >= self.tracon.top - 900:
                    alt = self.tracon.top - 1000
                    break
            count += 1
            if count > 30:
                if self.mode != 'sim':
                    stack.stack(f"ECHO Unable to generate arrival aircrafts at this time.")
                return
            
        # Get the position of the aircraft
        lat, lon = qdrpos(self.tracon.ctr_lat, self.tracon.ctr_lon, direction, self.tracon.range + 5.0)
        # Choose a random arrival runway
        apt = random.choice(list(self.tracon.arr_rwy_id.keys()))
        rwy = random.choice(self.tracon.arr_rwy_id[apt])
        # Generate a random aircraft ID
        acfid = "ARR"
        for i in range(1, 51):
            if f"{acfid}{i:04}" not in traf.id:
                acfid = f"{acfid}{i:04}"
                break
        # Use default aircraft type B737
        # Always flying towards the center of the TRACON
        self.appbs_cre(acfid=acfid,
                        acftype="B737",
                        lat=lat,
                        lon=lon,
                        hdg=(direction + 180) % 360,
                        alt=alt * ft,
                        spd=250 * kts,
                        intention="ARRIVAL",
                        target_lat=self.tracon.rwy_thres[apt][rwy][0],
                        target_lon=self.tracon.rwy_thres[apt][rwy][1])
        # Update the last generation time
        self.last_arr_gen_time = sim.simt
        # Sample next generation separation time
        gen_separation = 60.0 / self.auto_gen_setting['arr_gen_rate']
        self.next_arr_gen_sep = random.gauss(gen_separation, gen_separation / 10)
        # Record the recent approach direction
        if len(self.recent_arr_directions) >= self.recent_arr_directions_max:
            self.recent_arr_directions.pop(0)
        self.recent_arr_directions.append(direction)


    def handle_no_intention_acfs(self):
        """ Automatically set the intention to Departure for aircrafts without intention. """
        if not self.tracon.is_active():
            return
        
        mask_no_intention  = (self.intention == -1) & (self.under_ctrl)
        self.intention[mask_no_intention] = 1  # Set intention to Departure
        # Target waypoint
        depart_wp = random.choice(list(self.tracon.depart_wp.keys()))
        for i in range(len(self.wp_id)):
            if mask_no_intention[i]:
                self.wp_id[i] = depart_wp
        self.wp_lat[mask_no_intention] = self.tracon.depart_wp[depart_wp][0]
        self.wp_lon[mask_no_intention] = self.tracon.depart_wp[depart_wp][1]
        self.wp_alt[mask_no_intention] = self.tracon.depart_wp[depart_wp][2]
        # Enter time
        self.time_entered[mask_no_intention] = sim.simt

        if self.mode != 'sim' and np.any(mask_no_intention):
            stack.stack(f"ECHO Aircrafts without intention are set to Departure. Target waypoint: {depart_wp}.")


    def appbs_activate(self):
        """ Activate the TRACON. """
        if self.tracon.is_active():
            if self.mode != 'sim':
                stack.stack("ECHO The airspace is already activated.")
            return False
        
        # Activate the TRACON
        self.tracon.activate()
        active, msg = self.tracon.activate()
        if not active:
            if self.mode != 'sim':
                stack.stack(f"ECHO {msg}")
            return False
        if self.mode != 'sim':
            stack.stack("ECHO The airspace is activated.")
        return True
    

    def appbs_brief(self):
        """
        Brief the info of the area 
        """
        if self.mode == 'sim':
            return True
        
        line = f"{self.tracon.identifier}: "
        line += "ACTIVE, " if self.tracon.is_active() else "INACTIVE, "
        line += f"RANGE: {self.tracon.range}NM/{self.tracon.bottom}FT to {self.tracon.top}FT, "
        line += f"ACFS: {len(traf.id)}, "
        line += f"IN AREA: {np.sum(self.under_ctrl)}"
        stack.stack(f"ECHO {line}")

        # open runway info
        line = "DEP RWY: "
        for apt in self.tracon.dep_rwy_id:
            for rwy in self.tracon.dep_rwy_id[apt]:
                line += f"{apt}/{rwy}, "
        stack.stack(f"ECHO {line}")
        line = "ARR RWY: "
        for apt in self.tracon.arr_rwy_id:
            for rwy in self.tracon.arr_rwy_id[apt]:
                line += f"{apt}/{rwy}, "
        stack.stack(f"ECHO {line}")

        # generation rate
        line = f"ARR AIRCRAFTS GENERATION: {self.auto_gen_setting['arr_gen_rate']:.1f}ACFS/MIN\t"
        line += f"DEP AIRCRAFTS GENERATION: {self.auto_gen_setting['dep_gen_rate']:.1f}ACFS/MIN"
        stack.stack(f"ECHO {line}")
        
        return True


    def appbs_pos(self, acfidx: int=None):
        """ 
        Echo the information of the aircraft. 
        acfid: index of the aircraft or None for all aircrafts.
        """
        if self.mode == 'sim':
            return True

        def pos_acf(acfidx):
            """ Get the info of single aircraft. """
            msg = f"{traf.id[acfidx]}\t"

            # position
            bearing, dist = qdrdist(self.tracon.ctr_lat, self.tracon.ctr_lon, traf.lat[acfidx], traf.lon[acfidx])
            bearing = (bearing + 360) % 360
            if 22.5 <= bearing < 67.5:
                direction = "NE"
            elif 67.5 <= bearing < 112.5:
                direction = "E "
            elif 112.5 <= bearing < 157.5:
                direction = "SE"
            elif 157.5 <= bearing < 202.5:
                direction = "S "
            elif 202.5 <= bearing < 247.5:
                direction = "SW"
            elif 247.5 <= bearing < 292.5:
                direction = "W "
            elif 292.5 <= bearing < 337.5:
                direction = "NW"
            else:
                direction = "N "
            dist = round(dist, 1)
            msg += f"{dist}/{direction}\t"

            # dynamics
            trk = round(traf.trk[acfidx])
            alt = round(traf.alt[acfidx] / ft)
            spd = round(traf.cas[acfidx] / kts)
            gs = round(traf.gs[acfidx] / kts)
            msg += f"at {trk}/{alt}/{spd}/{gs}\t"

            # intention
            if self.intention[acfidx] == 0:
                msg += f"ARR to {self.wp_id[acfidx]}\t"
            elif self.intention[acfidx] == 1:
                msg += f"DEP to {self.wp_id[acfidx]}\t"
            else:
                msg += "N/A\t"

            # vector
            msg += "VEC\t" if self.radar_vector[acfidx] else "NAV\t"

            # debug mode only
            if self.mode == 'debug':
                msg += f"(WP:{self.dist_to_wp[acfidx]:.1f}/{self.bearing_to_wp[acfidx]:.1f}/{self.rwy_course[acfidx]:.1f})\t"
                msg += f"(T:{self.time_entered[acfidx]:.1f}/{self.time_last_cmd[acfidx]:.1f})\t"
                msg += f"(TO/OR:{self.take_over[acfidx]}/{self.out_of_range[acfidx]})\t"

            # other status
            if not self.under_ctrl[acfidx]:
                msg += "OUT\t"

            return msg

        if acfidx is not None and 0 <= acfidx < len(traf.id):
            stack.stack(f"ECHO {pos_acf(acfidx)}")
            return True
        else:
            for idx in range(len(traf.id)):
                stack.stack(f"ECHO {pos_acf(idx)}")
            return True 


    def appbs_set_genrate(self, dep_rate: float=None, arr_rate: float=None):
        """ 
        Set the generation rate of departure and arrival aircrafts. 
        """
        if dep_rate is None or dep_rate < 0.0:
            dep_rate = 0.0
        if arr_rate is None or arr_rate < 0.0:
            arr_rate = 0.0
        
        self.auto_gen_setting['dep_gen_rate'] = dep_rate
        self.auto_gen_setting['arr_gen_rate'] = arr_rate
        self.next_arr_gen_sep = self.next_dep_gen_sep = 0.0

        self.recent_arr_directions_max = int(arr_rate) + 1
        return True


    def appbs_los(self):
        """
        Show the current, one-minute, and three-minute loss of separation between aircrafts.
        """
        if self.mode == 'sim' or not self.tracon.is_active():
            return True
        
        if self.conflict_detector.lospairs_unique:
            stack.stack(f"ECHO Current LOS: {[tuple(pair) for pair in self.conflict_detector.lospairs_unique]}")
        else:
            stack.stack(f"ECHO Current LOS: None")

        if self.conflict_detector.conf_one_minute_unique:
            stack.stack(f"ECHO LOS in 1 minute: {[tuple(pair) for pair in self.conflict_detector.conf_one_minute_unique]}")
        else:
            stack.stack(f"ECHO LOS in 1 minute: None")

        if self.conflict_detector.conf_three_minute_unique:
            stack.stack(f"ECHO LOS in 3 minutes: {[tuple(pair) for pair in self.conflict_detector.conf_three_minute_unique]}")
        else:
            stack.stack(f"ECHO LOS in 3 minutes: None")

        return True


    def appbs_invasion(self):
        """
        Show the current area invasion status of aircrafts.
        """
        if self.mode == 'sim' or not self.tracon.is_active():
            return True
        
        if self.invasion_detector.current_invasion:
            stack.stack(f"ECHO Current Invasion: {self.invasion_detector.current_invasion}")
        else:
            stack.stack(f"ECHO Current Invasion: None")

        if self.invasion_detector.one_minute_invasion:
            stack.stack(f"ECHO Invasion in 1 minute: {self.invasion_detector.one_minute_invasion}")
        else:
            stack.stack(f"ECHO Invasion in 1 minute: None")

        if self.invasion_detector.three_minute_invasion:
            stack.stack(f"ECHO Invasion in 3 minutes: {self.invasion_detector.three_minute_invasion}")
        else:
            stack.stack(f"ECHO Invasion in 3 minutes: None")

        return True


    def appbs_debug(self):
        stack.stack(f"ECHO {id(self.conflict_detector)}, {id(traf.cd)}")


class MyConflictDetection(core.Entity, replaceable=False):
    """
    Override the ConflictDetection class to simultaneously detect conflicts in one minute and three minutes.
    """
    def __init__(self):
        super().__init__()
        ## Default values
        # [m] Horizontal separation minimum for detection
        self.rpz = 5.0 * nm
        # [m] Vertical separation minimum for detection
        self.hpz = 1000 * ft

        # Current LoS
        self.lospairs = list()
        self.lospairs_unique = set()
        self.lospairs_all = list()              # All LoS since beginning

        # Conflicts in one minute
        self.conf_one_minute = list()
        self.qdr_one_minute = np.array([])
        self.dist_one_minute = np.array([])
        self.dcpa_one_minute = np.array([])
        self.tcpa_one_minute = np.array([])
        self.tLOS_one_minute = np.array([])
        self.conf_one_minute_unique = set()
        self.conf_one_minute_all = list()       # All conflicts (one minute) since beginning

        # Conflicts in three minutes
        self.conf_three_minute = list()
        self.qdr_three_minute = np.array([])
        self.dist_three_minute = np.array([])
        self.dcpa_three_minute = np.array([])
        self.tcpa_three_minute = np.array([])
        self.tLOS_three_minute = np.array([])
        self.conf_three_minute_unique = set()
        self.conf_three_minute_all = list()     # All conflicts (three minutes) since beginning

        # Per-aircraft conflict data
        with self.settrafarrays():
            self.inconf_one_minute = np.array([], dtype=bool)  # In-conflict flag
            self.tcpamax_one_minute = np.array([]) # Maximum time to CPA for aircraft in conflict
            self.inconf_three_minute = np.array([], dtype=bool)  # In-conflict flag
            self.tcpamax_three_minute = np.array([]) # Maximum time to CPA for aircraft in conflict


    def clearconfdb(self):
        """ Clear the conflict database. """
        self.lospairs_unique.clear()
        self.lospairs.clear()
        self.conf_one_minute_unique.clear()
        self.conf_one_minute.clear()
        self.conf_three_minute_unique.clear()
        self.conf_three_minute.clear()

        self.qdr_one_minute = np.array([])
        self.dist_one_minute = np.array([])
        self.dcpa_one_minute = np.array([])
        self.tcpa_one_minute = np.array([])
        self.tLOS_one_minute = np.array([])
        
        self.qdr_three_minute = np.array([])
        self.dist_three_minute = np.array([])
        self.dcpa_three_minute = np.array([])
        self.tcpa_three_minute = np.array([])
        self.tLOS_three_minute = np.array([])

        self.inconf_one_minute = np.zeros(traf.ntraf, dtype=bool)
        self.tcpamax_one_minute = np.zeros(traf.ntraf)
        self.inconf_three_minute = np.zeros(traf.ntraf, dtype=bool)
        self.tcpamax_three_minute = np.zeros(traf.ntraf)

    
    def create(self, n):
        super().create(n)


    def reset(self):
        super().reset()
        self.clearconfdb()
        self.lospairs_all.clear()
        self.conf_one_minute_all.clear()
        self.conf_three_minute_all.clear()


    def update(self, ownship, intruder):
        ''' Update the conflicts. '''
        if ownship.ntraf == 0 or intruder.ntraf == 0:
            self.lospairs_unique.clear()
            self.conf_one_minute_unique.clear()
            self.conf_three_minute_unique.clear()
            return
        self.conf_one_minute, self.lospairs, self.inconf_one_minute, self.tcpamax_one_minute, self.qdr_one_minute, \
            self.dist_one_minute, self.dcpa_one_minute, self.tcpa_one_minute, self.tLOS_one_minute = \
                self.detect(ownship, intruder, self.rpz, self.hpz, 60.0)
        
        self.conf_three_minute, _, self.inconf_three_minute, self.tcpamax_three_minute, self.qdr_three_minute, \
            self.dist_three_minute, self.dcpa_three_minute, self.tcpa_three_minute, self.tLOS_three_minute = \
                self.detect(ownship, intruder, self.rpz, self.hpz, 180.0)

        # confpairs has conflicts observed from both sides (a, b) and (b, a)
        # confpairs_unique keeps only one of these
        lospairs_unique = {frozenset(pair) for pair in self.lospairs}
        conf_one_minute_unique = {frozenset(pair) for pair in self.conf_one_minute}
        conf_three_minute_unique = {frozenset(pair) for pair in self.conf_three_minute}

        self.lospairs_all.extend(lospairs_unique - self.lospairs_unique)
        self.conf_one_minute_all.extend(conf_one_minute_unique - self.conf_one_minute_unique)
        self.conf_three_minute_all.extend(conf_three_minute_unique - self.conf_three_minute_unique)

        # Update confpairs_unique and lospairs_unique
        self.lospairs_unique = lospairs_unique
        self.conf_one_minute_unique = conf_one_minute_unique
        self.conf_three_minute_unique = conf_three_minute_unique

    
    def detect(self, ownship, intruder, rpz, hpz, dtlookahead_def):
        ''' State-based conflict detection. Copied from the original code. '''
        # Expand rpz, hpz, dtlookahead to an array
        rpz = np.full(ownship.ntraf, rpz, dtype=float)
        hpz = np.full(ownship.ntraf, hpz, dtype=float)
        dtlookahead = np.full(ownship.ntraf, dtlookahead_def, dtype=float)
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


class InvasionDetection(core.Entity, replaceable=False):

    def __init__(self):
        # Dictionaries mapping area IDs to a list of invaded aircraft IDs.
        self.current_invasion = dict()
        self.one_minute_invasion = dict()
        self.three_minute_invasion = dict()

        with self.settrafarrays():
            self.acf_invasion_flag = np.zeros([], dtype=bool)  # Invasion flag (current)
            self.acf_invasion_flag_one_minute = np.zeros([], dtype=bool)  # Invasion flag (one minute)
            self.acf_invasion_flag_three_minute = np.zeros([], dtype=bool)  # Invasion flag (three minutes)


    def reset(self):
        super().reset()

        self.current_invasion = dict()
        self.one_minute_invasion = dict()
        self.three_minute_invasion = dict()


    def create(self, n):
        super().create(n)
        self.acf_invasion_flag[-n:] = False
        self.acf_invasion_flag_one_minute[-n:] = False
        self.acf_invasion_flag_three_minute[-n:] = False

    
    def update(self, intruder, tracon):
        """
        Update the area invasion status of aircrafts.
        """
        # Check current area invasion
        self.check_invasion(intruder, tracon)
        # Check one-minute and three-minute area invasion
        self.predict_invasions(intruder, tracon)


    def check_invasion(self, intruder, tracon):
        """
        Check current area invasion.
        Update self.current_invasion and self.acf_invasion_flag.
        """
        self.current_invasion = dict()
        # No restricted area or no aircrafts
        if intruder.ntraf == 0 or not tracon.restrict:
            return
        
        # Iterate through areas
        for area_id in tracon.restrict:
            inside_flag = checkInside(area_id, intruder.lat, intruder.lon, intruder.alt / ft)
            self.acf_invasion_flag[inside_flag] = True
            self.current_invasion[area_id] = list(intruder.id[inside_flag])


    def predict_invasions(self, intruder, tracon):
        """
        Check area invasion.
        Update self.one_minute_invasion, self.acf_invasion_flag_one_minute, self.three_minute_invasion, self.acf_invasion_flag_three_minute.
        """
        self.one_minute_invasion = dict()
        self.three_minute_invasion = dict()
        self.acf_invasion_flag_one_minute = np.zeros(intruder.ntraf, dtype=bool)  # Invasion flag (one minute)
        self.acf_invasion_flag_three_minute = np.zeros(intruder.ntraf, dtype=bool)  # Invasion flag (three minutes)
        if intruder.ntraf == 0 or not tracon.restrict:
            return
        
        # Iterate through areas
        for area_id in tracon.restrict:
            if tracon.restrict[area_id][0] == 0.0:
                # Circular area
                area_args = tracon.restrict[area_id][1:]
                inside_flag_one_minute = self.invasion_circle(60.0, area_args)
                inside_flag_three_minute = self.invasion_circle(180.0, area_args)
            else:
                # Polygonal area
                area_args = tracon.restrict[area_id][1:]
                inside_flag_one_minute = self.invasion_polygon(60.0, area_args)
                inside_flag_three_minute = self.invasion_polygon(180.0, area_args)
            
            self.acf_invasion_flag_one_minute[inside_flag_one_minute] = True
            self.one_minute_invasion[area_id] = list(intruder.id[inside_flag_one_minute])
            self.acf_invasion_flag_three_minute[inside_flag_three_minute] = True
            self.three_minute_invasion[area_id] = list(intruder.id[inside_flag_three_minute])


    def invasion_circle(self, intruder, dtlookahead, area_args):
        """
        Predict the invasion of a circular area.
        """
        
        bottom, top, ctr_lat, ctr_lon, radius = area_args

        # Ray endpoints
        end_lat, end_lon = qdrpos(intruder.lat, intruder.lon, intruder.trk, intruder.gs * dtlookahead / nm)

        # Vectors
        startpoints = np.stack([intruder.lat, intruder.lon], axis=1)
        endpoints = np.stack([end_lat, end_lon], axis=1)
        seg_vectors = endpoints - startpoints       # Vectors from start to end point
        seg_vec_norm = np.linalg.norm(seg_vectors, axis=1)      # Norm of the vectors

        # calculate the projection (of the startpoint-center vector) vector on the ray vector
        start_center_vecs = np.array([[ctr_lat, ctr_lon]]) - startpoints
        t_close = np.einsum('ij,ij->i', seg_vectors, start_center_vecs) / seg_vec_norm**2
        t_clamped = np.clip(t_close, 0, 1)  # Clamp t to [0, 1]
        closest_points = startpoints + t_clamped[:, np.newaxis] * seg_vectors  # Closest points on the segment to the center of the area
        _, distance = qdrdist(closest_points[:, 0], closest_points[:, 1], ctr_lat, ctr_lon)
        # Check if the closest point is within the radius of the area
        inside_flag = distance < radius
            # Get t_in and t_out
        virtual_closest = startpoints[inside_flag] + t_close[inside_flag][:, np.newaxis] * seg_vectors[inside_flag]
        _, virtual_distance = qdrdist(virtual_closest[:, 0], virtual_closest[:, 1], ctr_lat, ctr_lon)
        # Use Pythagorean
        _, dist_start_virtual = qdrdist(intruder.lat[inside_flag], intruder.lon[inside_flag], virtual_closest[:, 0], virtual_closest[:, 1])
        delta_t = np.sqrt(radius**2 - virtual_distance**2) / dist_start_virtual * t_close[inside_flag]
        t_in = np.clip(t_close[inside_flag] - delta_t, 0, 1)    # Clamp to [0, 1]
        t_out = np.clip(t_close[inside_flag] + delta_t, 0, 1)   # Clamp to [0, 1]
        # Get the altitude at t_in and t_out
        alt_in = (intruder.alt[inside_flag] + intruder.vs[inside_flag] * t_in * dtlookahead) / ft
        alt_out = (intruder.alt[inside_flag] + intruder.vs[inside_flag] * t_out * dtlookahead) / ft
        # Check if the aircraft enters or crosses the area
        below_in = alt_in < bottom
        above_in = alt_in > top
        below_out = alt_out < bottom
        above_out = alt_out > top
        not_crossing = (below_in & below_out) | (above_in & above_out)
        inside_flag[inside_flag] = ~not_crossing

        return inside_flag
    

    def invasion_polygon(self, intruder, dtlookahead, area_args):
        """
        Predict the invasion of a polygonal area.
        Use a simplified model: only consider the start and end altitude.
        """
        
        bottom, top = area_args[:2]
        vertices = area_args[2:]
        area_path = Path(vertices.reshape(-1, 2))

        # Ray endpoints
        end_lat, end_lon = qdrpos(intruder.lat, intruder.lon, intruder.trk, intruder.gs * dtlookahead / nm)
        end_alt = (intruder.alt + intruder.vs * dtlookahead) / ft

        # Vectors
        startpoints = np.stack([intruder.lat, intruder.lon], axis=1)
        endpoints = np.stack([end_lat, end_lon], axis=1)
        segments = np.stack([startpoints, endpoints], axis=1)   # Shape (ntraf, 2, 2)
        seg_paths = [Path(seg) for seg in segments]             # List of Path objects

        # intersect
        inside_flag = np.array([path.intersects_path(area_path) for path in seg_paths], dtype=bool)
        # Check if the aircraft is within the altitude range
        below_in = (intruder.alt / ft) < bottom
        above_in = (intruder.alt / ft) > top
        below_out = end_alt < bottom
        above_out = end_alt > top
        not_crossing = (below_in & below_out) | (above_in & above_out)
        inside_flag[inside_flag] = ~not_crossing[inside_flag]

        return inside_flag


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
        range: Radius of TRACON area (nm)
        elevation: Ground elevation (ft)
        apt: Dictionary mapping airport IDs to an ndarray [lat, lon, ele]
        rwy_thres: Dictionary mapping airport IDs to dictinaries
            rwy_thres[apt_id]: Dictionary of runway IDs to an ndarray [lat, lon, bearing]
        fap: Dictionary mapping airport IDs to dictionaries
            fap[apt_id]: Dictionary of runway IDs to an ndarray [lat, lon, alt]
        dep_rwy_id: Dictionary mapping airport IDs to list of open departure runway IDs, e.g., ["KSFO": ["28L", "28R"], "KSJC": ["30R"]]
        arr_rwy_id: Dictionary mapping airport IDs to list of open arrival runway IDs, e.g., ["KSFO": ["28L", "28R"], "KSJC": ["30R"]]
        depart_wp: Dictionary mapping departure waypoint IDs to an ndarray [lat, lon, alt], 
            e.g., {"EAONE": ndarray([33.8744, -83.8000, 10000]), "WEONE": ndarray([33.52569, -85.1224, 10000])}
        restrict: Dictionary mapping restricted area IDs to an ndarray of length 6 to 19
    """
    
    def __init__(self):
        """
        Initialize the TRACON object.
        """
        self._active = False

        self.identifier = "DEFAULT"
        
        self.ctr_lat = 0.0
        self.ctr_lon = 0.0
        self.top = 5000
        self.bottom = 1000
        self.range = 30.0
        self.elevation = 0.0
        self.apt = dict()
        self.rwy_thres = dict()
        self.fap = dict()
        self.fap_id_gui = dict()
        self.dep_rwy_id = dict()
        self.arr_rwy_id = dict()
        self.depart_wp = dict()
        self.restrict = dict()

        self.reset()
    

    def reset(self, preset: dict = None):
        """ Reset the TRACON. """
        
        self._active = False
        # Empty the departure runways
        while self.dep_rwy_id:
            apt, rwys = next(iter(self.dep_rwy_id.items()))
            for rwy in rwys:
                self.remove_dep_rwy(apt, rwy)
        self.dep_rwy_id = dict()
        # Empty the arrival runways
        while self.arr_rwy_id:
            apt, rwys = next(iter(self.arr_rwy_id.items()))
            for rwy in rwys:
                self.remove_arr_rwy(apt, rwy)
        self.arr_rwy_id = dict()
        self.fap_id_gui = dict()
        # Empty the departure waypoints
        while self.depart_wp:
            wp = next(iter(self.depart_wp.keys()))
            self.remove_departure_wp(wp)
        self.depart_wp = dict()
        # Empty the restricted areas
        while self.restrict:
            area_id = next(iter(self.restrict.keys()))
            self.remove_restricted(area_id)
        self.restrict = dict()
        # Empty the airport and runway information
        self.apt = dict()
        self.rwy_thres = dict()
        self.fap = dict()

        # Reset to new TRACON
        if preset is None:
            preset = preset_ksfo
        
        self.identifier = preset['identifier']
        self.ctr_lat = preset['ctr_lat']
        self.ctr_lon = preset['ctr_lon']
        self.top = preset['top']
        self.bottom = preset['bottom']
        self.range = preset['range']
        if preset['apt_id'] is not None:
            for apt in preset['apt_id']:
                self.apt[apt] = dict()
        self._obtain_airport_info()

        if self.apt:
            self.elevation = min([self.apt[apt][2] for apt in self.apt])

        if preset['dep_rwy_id'] is not None:
            for dep_rwy in preset['dep_rwy_id']:
                apt, rwy = dep_rwy.upper().split('/')
                self.add_dep_rwy(apt, rwy)

        if preset['arr_rwy_id'] is not None:
            for arr_rwy in preset['arr_rwy_id']:
                apt, rwy = arr_rwy.upper().split('/')
                self.add_arr_rwy(apt, rwy)

        if preset['depart_wp'] is not None:
            for wp in preset['depart_wp']:
                self.add_departure_wp(wp, preset['depart_wp'][wp][0], preset['depart_wp'][wp][1], preset['depart_wp'][wp][2])

        if preset['restrict'] is not None:
            for area in preset['restrict']:
                self.add_restricted(area)

        if preset['activate']:
            self.activate_tracon()


    def is_active(self):
        """ Check if the TRACON is active. """
        return self._active


    def activate(self):
        """ Activate the TRACON if it is qualified. """

        if not self.identifier:
            msg = "Warning: TRACON identifier is not set. TRACON will not be activated."
            return False, msg

        if self.ctr_lat < -90 or self.ctr_lat > 90 or self.ctr_lon < -180 or self.ctr_lon > 180:
            msg = "Warning: TRACON center position is not valid. TRACON will not be activated."
            return False, msg

        if self.range < 10 or self.range > 100:
            msg = "Warning: TRACON range is not valid (needs to be in [10, 100] in NM). TRACON will not be activated."
            return False, msg

        if self.top < 3000 or self.top > 18000:
            msg = "Warning: TRACON top altitude is not valid (needs to be in [3000, 18000] in ft). TRACON will not be activated."
            return False, msg
        
        if self.top - self.elevation < 5000:
            msg = "Warning: Vertical space is not enough (need at least 5000 ft). TRACON will not be activated."
            return False, msg

        if self.bottom < DELETE_BELOW + 100:
            msg = f"Warning: TRACON bottom altitude is less than {DELETE_BELOW + 100}. TRACON will not be activated."
            return False, msg

        if not self.apt:
            msg = "Warning: There is no valid airport. TRACON will not be activated."
            return False, msg
        for apt in self.apt:
            if not self._is_valid_apt(apt):
                msg = f"Warning: Airport {apt} is not valid. TRACON will not be activated."
                return False, msg
            
        if not self.dep_rwy_id:
            msg = "Warning: TRACON does not have any open runways for departure. TRACON will not be activated."
            return False, msg
        if not self.arr_rwy_id:
            msg = "Warning: TRACON does not have any open runways for arrival. TRACON will not be activated."
            return False, msg
        if not self.depart_wp:
            msg = "Warning: TRACON does not have any departure waypoints. TRACON will not be activated."
            return False, msg
            
        self._active = True
        self._gui_draw()
        msg = f"TRACON {self.identifier} is activated."
        return True, msg


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
        if self.is_active():
            msg = "Error: Cannot add restricted area to an active TRACON."
            return False, msg

        # At most 20 restricted areas
        if len(self.restrict) >= 20:
            msg = "Error: The number of restricted areas exceeds the limit (20)."
            return False, msg
        
        # Check if the area ID already exists
        if args[0] in self.restrict:
            msg = f"Error: Restricted area ID {args[0]} already exists."
            return False, msg
        
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
            msg = "Error: Invalid arguments for restricted area."
            return False, msg
        
        # Add the restricted area
        self.restrict[args[0]] = np.array(args[1:], dtype=float)
        # Add the restricted area to simulator. It will be drawn in the GUI automatically.
        if args[1] == 0:
            # Circle
            defineArea(args[0].upper(), "CIRCLE", [args[4], args[5], args[6]], args[3], args[2])
        elif args[1] == 1:
            # Polygon
            defineArea(args[0].upper(), "POLY", args[4:], args[3], args[2])

        return True


    def remove_restricted(self, area_id: str):
        """
        Remove a restricted area from the TRACON.
        
        Args:
            area_id: ID of the restricted area to remove
        """
        # Check if the TRACON is active
        if self.is_active():
            return False

        if area_id in self.restrict:
            if area_id.upper() in basic_shapes:
                # Remove the area from the simulator
                deleteArea(area_id.upper())
            del self.restrict[area_id]
            return True
        else:
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
        else:
            print(f"Error: Departure runway {rwy_id} does not exist or not open for departure.")
            return False

        return True


    def add_arr_rwy(self, apt_id: str, rwy_id: str):
        """
        Add an arrival runway to the TRACON.
        """
        
        # Check if the airport ID and rwy id is valid
        if not self.apt or apt_id not in self.apt or rwy_id not in self.rwy_thres[apt_id]:
            return False
        
        # Check if the runway is open for arrival already
        if apt_id in self.arr_rwy_id and rwy_id in self.arr_rwy_id[apt_id]:
            return True
        
        # Display the FAP on the GUI
        if apt_id not in self.fap_id_gui or rwy_id not in self.fap_id_gui[apt_id]:
            # Create a new FAP ID
            fap_id = f"F{rwy_id}"
            if fap_id in navdb.wpid:
                i = ord('A')
                while fap_id + chr(i) in navdb.wpid:
                    i += 1
                fap_id = fap_id + chr(i)
            # Record the GUI FAP ID
            if apt_id not in self.fap_id_gui:
                self.fap_id_gui[apt_id] = dict()
            self.fap_id_gui[apt_id][rwy_id] = fap_id
            # Add the FAP to the GUI
            navdb.defwpt(fap_id, self.fap[apt_id][rwy_id][0], self.fap[apt_id][rwy_id][1], "FIX")
        
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
        else:
            print(f"Error: Arrival runway {rwy_id} does not exist.")
            return False
        
        # Remove the FAP from the GUI
        if apt_id in self.fap_id_gui and rwy_id in self.fap_id_gui[apt_id]:
            fap_id = self.fap_id_gui[apt_id][rwy_id]
            # Remove the FAP from the GUI
            navdb.delwpt(fap_id)
            del self.fap_id_gui[apt_id][rwy_id]
            if len(self.fap_id_gui[apt_id]) == 0:
                del self.fap_id_gui[apt_id]

        return True


    def add_departure_wp(self, wp_id: str, lat: float, lon: float, alt: float):
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
        dist_from_ctr = qdrdist(self.ctr_lat, self.ctr_lon, lat, lon)[1]
        if dist_from_ctr > self.range + 15.0:
            print(f"Error: Departure waypoint {wp_id} is out of range.")
            return False
        if dist_from_ctr < self.range:
            print(f"Error: Departure waypoint {wp_id} is too close to the center.")
            return False
        
        # Add the departure waypoint
        self.depart_wp[wp_id] = np.array([lat, lon, alt], dtype=float)

        # Register the waypoint to the simulator
        navdb.defwpt(wp_id, lat, lon, "FIX")
        # Debug message
        # print(f"{wp_id} is added to the departure waypoints.")

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
        else:
            print(f"Error: Departure waypoint {wp_id} does not exist.")
            return False
        
        # Remove the waypoint from the GUI
        navdb.delwpt(wp_id)

        return True


    def get_vertical_envelope(self, bearing):
        """
        Get the vertical restrictions from the center with a given bearing.
        When generating random arriving traffic, the vertical envelope is used to determine
         the altitude of the aircraft based on its bearing from the TRACON center.
        Using straight line segments instead of geodesics for simplicity.
        return: min_alt, max_alt (ft)
        """

        min_alt = float('inf')
        max_alt = float('-inf')

        # Ray endpoint
        end_lat, end_lon = qdrpos(self.ctr_lat, self.ctr_lon, bearing, self.range + 10)
        # Ray
        ray = np.array([[self.ctr_lat, self.ctr_lon], [end_lat, end_lon]])
        ray_vec = np.array([end_lat - self.ctr_lat, end_lon - self.ctr_lon])
        ray_norm = np.linalg.norm(ray_vec)
        ray_path = Path(ray)

        for area_id in self.restrict:
            if self.restrict[area_id][0] == 0:
                # circle
                center_lat, center_lon, radius = self.restrict[area_id][3:6]
                # calculate the projection vector
                t = np.dot(ray_vec, np.array([center_lat - self.ctr_lat, center_lon - self.ctr_lon])) / (ray_norm ** 2)
                t_clamped = np.clip(t, 0, 1)
                # calculate closest point
                closest_point = np.array([self.ctr_lat, self.ctr_lon]) + t_clamped * ray_vec
                distance = np.linalg.norm(closest_point - np.array([center_lat, center_lon]))
                if distance < radius:
                    # intersecting
                    min_alt = min(min_alt, self.restrict[area_id][1])
                    max_alt = max(max_alt, self.restrict[area_id][2])
            elif self.restrict[area_id][0] == 1:
                # polygon
                shape_path = Path(self.restrict[area_id][3:].reshape(-1, 2))
                # use matplotlib to check if the ray intersects with the polygon
                if shape_path.intersects_path(ray_path):
                    min_alt = min(min_alt, self.restrict[area_id][1])
                    max_alt = max(max_alt, self.restrict[area_id][2])

        return min_alt, max_alt


    def _gui_draw(self):
        """
        Draw the TRACON area on the GUI.
        """
        
        # centralize
        zoom = 1.0 * 30 / self.range
        stack.stack(f"PAN {self.ctr_lat}, {self.ctr_lon}; ZOOM {zoom}")

        # Draw the TRACON area
        stack.stack(f"CIRCLE {self.identifier}, {self.ctr_lat}, {self.ctr_lon}, {self.range}")
        stack.stack(f"COL {self.identifier} GREEN")

        # Turn on aircraft symbol
        stack.stack("SYMBOL")


    def _gui_erase(self):
        if bs.mode == 'sim':
            return
        
        # Erase the TRACON area
        if self.identifier in basic_shapes:
            deleteArea(self.identifier.upper())
        

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
            self.apt[apt] = np.array([navdb.aptlat[idx], navdb.aptlon[idx], navdb.aptelev[idx]], dtype=float)
        
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
                self.fap[apt][rwy] = np.array([fap_lat, fap_lon, self.apt[apt][2] + FAP_ELEVATION], dtype=float)

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

        dist_from_ctr = qdrdist(self.ctr_lat, self.ctr_lon, self.apt[apt_id][0], self.apt[apt_id][1])[1]
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



preset_ksfo = {
    "identifier": "KSFO_DEFAULT",
    "ctr_lat": 37.6189,
    "ctr_lon": -122.3754,
    "range": 40.0,
    "top": 11000,
    "bottom": 1000,
    "apt_id": ["KSFO", "KSJC", "KOAK"],
    "dep_rwy_id": ["KSFO/28L", "KSFO/28R", "KSFO/01L", "KSFO/01R", "KSJC/30R", "KSJC/30L", "KOAK/29", "KOAK/27L"],
    "arr_rwy_id": ["KSFO/28L", "KSFO/28R", "KSJC/30R", "KSJC/30L", "KOAK/29", "KOAK/27R"],
    "depart_wp": {
        "VDEAST": [37.6152, -121.42869, 10000],
        "VDSOUTH": [36.8694, -122.375, 10000],
        "VDWEST": [37.6152, -123.32131, 10000],
        "VDNORTH": [38.3686, -122.375, 10000]
    },
    "restrict": [
        ["SFDOWNTOWN", 0, 0, 1500, 37.7559, -122.41025, 2],
        ["MOUNTAINS", 1, 0, 4500, 37.4596, -122.34264, 37.4220, -122.35569, 37.3616, -122.29408, 37.3811, -122.25142]
    ],
    "activate": False
}



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

