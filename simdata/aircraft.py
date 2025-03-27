



class Aircraft():
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
                 trk_to_wpt=None):
        
        # Identifier of the aircraft
        self.callsign = callsign
        # Aircraft type, e.g. B38M, A20N
        self.acf_type = acf_type
        # Weight class: "light", "medium", "heavy" or "A380"
        self.acf_class = acf_class
        # Intension of the aircraft: "departure" or "arrival"
        self.intention = intention
        # Target waypoint (rwy FAP for arrival or departur wp) latitude
        self.target_wp_lat = target_wp_lat
        # Target waypoint longitude
        self.target_wp_lon = target_wp_lon
        # Target waypoint altitude
        self.target_wp_alt = target_wp_alt
        # Whether the aircraft is under controll of the TRACON
        self.under_ctrl = under_ctrl
        # Time (in sec) entering the TRACON (under controll)
        self.time_entered = time_entered
        # Time (in sec) stayed in the TRACON (under controll)
        self.time_stayed = time_stayed
        # Time of last transmitted command
        self.time_last_command = time_last_command
        # How long since last transmitted command
        self.time_since_last_command = time_since_last_command
        # Whether the LNAV mode is on (boolean)
        self.lnav_on = lnav_on
        # Whether the VNAV mode is on (boolean)
        self.vnav_on = vnav_on
        # Whether the VNAV SPD mode is on
        self.vnav_spd_on = vnav_spd_on
        # Current latitude
        self.lat = lat
        # Current longitude
        self.lon = lon
        # Current altitude
        self.curr_alt = curr_alt
        # Instructed altitude
        self.target_alt = target_alt if target_alt is not None else curr_alt
        # Current vertical speed
        self.curr_vs = curr_vs
        # Instructed vertical speed
        self.target_vs = target_vs if target_vs is not None else curr_vs
        # Current airspeed
        self.curr_cas = curr_cas
        # Instructed airspeed
        self.target_cas = target_cas if target_cas is not None else curr_cas
        # Current track (true heading)
        self.curr_trk = curr_trk
        # Instructed track (true heading)
        self.target_trk = target_trk if target_trk is not None else curr_trk
        # Distance to target waypoint (for departure acf) or target FAP (for arrival acf)
        self.dist_to_wpt = dist_to_wpt
        # Track (initial bearing) from current location to target
        self.trk_to_wpt = trk_to_wpt


    def update_dist_to_wpt(self):
        pass


    def update_trk_to_wpt(self):
        pass


    def update_time_stayed(self, curr_time):
        # Called every second
        if self.time_entered is not None:
            self.time_stayed = curr_time - self.time_entered


    def update_time_since_last_command(self, curr_time):
        # Called every second
        if self.time_last_command is not None:
            self.time_since_last_command = curr_time - self.time_last_command
        self.time_last_command = curr_time

