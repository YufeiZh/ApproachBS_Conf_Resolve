''' Load and save scenarios '''

from config import OUTPUT_PATH
from simdata import aircraft, tracon
import utils.geocalc as geo
from resources.acftype import AIRCRAFTCLASS
import random
import numpy as np
from PIL import Image, ImageDraw


class Scenario():
    def __init__(self, tracon, aircrafts=None, simt=0):
        self.tracon = tracon
        self.aircrafts = aircrafts if aircrafts else []
        # Simulator time in seconds. Default start time is 0.
        self.simt = simt
        self.num_apts = 0
        self.num_runways = 0
        self.num_restrict = 0
        self.num_acfs = 0
        self.update_num_info()
        # LoS/invasion matrix
        # Entry (i, j) is binary, denoting if there is currently a LoS or invasion
        # i is between 21 and 70, denotes an aircraft
        # j is between 0 and 70, denotes either a restricted area or an aircraft
        # (i, j) entry equals 1 if and only if aircraft i is invading area j or loss separation
        #    with aircraft j within 1 minute
        self.conflict = np.zeros(shape=(50,71))
        # Conflict happening in 1 minute
        self.conflict_in_one_min = np.zeros(shape=(50,71))


    def update_num_info(self):
        if self.tracon is not None:
            self.num_apts = len(self.tracon.apt_id)
            self.num_runways = len(self.tracon.runway_id)
            self.num_restrict = len(self.tracon.restrict)
        else:
            self.num_apts = self.num_runways = self.num_restrict = 0
        if self.aircrafts is not None:
            self.num_acfs = len(self.aircrafts)
        else:
            self.num_acfs = 0


    def to_dict(self):
        dict_data = dict()
        dict_data["tracon"] = self.tracon.to_dict()

        acf_dict_list = []
        for ele in self.aircrafts:
            acf_dict_list.append(ele.to_dict())
        dict_data["aircrafts"] = acf_dict_list

        dict_data["simt"] = self.simt
        dict_data["num_apts"] = self.num_apts
        dict_data["num_runways"] = self.num_runways
        dict_data["num_restrict"] = self.num_restrict
        dict_data["num_acfs"] = self.num_acfs


    def get_acf_info(self):
        acf_info_lst = []
        for acf in self.aircrafts:
            # Get the distance from the center of the tracon
            dist_from_ctr = geo.distance_between(acf.lat, acf.lon, self.tracon.ctr_lat, self.tracon.ctr_lon)
            dist_from_ctr = round(dist_from_ctr, 1)
            # Get the direction of the airctraft from the center of the tracon
            bearing_from_ctr = geo.bearing_from_to(self.tracon.ctr_lat, self.tracon.ctr_lon, acf.lat, acf.lon)
            sector = (bearing_from_ctr + 22.5) // 45
            match sector:
                case 0:
                    direction = "N "
                case 1:
                    direction = "NE"
                case 2:
                    direction = "E "
                case 3:
                    direction = "SE"
                case 4:
                    direction = "S "
                case 5:
                    direction = "SW"
                case 6:
                    direction = "W "
                case 7:
                    direction = "NW"
                case _:
                    direction = "DK"
            
            acf_info = f"{acf.callsign}\t{dist_from_ctr}NM\t" + direction + f" TRK {acf.curr_trk:03} {acf.curr_cas:03}KT at {acf.curr_alt}FT"
            acf_info_lst.append(acf_info)
        
        return acf_info_lst
 

    def show_acf_info(self):
        info = self.get_acf_info()
        for item in info:
            print(item)


    @classmethod
    def from_dict(cls, dict_data):
        instance = cls()
        try:
            instance.tracon = tracon.Tracon.from_dict(dict_data["tracon"])
        except:
            print("Illegal TRACON data.")
        
        acf_list = []
        try:
            for ele in dict_data["aircrafts"]:
                acf_list.append(aircraft.Aircraft.from_dict(ele))
            instance.aircrafts = acf_list
        except:
            print("Illegal aircraft data.")

        instance.simt = dict_data["simt"]
        instance.num_apts = dict_data["num_apts"]
        instance.num_runways = dict_data["num_runways"]
        instance.num_restrict = dict_data["num_restrict"]
        instance.num_acfs = dict_data["num_acfs"]

        return instance


    def load_scenario(self, scenario_name=None):
        print("This function 'Scenario.load_scenario' has not been implemented yet.")


    def update_conflict(self):
        self.update_num_info()

        for i in range(self.num_acfs):
            curr_acf = self.aircrafts[i]

            # Check if acf[i] is out of TRACON:
            #   0 - in TRACON
            #   1 - out of TRACON
            if curr_acf.curr_alt < self.tracon.min_alt \
            or curr_acf.curr_alt > self.tracon.max_alt \
            or geo.distance_between(curr_acf.lat, curr_acf.lon, self.tracon.ctr_lat, self.tracon.ctr_lon) > self.tracon.range:
                self.conflict[i,0] = 1
            else:
                self.conflict[i,0] = 0

            # Check if acf[i] is in restricted areas
            for j in range(self.num_restrict):
                if self.tracon.restrict[j].in_area(curr_acf.lat, curr_acf.lon, curr_acf.curr_alt):
                    self.conflict[i,j+1] = 1
                else:
                    self.conflict[i,j+1] = 0

            # Check if acf[i] is in LoS(Loss of Separation) with acf[j]
            # Two aircrafts are in LoS if their horizontal distance < 5NM and vertical distance < 950ft
            for j in range(i+1, self.num_acfs):
                if geo.distance_between(curr_acf.lat, curr_acf.lon, self.aircrafts[j].lat, self.aircrafts[j].lon) < 5 \
                and np.abs(curr_acf.curr_alt - self.aircrafts[j].curr_alt) < 950:
                    self.conflict[i,j+21] = 1
                    self.conflict[j,i+21] = 1
                else:
                    self.conflict[i,j+21] = 0
                    self.conflict[j,i+21] = 0

            # Set to zero for each entry corresponding to an empty area or an empty aircraft
            self.conflict[i, self.num_restrict+1:21] = 0
            self.conflict[i, self.num_acfs+21:] = 0
            

    def delete_out_range_acf(self):
        ''' Delete aircrafts that are:
         - more than tracon.range + 10NM away from the tracon center, or
         - higher than tracon.max_alt + 2000ft, or
         - lower than tracon.min_alt - 1500ft. '''

        if self.aircrafts == None:
            return

        # If the tracon is not defined, delete all aircrafts
        if self.tracon == None:
            self.aircrafts = None
            return
        
        i = 0
        while i < self.num_acfs:
            dist = geo.distance_between(self.tracon.ctr_lat, self.tracon.ctr_lon, self.aircrafts[i].lat, self.aircrafts[i].lon)
            if dist > self.tracon.range + 10 \
             or self.aircrafts[i].curr_alt > self.tracon.max_alt + 2000 \
             or self.aircrafts[i].curr_alt < self.tracon.min_alt - 1500:
                self.aircrafts.pop(i)
                self.num_acfs -= 1
                continue
            i += 1


    def add_acf(self, acf):
        # Do not add the aircraft if there are already 50 aircrafts in scenario
        if self.num_acfs >= 50:
            return
        
        # Do not add the aircraft if the callsign already exists.
        exist_id = []
        for ele in self.aircrafts:
            exist_id.append(ele.callsign)
        if acf.callsign in exist_id:
            print("Callsign already exists.")
            return

        dist = geo.distance_between(self.tracon.ctr_lat, self.tracon.ctr_lon, acf.lat, acf.lon)

        # Do not add the aircraft if it is out of range
        if dist > self.tracon.range + 10.1:
            print(f"Aircract is {dist}NM from the center, greater than {self.tracon.range+10}NM.")
            return

        if acf.curr_alt > self.tracon.max_alt + 5000:
            print(f"Aircract is {acf.curr_alt}ft, greater than {self.tracon.max_alt+5000}ft.")
            return

        if acf.curr_alt < 0:
            print(f"Aircract is {acf.curr_alt}ft, less than 0ft.")
            return
        
        self.num_acfs += 1
        
        # Set acf.under_control as True if the aircraft is in TRACON area.
        if dist > self.tracon.range \
         or acf.curr_alt > self.tracon.max_alt \
         or acf.curr_alt < self.tracon.min_alt:
            acf.under_ctrl = False
        else:
            acf.under_ctrl = True
            acf.time_entered = self.simt

        self.aircrafts.append(acf)


    def add_random_acf(self, 
                       acf_id=None,
                       acf_type=None,
                       acf_class=None, 
                       avoid_restrict=False, 
                       trkchange=False, 
                       altchange=False, 
                       spdchange=False,
                       unctrl_area=True):
        ''' Add a random new aircraft to the scenario. '''
        
        # Do not add the aircraft if there are already 50 aircrafts in scenario
        if self.num_acfs >= 50:
            return
        

        # Add callsign
        exist_id = []
        for ele in self.aircrafts:
            exist_id.append(ele.callsign)
        # Do not add the aircraft if the callsign already exists.
        if acf_id and acf_id in exist_id:
            print("Callsign already exists.")
            return
        
        if acf_id is None:
            # Generate a callsign
            new_id = "TEST"
            for i in range(1, 51):
                if new_id + str(i) not in exist_id:
                    acf_id = new_id + str(i)
                    break


        # Randomly generate latitude, longitude and altitude
        counter = 0
        while True:
            counter += 1
            if counter > 50:
                print("Unable to generate aircraft.")
                return
            bearing_from_ctr = round(random.uniform(0, 360), 1)
            if unctrl_area:
                dist_from_ctr = round(random.uniform(0, self.tracon.range + 10), 1)
            else:
                dist_from_ctr = round(random.uniform(0, self.tracon.range), 1)
            lat, lon = geo.destination_by_bearing(self.tracon.ctr_lat, self.tracon.ctr_lon, bearing_from_ctr, dist_from_ctr)
            lat, lon = round(lat, 7), round(lon, 7)
            alt = random.randint(max(self.tracon.min_alt - 1500, 0), self.tracon.max_alt + 3000)

            # If we want to avoid restricted area and the aircraft is in an restricted
            #     area, then discard the lat, lon and alt and generate new location
            if not avoid_restrict or len(self.tracon.restrict) == 0:
                break
            else:
                in_restrict = False
                for area in self.tracon.restrict:
                    if area.in_area(lat, lon, alt):
                        in_restrict = True
                        break
                if not in_restrict:
                    break

        
        # Randomely generate class and type if unspecified
        if acf_class is None:
            if acf_type is not None:
                # If acf type is specified but class is not, check if the acf type is in database
                for ele in AIRCRAFTCLASS:
                    if acf_type in AIRCRAFTCLASS[ele]:
                        acf_class = ele
                        break
                if acf_class is None:
                    # If not in database, randomly assume the aircraft class
                    acf_class = random.choices(["light", "medium", "heavy"], weights=[0.5, 5.0, 4.5])[0]
            else:
                # If acf type and class are not specified, randomly choose them
                acf_class = random.choices(["light", "medium", "heavy", "A380"], weights=[0.4, 5.0, 4.5, 0.1])[0]
                acf_type = random.choice(AIRCRAFTCLASS[acf_class])
        # If acf class is specified but type is not, randomly choose from the class
        if acf_type is None:
            acf_type = random.choice(AIRCRAFTCLASS[acf_class])

        # Randomly generate current CAS and TRK
        cas = random.randint(160, 250)
        trk = random.randint(0, 359)

        # Generate target altitude and vertical speed if altchange is True
        #     meaning that the acf is changing level
        if altchange:
            # Randomly decide whether the acf is climbing or descending
            climb = random.getrandbits(1)
            if climb == 1:
                # Climb at least 1000 ft
                target_alt = random.randint(alt+1000, self.tracon.max_alt+5000)
                # Decide if the current vs is 0, the target vs(+1500fpm), or a random fpm between 0 and +3000
                temp_vs = random.randint(0, 3000)
                vs = random.choice([0, temp_vs, 1500])
                # The default target_vs is 1500 fpm
                target_vs = 1500
            else:
                # Do not descend if the aircraft is too low
                if alt <= 1000:
                    target_alt = alt
                    vs = 0
                    target_vs = 0
                else:
                    # Descend at least 1000 ft
                    target_alt = random.randint(0, alt-1000)
                    # Decide if the current vs is 0, the target vs(-1500fpm), or a random fpm between -3000 and 0
                    temp_vs = random.randint(-3000,0)
                    vs = random.choice([0, temp_vs, -1500])
                    # The default target_vs is -1500 fpm
                    target_vs = -1500
        else:
            # If not changing level, set vs, target_vs and target_alt
            vs = 0
            target_vs = 0
            target_alt = alt


        # Generate target trk
        if trkchange:
            # Turning range: left 120 to right 120 degrees
            delta_trk = random.randint(-120, 120)
            target_trk = (trk + delta_trk) % 360
            # Decide if the current bank angle is 0, the target angle(25 degrees), or a random degree between 0 and 25
        else:
            target_trk = trk


        # Generate target CAS if spdchange is True
        if spdchange:
            target_cas = random.randint(160, 250)
        else:
            target_cas = cas


        # Create new aircraft
        new_acf = aircraft.Aircraft(callsign=acf_id, 
                                    lat=lat, 
                                    lon=lon, 
                                    curr_alt=alt, 
                                    curr_cas=cas, 
                                    curr_trk=trk, 
                                    curr_vs=vs, 
                                    acf_type=acf_type, 
                                    acf_class=acf_class,
                                    target_alt=target_alt,
                                    target_vs=target_vs,
                                    target_cas=target_cas,
                                    target_trk=target_trk,
                                    )
        
        self.add_acf(new_acf)


    def add_LoS_acf(self,
                     existing_acf_id,
                     acf_type=None,
                     acf_id=None,
                     acf_class=None, 
                     avoid_restrict=False, 
                     trkchange=False, 
                     altchange=False, 
                     spdchange=False,
                     unctrl_area=False):
        ''' Add a random aircraft that is possibly in conflict with an existing aircraft. '''
        
        # Do not add the aircraft if there are already 50 aircrafts in scenario
        if self.num_acfs >= 50:
            return
        
        # If there's no existing aircraft that matches existing_acf_id, just generate random new aircraft
        exist_id = []
        for ele in self.aircrafts:
            exist_id.append(ele.callsign)
        if existing_acf_id not in exist_id:
            self.add_random_acf(acf_id=acf_id, 
                                acf_type=acf_type, 
                                acf_class=acf_class, 
                                avoid_restrict=avoid_restrict,
                                trkchange=trkchange,
                                altchange=altchange,
                                spdchange=spdchange,
                                unctrl_area=unctrl_area)
            print("Target aircraft does not exist, generating random aircraft.")
            return
        else:
            target_acf = self.aircrafts[exist_id.index(existing_acf_id)]

        # Do not add the aircraft if the callsign already exists.
        if acf_id and acf_id in exist_id:
            print("Callsign already exists.")
            return
        
        # Generate callsign if not assigned already
        if acf_id is None:
            # Generate a callsign
            new_id = "TEST"
            for i in range(1, 51):
                if new_id + str(i) not in exist_id:
                    acf_id = new_id + str(i)
                    break


        # Generate latitude, longitude and altitude
        counter = 0
        while True:
            counter += 1
            if counter > 50:
                print("Unable to generate aircraft.")
                return
            bearing_from_existing_acf = round(random.uniform(0, 360), 1)
            # The horizontal distance from the new aircraft to the existing aircraft is between 0 and 8NM
            dist_from_existing_acf = round(random.uniform(0, 8), 2)
            # The vertical distance from the new aircraft to the existing aircraft is between -3000 to 3000
            # The altitude of the new aircraft needs to be at least 500ft
            vert_dist_from_existing_acf = -20000
            while target_acf.curr_alt + vert_dist_from_existing_acf < 500:
                vert_dist_from_existing_acf = random.randint(-1000,1000)
            new_lat, new_lon = geo.destination_by_bearing(target_acf.lat, target_acf.lon, bearing_from_existing_acf, dist_from_existing_acf)
            new_lat = round(new_lat, 7)
            new_lon = round(new_lon, 7)
            new_alt = target_acf.curr_alt + vert_dist_from_existing_acf

            # Check if the generation is valid, i.e. satisfying:
            #    - Not out of range (and not in uncontrolled area if unctrl_area==False)
            #    - Not in restricted area if avoid_restricted==True
            # If invalid, discard and regenerate.
            dist_from_ctr = geo.distance_between(new_lat, new_lon, self.tracon.ctr_lat, self.tracon.ctr_lon)
            if dist_from_ctr > self.tracon.range + 10:
                continue
            if dist_from_ctr > self.tracon.range and not unctrl_area:
                continue
            if new_alt < 0 or new_alt > self.tracon.max_alt+3000:
                continue
            if avoid_restrict:
                in_restrict = False
                for area in self.tracon.restrict:
                    if area.in_area(new_lat, new_lon, new_alt):
                        in_restrict = True
                        break
                if in_restrict:
                    continue
            break

        # Randomely generate class and type if unspecified
        if acf_class is None:
            if acf_type is not None:
                # If acf type is specified but class is not, check if the acf type is in database
                for ele in AIRCRAFTCLASS:
                    if acf_type in AIRCRAFTCLASS[ele]:
                        acf_class = ele
                        break
                if acf_class is None:
                    # If not in database, randomly assume the aircraft class (cannot be 'A380')
                    acf_class = random.choices(["light", "medium", "heavy"], weights=[0.5, 5.0, 4.5])[0]
            else:
                # If acf type and class are not specified, randomly choose them
                acf_class = random.choices(["light", "medium", "heavy", "A380"], weights=[0.4, 5.0, 4.5, 0.1])[0]
                acf_type = random.choice(AIRCRAFTCLASS[acf_class])
        # If acf class is specified but type is not, randomly choose from the class
        if acf_type is None:
            acf_type = random.choice(AIRCRAFTCLASS[acf_class])

        # Randomly generate current CAS and TRK
        cas = random.randint(160, 250)
        trk = random.randint(0, 359)

        # Generate target altitude and vertical speed if altchange is True
        #     meaning that the acf is changing level
        if altchange:
            # Randomly decide whether the acf is climbing or descending
            climb = random.randint(0,1)
            if climb == 1:
                # Climb at least 1000 ft
                target_alt = random.randint(new_alt+1000, self.tracon.max_alt+5000)
                # Decide if the current vs is 0, the target vs(+1500fpm), or a random fpm between 0 and +3000
                temp_vs = random.randint(0, 3000)
                vs = random.choice([0, temp_vs, 1500])
                # The default target_vs is 1500 fpm
                target_vs = 1500
            else:
                # Do not descend if the aircraft is too low
                if new_alt <= 1000:
                    target_alt = new_alt
                    vs = 0
                    target_vs = 0
                else:
                    # Descend at least 1000 ft
                    target_alt = random.randint(0, new_alt-1000)
                    # Decide if the current vs is 0, the target vs(-1500fpm), or a random fpm between -3000 and 0
                    temp_vs = random.randint(-3000,0)
                    vs = random.choice([0, temp_vs, -1500])
                    # The default target_vs is -1500 fpm
                    target_vs = -1500
        else:
            # If not changing level, set vs, target_vs and target_alt
            vs = 0
            target_vs = 0
            target_alt = new_alt


        # Generate target trk
        if trkchange:
            # Turning range: left 120 to right 120 degrees
            delta_trk = random.randint(-120, 120)
            target_trk = (trk + delta_trk) % 360
        else:
            target_trk = trk


        # Generate target CAS if spdchange is True
        if spdchange:
            target_cas = random.randint(160, 250)
        else:
            target_cas = cas


        # Create new aircraft
        new_acf = aircraft.Aircraft(callsign=acf_id, 
                                    lat=new_lat, 
                                    lon=new_lon, 
                                    curr_alt=new_alt, 
                                    curr_cas=cas, 
                                    curr_trk=trk, 
                                    curr_vs=vs, 
                                    acf_type=acf_type, 
                                    acf_class=acf_class,
                                    target_alt=target_alt,
                                    target_vs=target_vs,
                                    target_cas=target_cas,
                                    target_trk=target_trk,
                                    )
        
        self.add_acf(new_acf)

    
    def add_close_acf(self,
                     existing_acf_id,
                     acf_type=None,
                     acf_id=None,
                     acf_class=None, 
                     avoid_restrict=False, 
                     trkchange=False, 
                     altchange=False, 
                     spdchange=False,
                     unctrl_area=False):
        ''' Add a random aircraft that is possibly in conflict with an existing aircraft. '''
        
        # Do not add the aircraft if there are already 50 aircrafts in scenario
        if self.num_acfs >= 50:
            return
        
        # If there's no existing aircraft that matches existing_acf_id, just generate random new aircraft
        exist_id = []
        for ele in self.aircrafts:
            exist_id.append(ele.callsign)
        if existing_acf_id not in exist_id:
            self.add_random_acf(acf_id=acf_id, 
                                acf_type=acf_type, 
                                acf_class=acf_class, 
                                avoid_restrict=avoid_restrict,
                                trkchange=trkchange,
                                altchange=altchange,
                                spdchange=spdchange,
                                unctrl_area=unctrl_area)
            print("Target aircraft does not exist, generating random aircraft.")
            return
        else:
            target_acf = self.aircrafts[exist_id.index(existing_acf_id)]

        # Do not add the aircraft if the callsign already exists.
        if acf_id and acf_id in exist_id:
            print("Callsign already exists.")
            return
        
        # Generate callsign if not assigned already
        if acf_id is None:
            # Generate a callsign
            new_id = "TEST"
            for i in range(1, 51):
                if new_id + str(i) not in exist_id:
                    acf_id = new_id + str(i)
                    break


        # Generate latitude, longitude and altitude
        counter = 0
        while True:
            counter += 1
            if counter > 50:
                print("Unable to generate aircraft.")
                return
            bearing_from_existing_acf = round(random.uniform(0, 360), 1)
            # The horizontal distance from the new aircraft to the existing aircraft is between 0 and 8NM
            dist_from_existing_acf = round(random.uniform(0, 10), 2)
            # The vertical distance from the new aircraft to the existing aircraft is between -3000 to 3000
            # The altitude of the new aircraft needs to be at least 500ft
            vert_dist_from_existing_acf = -20000
            while target_acf.curr_alt + vert_dist_from_existing_acf < 500:
                vert_dist_from_existing_acf = random.randint(-3000,3000)
            new_lat, new_lon = geo.destination_by_bearing(target_acf.lat, target_acf.lon, bearing_from_existing_acf, dist_from_existing_acf)
            new_lat = round(new_lat, 7)
            new_lon = round(new_lon, 7)
            new_alt = target_acf.curr_alt + vert_dist_from_existing_acf

            # Check if the generation is valid, i.e. satisfying:
            #    - Not out of range (and not in uncontrolled area if unctrl_area==False)
            #    - Not in restricted area if avoid_restricted==True
            # If invalid, discard and regenerate.
            dist_from_ctr = geo.distance_between(new_lat, new_lon, self.tracon.ctr_lat, self.tracon.ctr_lon)
            if dist_from_ctr > self.tracon.range + 10:
                continue
            if dist_from_ctr > self.tracon.range and not unctrl_area:
                continue
            if new_alt < 0 or new_alt > self.tracon.max_alt+3000:
                continue
            if avoid_restrict:
                in_restrict = False
                for area in self.tracon.restrict:
                    if area.in_area(new_lat, new_lon, new_alt):
                        in_restrict = True
                        break
                if in_restrict:
                    continue
            break

        # Randomely generate class and type if unspecified
        if acf_class is None:
            if acf_type is not None:
                # If acf type is specified but class is not, check if the acf type is in database
                for ele in AIRCRAFTCLASS:
                    if acf_type in AIRCRAFTCLASS[ele]:
                        acf_class = ele
                        break
                if acf_class is None:
                    # If not in database, randomly assume the aircraft class (cannot be 'A380')
                    acf_class = random.choices(["light", "medium", "heavy"], weights=[0.5, 5.0, 4.5])[0]
            else:
                # If acf type and class are not specified, randomly choose them
                acf_class = random.choices(["light", "medium", "heavy", "A380"], weights=[0.4, 5.0, 4.5, 0.1])[0]
                acf_type = random.choice(AIRCRAFTCLASS[acf_class])
        # If acf class is specified but type is not, randomly choose from the class
        if acf_type is None:
            acf_type = random.choice(AIRCRAFTCLASS[acf_class])

        # Randomly generate current CAS and TRK
        cas = random.randint(160, 250)
        trk = random.randint(0, 359)

        # Generate target altitude and vertical speed if altchange is True
        #     meaning that the acf is changing level
        if altchange:
            # Randomly decide whether the acf is climbing or descending
            climb = random.randint(0,1)
            if climb == 1:
                # Climb at least 1000 ft
                target_alt = random.randint(new_alt+1000, self.tracon.max_alt+5000)
                # Decide if the current vs is 0, the target vs(+1500fpm), or a random fpm between 0 and +3000
                temp_vs = random.randint(0, 3000)
                vs = random.choice([0, temp_vs, 1500])
                # The default target_vs is 1500 fpm
                target_vs = 1500
            else:
                # Do not descend if the aircraft is too low
                if new_alt <= 1000:
                    target_alt = new_alt
                    vs = 0
                    target_vs = 0
                else:
                    # Descend at least 1000 ft
                    target_alt = random.randint(0, new_alt-1000)
                    # Decide if the current vs is 0, the target vs(-1500fpm), or a random fpm between -3000 and 0
                    temp_vs = random.randint(-3000,0)
                    vs = random.choice([0, temp_vs, -1500])
                    # The default target_vs is -1500 fpm
                    target_vs = -1500
        else:
            # If not changing level, set vs, target_vs and target_alt
            vs = 0
            target_vs = 0
            target_alt = new_alt


        # Generate target trk
        if trkchange:
            # Turning range: left 120 to right 120 degrees
            delta_trk = random.randint(-120, 120)
            target_trk = (trk + delta_trk) % 360
        else:
            target_trk = trk


        # Generate target CAS if spdchange is True
        if spdchange:
            target_cas = random.randint(160, 250)
        else:
            target_cas = cas


        # Create new aircraft
        new_acf = aircraft.Aircraft(callsign=acf_id, 
                                    lat=new_lat, 
                                    lon=new_lon, 
                                    curr_alt=new_alt, 
                                    curr_cas=cas, 
                                    curr_trk=trk, 
                                    curr_vs=vs, 
                                    acf_type=acf_type, 
                                    acf_class=acf_class,
                                    target_alt=target_alt,
                                    target_vs=target_vs,
                                    target_cas=target_cas,
                                    target_trk=target_trk,
                                    )
        
        self.add_acf(new_acf)


    def display(self):
        img = self.tracon.display_tracon()
        draw = ImageDraw.Draw(img)

        # Draw aircrafts with wake turbulence regions
        for acf in self.aircrafts:
            bearing_from_ctr = geo.bearing_from_to(self.tracon.ctr_lat, self.tracon.ctr_lon, acf.lat, acf.lon)
            dist_from_ctr = geo.distance_between(self.tracon.ctr_lat, self.tracon.ctr_lon, acf.lat, acf.lon)
            cx, cy = polar_to_xOy(bearing_from_ctr, dist_from_ctr, self.tracon.range)
            left_up_point = (cx-8, cy-8)
            right_down_point = (cx+8, cy+8)
            # Draw a circle and arrow to represent the aircraft
            draw.ellipse([left_up_point, right_down_point], outline='black')
            ar1x = cx + 8 * np.cos(np.radians(-acf.curr_trk-30))
            ar1y = cy - 8 * np.sin(np.radians(-acf.curr_trk-30))
            start_point = (cx, cy)
            end_point = (ar1x, ar1y)
            draw.line([start_point, end_point], fill='black', width=3)
            ar2x = cx + 8 * np.cos(np.radians(-acf.curr_trk+210))
            ar2y = cy - 8 * np.sin(np.radians(-acf.curr_trk+210))
            start_point = (cx, cy)
            end_point = (ar2x, ar2y)
            draw.line([start_point, end_point], fill='black', width=3)
            # Draw a line segment to denote the current track
            trkx = cx + 8 * np.cos(np.radians(-acf.curr_trk+90))
            trky = cy - 8 * np.sin(np.radians(-acf.curr_trk+90))
            start_point = (cx, cy)
            end_point = (trkx, trky)
            draw.line([start_point, end_point], fill='black', width=3)
            # Draw a line segment in green to denote the wake turbulence
            wt_range = 2.5
            wtx = cx - wt_range * 400 / self.tracon.range * np.cos(np.radians(-acf.curr_trk+90))
            wty = cy + wt_range * 400 / self.tracon.range * np.sin(np.radians(-acf.curr_trk+90))
            start_point = (cx, cy)
            end_point = (wtx, wty)
            draw.line([start_point, end_point], fill='green', width=2)
            # Add label for the aircraft
            draw.text((cx+5, cy-15), acf.callsign, fill='black')

        return img


    def to_tokens(self):

        # Part1: the tracon info (token0)
        # TRACON area: center lat, center lon, range, min_alt, max_alt, number of aircrafts
        tracon_token = np.zeros(shape=(21))
        tracon_token[0] = self.tracon.ctr_lat
        tracon_token[1] = self.tracon.ctr_lon
        tracon_token[2] = self.tracon.range
        tracon_token[3] = self.tracon.min_alt
        tracon_token[4] = self.tracon.max_alt
        tracon_token[5] = self.num_acfs
        # Airport info: bearing from center, dist from center, elevation
        for i in range(self.num_apts):
            if i > 4:
                break
            bfc = geo.bearing_from_to(self.tracon.ctr_lat, self.tracon.ctr_lon, self.tracon.apt_lat[i], self.tracon.apt_lon[i])
            dfc = geo.distance_between(self.tracon.ctr_lat, self.tracon.ctr_lon, self.tracon.apt_lat[i], self.tracon.apt_lon[i])
            tracon_token[6+i*3] = bfc
            tracon_token[7+i*3] = dfc
            tracon_token[8+i*3] = self.tracon.apt_ele[i]

        # Part2: the restricted areas (token1 to token20)
        res_tokens = np.zeros(shape=(20,20))
        for i in range(len(self.tracon.restrict)):
            res = self.tracon.restrict[i]
            # The first two entries denotes the type of the area: [1, 0] for circle, [0, 1] for polygon
            if res.area_type == "circle":
                res_tokens[i,0] = 1
            if res.area_type == "polygon":
                res_tokens[i,1] = 1
            # Area bottom and top alt
            res_tokens[i,2] = res.area_bottom_alt
            res_tokens[i,3] = res.area_top_alt
            # Area args
            for j in range(len(res.area_args)):
                res_tokens[i,4+j] = res.area_args[j]

        # Part3: the aircrafts (token 21 to token 70)
        acf_tokens = np.zeros(shape=(50,20))
        for i in range(len(self.aircrafts)):
            acf = self.aircrafts[i]
            # intention: [1, 0] for arrival, [0, 1] for departure
            if acf.intention == "arrival":
                acf_tokens[i,0] = 1
            if acf.intention == "departure":
                acf_tokens[i,1] = 1
            # under control: 1 for True, 0 for False
            acf_tokens[i,2] = acf.under_ctrl
            # how long stayed in airspace
            acf_tokens[i,3] = acf.time_stayed
            # how long since receiving last command
            if acf.time_since_last_command is not None:
                acf_tokens[i,4] = acf.time_since_last_command
            # LNAV/VNAV on: 1 for True, 0 for False
            acf_tokens[i,5] = acf.lnav_on
            # latitude, longitude
            acf_tokens[i,6] = acf.lat
            acf_tokens[i,7] = acf.lon
            # bearing from center, dist from center
            acf_tokens[i,8] = geo.bearing_from_to(self.tracon.ctr_lat, self.tracon.ctr_lon, acf.lat, acf.lon)
            acf_tokens[i,9] = geo.distance_between(self.tracon.ctr_lat, self.tracon.ctr_lon, acf.lat, acf.lon)
            # bearing to wpt, dist to wpt
            if acf.target_wp_lat is not None and acf.target_wp_lon is not None:
                acf_tokens[i,10] = geo.bearing_from_to(acf.lat, acf.lon, acf.target_wp_lat, acf.target_wp_lon)
                acf_tokens[i,11] = geo.distance_between(acf.lat, acf.lon, acf.target_wp_lat, acf.target_wp_lon)
            # target wpt altitude
            if acf.target_wp_alt is not None:
                acf_tokens[i,12] = acf.target_wp_alt
            # current alt, cas, trk, vs
            acf_tokens[i,13] = acf.curr_alt
            acf_tokens[i,14] = acf.curr_cas
            acf_tokens[i,15] = acf.curr_trk
            acf_tokens[i,16] = acf.curr_vs
            # command alt, cas, trk
            acf_tokens[i,17] = acf.target_alt if acf.target_alt else acf.curr_alt
            acf_tokens[i,18] = acf.target_cas if acf.target_cas else acf.curr_cas
            acf_tokens[i,19] = acf.target_trk if acf.target_trk else acf.curr_trk

        return [tracon_token, res_tokens, acf_tokens]



def polar_to_xOy(theta, rho, tracon_range, scale=400):
    ''' Convert bearing_from_center and dist_from_center to the xOy coordinate
     to display it. The default scale (half size of the image) is 400 (pixels) '''
    theta = -theta+90
    cx = scale + rho * np.cos(np.radians(theta)) * scale / tracon_range
    cy = scale - rho * np.sin(np.radians(theta)) * scale / tracon_range

    return cx, cy